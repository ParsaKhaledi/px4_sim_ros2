"""Service /sim/preflight_check (std_srvs/Trigger).

Success is true only when every check passes. The message is one reason per
line, each starting with PASS or FAIL. Thresholds come from the environment.
"""

from __future__ import annotations

import os
import time

from sim_monitor.imu_source import default_tf_pairs, optical_frame
from sim_monitor.checks import (
    RateTracker,
    check_max,
    check_min,
    check_rate,
    default_min_rtf,
    gated_rate_hz,
    header_stamp_s,
    line,
    match_versioned_topic,
    minimum_rate_hz,
    relative_position_error,
    summarize,
)


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name, "")
    if raw == "":
        return default
    return float(raw)


def main() -> None:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rosidl_runtime_py.utilities import get_message
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener

    class Preflight(Node):
        def __init__(self) -> None:
            use_sim = os.environ.get("USE_SIM_TIME", "true").lower() in {"1", "true", "yes"}
            super().__init__(
                "preflight_check",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, use_sim)],
            )
            self.min_rtf = default_min_rtf()
            self.max_pose = _env_float("PREFLIGHT_MAX_POSE_ERR_M", 0.10)
            self.min_camera = minimum_rate_hz("camera")
            self.min_imu = minimum_rate_hz("imu")
            self.min_gt = _env_float("PREFLIGHT_MIN_GT_HZ", 20.0)
            self.clock_max_age = _env_float("PREFLIGHT_CLOCK_MAX_AGE_S", 1.0)
            self.camera_topic = os.environ.get("PREFLIGHT_CAMERA_TOPIC", "/camera/rgb/image_raw")
            self.imu_topic = os.environ.get("PREFLIGHT_IMU_TOPIC", "/imu")
            self.gt_topic = os.environ.get("PREFLIGHT_GT_TOPIC", "/ground_truth/odom")
            self.rtab_topic = os.environ.get("PREFLIGHT_RTABMAP_ODOM_TOPIC", "/rtabmap/odom")
            self.rtab_info_topic = os.environ.get("PREFLIGHT_RTABMAP_INFO_TOPIC", "/rtabmap/odom_info")
            self.tf_pairs = default_tf_pairs()
            self.rates = {
                self.camera_topic: RateTracker(2.0),
                self.imu_topic: RateTracker(2.0),
                self.gt_topic: RateTracker(2.0),
                "/sim/real_time_factor": RateTracker(2.0),
            }
            self.rtf = None
            self.clock_wall = None
            self.clock_sim = None
            self.previous_clock_sim = None
            self.clock_advanced = False
            self.gt_origin = None
            self.gt_position = None
            self.rtab_origin = None
            self.rtab_position = None
            self.rtab_lost = None
            self.preflight_pass = None
            self.vision_flags = None
            self.status_topic = None
            self.flags_topic = None
            self.imu_frame_id = ""
            self._subscribed = set()
            self.type_errors: dict[str, str] = {}
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_service(Trigger, "/sim/preflight_check", self._handle)
            self.create_timer(1.0, self._discover)
            self._discover()
            self.get_logger().info("serving /sim/preflight_check")

        def _discover(self) -> None:
            try:
                names_and_types = dict(self.get_topic_names_and_types())
                for topic in (self.camera_topic, self.imu_topic, self.gt_topic, self.rtab_topic, "/clock", "/sim/real_time_factor"):
                    self._subscribe(names_and_types, topic)
                self._subscribe(names_and_types, self.rtab_info_topic)
                names = list(names_and_types)
                status = match_versioned_topic(names, "vehicle_status")
                flags = match_versioned_topic(names, "estimator_status_flags")
                if status:
                    self.status_topic = status
                    self._subscribe(names_and_types, status)
                if flags:
                    self.flags_topic = flags
                    self._subscribe(names_and_types, flags)
            except Exception as exc:
                self.get_logger().error(f"topic discovery failed: {exc}")

        def _subscribe(self, names_and_types, topic: str) -> None:
            if topic in self._subscribed or topic in self.type_errors or topic not in names_and_types:
                return
            try:
                message_type = get_message(names_and_types[topic][0])
            except Exception as exc:
                self.type_errors[topic] = str(exc)
                self.get_logger().error(f"cannot load message type for {topic}: {exc}")
                return
            self.create_subscription(
                message_type, topic, lambda msg, topic=topic: self._on_msg(topic, msg), qos_profile_sensor_data,
            )
            self._subscribed.add(topic)

        def _on_msg(self, topic: str, msg) -> None:
            now = time.monotonic()
            if topic in self.rates:
                self.rates[topic].add(now, header_stamp_s(msg))
            if topic == self.imu_topic:
                header = getattr(msg, "header", None)
                frame = getattr(header, "frame_id", "") if header is not None else ""
                if frame:
                    self.imu_frame_id = frame
            if topic == "/sim/real_time_factor":
                self.rtf = float(msg.data)
            elif topic == "/clock":
                self.clock_wall = now
                sim = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
                if self.clock_sim is not None and sim > self.clock_sim:
                    self.clock_advanced = True
                self.previous_clock_sim = self.clock_sim
                self.clock_sim = sim
            elif topic == self.gt_topic:
                position = _xyz(msg.pose.pose.position)
                if self.gt_origin is None:
                    self.gt_origin = position
                self.gt_position = position
            elif topic == self.rtab_topic:
                position = _xyz(msg.pose.pose.position)
                if self.rtab_origin is None:
                    self.rtab_origin = position
                self.rtab_position = position
            elif topic == self.rtab_info_topic:
                if hasattr(msg, "lost"):
                    self.rtab_lost = bool(msg.lost)
            elif topic == self.status_topic:
                if hasattr(msg, "pre_flight_checks_pass"):
                    self.preflight_pass = bool(msg.pre_flight_checks_pass)
            elif topic == self.flags_topic:
                self.vision_flags = {
                    "cs_ev_pos": bool(getattr(msg, "cs_ev_pos", False)),
                    "cs_ev_vel": bool(getattr(msg, "cs_ev_vel", False)),
                    "cs_ev_hgt": bool(getattr(msg, "cs_ev_hgt", False)),
                    "cs_ev_yaw": bool(getattr(msg, "cs_ev_yaw", False)),
                }

        def _handle(self, _request, response):
            now = time.monotonic()
            results = []
            if self.rtf is None:
                results.append((False, line(False, "real_time_factor: no messages on /sim/real_time_factor")))
            else:
                results.append(check_min("real_time_factor", self.rtf, self.min_rtf, ""))
            if self.clock_wall is None:
                results.append((False, line(False, "clock: /clock is not publishing")))
            else:
                age = now - self.clock_wall
                results.append(check_max("clock age", age, self.clock_max_age, "s"))
                results.append((
                    self.clock_advanced,
                    line(self.clock_advanced, "clock: sim time is advancing" if self.clock_advanced else "clock: sim time is not advancing"),
                ))
            results.append(self._rate(self.camera_topic, self.min_camera, now))
            results.append(self._rate(self.imu_topic, self.min_imu, now))
            results.append(self._imu_to_optical())
            results.append(self._rate(self.gt_topic, self.min_gt, now))
            results.extend(self._rtabmap())
            results.extend(self._px4())
            results.extend(self._tf())
            response.success, response.message = summarize(results)
            self.get_logger().info("preflight %s\n%s" % ("OK" if response.success else "FAILED", response.message))
            return response

        def _rate(self, topic: str, minimum: float, now: float):
            if topic in self.type_errors:
                return False, line(False, f"{topic}: message type unavailable ({self.type_errors[topic]})")
            tracker = self.rates.get(topic)
            wall_hz = tracker.hz_wall(now) if tracker else 0.0
            sim_hz = None
            if tracker is not None and len(tracker.sim_times) >= 2:
                sim_hz = tracker.hz_sim(tracker.sim_times[-1])
            gated = gated_rate_hz(sim_hz, wall_hz, self.rtf)
            return check_rate(topic, gated, wall_hz, minimum)

        def _imu_to_optical(self):
            target = optical_frame()
            frame = self.imu_frame_id
            if not frame:
                return False, line(False, "imu frame: /imu has no frame_id yet")
            try:
                ok = self.tf_buffer.can_transform(
                    target, frame, Time(), timeout=Duration(seconds=0.2),
                )
            except Exception as exc:
                return False, line(False, f"imu frame: {frame} -> {target} ({exc})")
            return ok, line(ok, f"imu frame: {frame} -> {target}")

        def _rtabmap(self):
            if self.rtab_lost is None:
                return [(False, line(False, f"rtabmap tracking: {self.rtab_info_topic} has no 'lost' field yet"))]
            tracking_ok = not self.rtab_lost
            lines = [(tracking_ok, line(tracking_ok, "rtabmap tracking: lost=false" if tracking_ok else "rtabmap tracking: lost=true"))]
            if None in (self.gt_origin, self.gt_position, self.rtab_origin, self.rtab_position):
                lines.append((False, line(False, "rtabmap pose: ground truth or /rtabmap/odom missing")))
                return lines
            error = relative_position_error(self.rtab_position, self.rtab_origin, self.gt_position, self.gt_origin)
            lines.append(check_max("rtabmap pose error relative to start", error, self.max_pose, "m"))
            return lines

        def _px4(self):
            lines = []
            if self.status_topic and self.status_topic in self.type_errors:
                lines.append((False, line(
                    False,
                    f"px4 pre-arm: message type unavailable ({self.type_errors[self.status_topic]})",
                )))
            elif self.preflight_pass is None:
                watched = self.status_topic or "vehicle_status(_vN)"
                lines.append((False, line(False, f"px4 pre-arm: no pre_flight_checks_pass on {watched}")))
            else:
                lines.append((
                    self.preflight_pass,
                    line(self.preflight_pass, "px4 pre-arm: pre_flight_checks_pass=true" if self.preflight_pass else "px4 pre-arm: pre_flight_checks_pass=false"),
                ))
            if self.flags_topic and self.flags_topic in self.type_errors:
                lines.append((False, line(
                    False,
                    f"ekf2 external vision: message type unavailable ({self.type_errors[self.flags_topic]})",
                )))
            elif not self.vision_flags:
                watched = self.flags_topic or "estimator_status_flags(_vN)"
                lines.append((False, line(False, f"ekf2 external vision: no flags on {watched}")))
            else:
                active = [name for name, value in self.vision_flags.items() if value]
                ok = len(active) > 0
                detail = ",".join(active) if active else "none"
                lines.append((ok, line(ok, f"ekf2 external vision: fusing {detail}")))
            return lines

        def _tf(self):
            lines = []
            for pair in self.tf_pairs.split(","):
                pair = pair.strip()
                if not pair or ":" not in pair:
                    continue
                parent, child = pair.split(":", 1)
                try:
                    ok = self.tf_buffer.can_transform(parent, child, Time(), timeout=Duration(seconds=0.2))
                except Exception as exc:
                    ok = False
                    lines.append((False, line(False, f"tf {parent} -> {child} ({exc})")))
                    continue
                lines.append((ok, line(ok, f"tf {parent} -> {child}")))
            if not lines:
                lines.append((False, line(False, "tf: PREFLIGHT_TF_PAIRS is empty")))
            return lines

    rclpy.init()
    node = Preflight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


def _xyz(point) -> tuple[float, float, float]:
    return (float(point.x), float(point.y), float(point.z))


if __name__ == "__main__":
    main()
