"""Live recorder. Timestamps come from the node clock (sim time when enabled)."""

from __future__ import annotations

import numpy as np

from trajectory_eval.frames import geodetic_to_enu, gps_fields_to_lla, ned_frd_to_enu_flu, quat_xyzw_to_wxyz
from trajectory_eval.metrics import PoseSample


def record_live(args) -> dict[str, list[PoseSample]]:
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message

    class Recorder(Node):
        def __init__(self) -> None:
            from rclpy.parameter import Parameter
            super().__init__(
                "trajectory_eval",
                parameter_overrides=[
                    Parameter("use_sim_time", Parameter.Type.BOOL, bool(args.use_sim_time)),
                ],
            )
            self.gt: list[PoseSample] = []
            self.rtabmap: list[PoseSample] = []
            self.ekf: list[PoseSample] = []
            self.gps_raw: list[tuple[float, float, float, float]] = []
            self._subscribed: set[str] = set()
            self.start_time = None
            # Wall clock. A sim-time timer slows with RTF and stops if /clock stalls.
            self.create_timer(
                1.0, self._discover, clock=Clock(clock_type=ClockType.STEADY_TIME),
            )
            self._discover()

        def _now(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def _stamp(self, sample_time: float) -> float:
            if self.start_time is None:
                self.start_time = sample_time
            return sample_time

        def _discover(self) -> None:
            topics = dict(self.get_topic_names_and_types())
            self._subscribe_exact(topics, args.gt_topic, self._on_odom_gt)
            self._subscribe_exact(topics, args.rtabmap_topic, self._on_odom_rtab)
            ekf = _resolve_px4_topic(topics, args.ekf_topic, "vehicle_odometry")
            gps = _resolve_px4_topic(topics, args.gps_topic, "vehicle_gps_position")
            if ekf:
                self._subscribe_exact(topics, ekf, self._on_ekf)
            if gps:
                self._subscribe_exact(topics, gps, self._on_gps)

        def _subscribe_exact(self, topics, name, callback) -> None:
            if name in self._subscribed or name not in topics:
                return
            message_type = get_message(topics[name][0])
            self.create_subscription(message_type, name, callback, qos_profile_sensor_data)
            self._subscribed.add(name)
            self.get_logger().info(f"recording {name} ({topics[name][0]})")

        def _on_odom_gt(self, msg) -> None:
            self.gt.append(_odom_sample(msg, self._stamp(self._now())))

        def _on_odom_rtab(self, msg) -> None:
            self.rtabmap.append(_odom_sample(msg, self._stamp(self._now())))

        def _on_ekf(self, msg) -> None:
            position = np.array(list(msg.position), dtype=float)
            quat = np.array(list(msg.q), dtype=float)
            position_enu, quat_enu = ned_frd_to_enu_flu(position, quat)
            self.ekf.append(PoseSample(self._stamp(self._now()), position_enu, quat_enu))

        def _on_gps(self, msg) -> None:
            lla = gps_fields_to_lla(msg)
            if lla is None:
                return
            self.gps_raw.append((self._stamp(self._now()), *lla))

        def finish(self) -> dict[str, list[PoseSample]]:
            gps: list[PoseSample] = []
            if self.gps_raw:
                _, lat0, lon0, alt0 = self.gps_raw[0]
                for stamp, lat, lon, alt in self.gps_raw:
                    enu = geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0)
                    gps.append(PoseSample(stamp, enu, np.array([1.0, 0.0, 0.0, 0.0])))
            return {
                "ground_truth": self.gt,
                "gps": gps,
                "rtabmap": self.rtabmap,
                "ekf2": self.ekf,
            }

    rclpy.init()
    node = Recorder()
    try:
        if args.duration and args.duration > 0.0:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.2)
                if node.start_time is None:
                    continue
                if node._now() - node.start_time >= args.duration:
                    break
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    streams = node.finish()
    node.destroy_node()
    rclpy.shutdown()
    return streams


def _odom_sample(msg, stamp: float) -> PoseSample:
    position = msg.pose.pose.position
    quat = msg.pose.pose.orientation
    return PoseSample(
        stamp,
        np.array([position.x, position.y, position.z]),
        quat_xyzw_to_wxyz(np.array([quat.x, quat.y, quat.z, quat.w])),
    )


def _resolve_px4_topic(topics: dict, requested: str, suffix: str) -> str | None:
    if requested in topics:
        return requested
    versioned = []
    exact = []
    for name in topics:
        tail = name.rstrip("/").split("/")[-1]
        if tail == suffix:
            exact.append(name)
        prefix = suffix + "_v"
        if tail.startswith(prefix) and tail[len(prefix):].isdigit():
            versioned.append((int(tail[len(prefix):]), name))
    if versioned:
        return sorted(versioned)[-1][1]
    if exact:
        return exact[0]
    return None
