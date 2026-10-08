"""Out-and-back over PX4's ROS 2 topics.

Publishes OffboardControlMode and TrajectorySetpoint, and uses
VehicleCommand to arm, switch to offboard, land, and disarm. Ground
truth is the Gazebo model pose. /ground_truth/odom is used instead
when that topic is already publishing.

This module imports rclpy only when a flight starts, so the crash
rules can be tested without a ROS install.
"""

from __future__ import annotations

import math
import os
import subprocess
import threading
import time

from crash_monitor import crash_reason, load_crash_thresholds
from grading import grade_mission, load_thresholds, yaw_from_quat
from camera_check import CameraTracker, rtf_summary
from gz_pose import (
    error_stats,
    gazebo_env,
    parse_pose_v,
    parse_rtf,
    px4_gt_error_m,
    setpoint_world,
    tilt_deg,
)


class FlightCrash(Exception):
    def __init__(self, reason: str, snapshot: dict | None = None, samples=None):
        super().__init__(reason)
        self.reason = reason
        self.snapshot = snapshot or {}
        self.samples = samples or []

    def payload(self, thresholds, driver: str) -> dict:
        data = {
            "status": "crashed",
            "passed": False,
            "driver": driver,
            "crash_reason": self.reason,
            "snapshot": self.snapshot,
            "checks": [],
            "samples_count": len(self.samples),
            "track": self.samples,
            "px4_position_error_m": error_stats(self.samples),
            "thresholds": _threshold_dump(thresholds),
        }
        extra = getattr(self, "extra", None)
        if isinstance(extra, dict):
            data.update(extra)
        return data


class FlightSetupError(Exception):
    """ROS or px4_msgs is not usable. Retrying the sim will not fix it."""


def _threshold_dump(thresholds) -> dict:
    if thresholds is None:
        return {}
    data = {}
    for field in (
        "takeoff_height_m",
        "hover_s",
        "leg_length_m",
        "hover_drift_m",
        "leg_tolerance_m",
        "yaw_tolerance_deg",
        "yaw_settle_deg",
        "return_tolerance_m",
        "height_tolerance_m",
        "overshoot_m",
        "settle_tolerance_m",
        "settle_hold_s",
        "settle_timeout_s",
        "hover_height_band_m",
        "hover_height_hold_s",
    ):
        if hasattr(thresholds, field):
            data[field] = getattr(thresholds, field)
    return data


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def read_gz_pose(world: str, model: str, timeout_s: float = 2.0):
    topic = f"/world/{world}/pose/info"
    try:
        proc = subprocess.run(
            ["gz", "topic", "-e", "-n", "1", "-t", topic],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            env=gazebo_env(),
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    return parse_pose_v((proc.stdout or "") + "\n" + (proc.stderr or ""), model)


def read_gz_rtf(world: str, timeout_s: float = 1.0):
    topic = f"/world/{world}/stats"
    try:
        proc = subprocess.run(
            ["gz", "topic", "-e", "-n", "1", "-t", topic],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            env=gazebo_env(),
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    return parse_rtf((proc.stdout or "") + "\n" + (proc.stderr or ""))


class FlightWatch:
    """Subscribe to PX4, sample Gazebo, and optionally fly the mission."""

    def __init__(self, thresholds=None, crash_limits=None, commanding: bool = True):
        self.thresholds = thresholds or load_thresholds()
        self.crash_limits = crash_limits or load_crash_thresholds()
        self.commanding = commanding
        self.phase = "preflight"
        self.samples = []
        self.crash = None
        self.snapshot = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._gz_thread = None
        self._node = None
        self._rclpy = None
        self._last_record_sim = -1.0
        self._origin = None
        self._gz = None
        self._gz_wall = None
        self._odom_msg = None
        self._px4_wall = None
        self.px4_n = 0.0
        self.px4_e = 0.0
        self.px4_d = 0.0
        self.px4_valid = False
        self.heading = None
        self.speed = None
        self.armed = None
        self.was_armed = False
        self.failsafe = False
        self.landed = False
        self.nav_state = None
        self.preflight_ok = False
        self.seen_status = False
        self.sp_n = 0.0
        self.sp_e = 0.0
        self.sp_d = 0.0
        self.yaw_sp = 0.0
        self.hold_captured = False
        self.ground_down = 0.0
        self.land_fallback = False
        self._phase_sim = None
        self._pubs = {}
        self._types = {}
        self.world = os.environ.get("World") or "default"
        self.model = os.environ.get("E2E_GZ_MODEL") or os.environ.get("PX4_GZ_MODEL") or "x500"
        scale = float(os.environ.get("E2E_WALL_SCALE") or "1")
        self.wall_scale = scale if scale > 0 else 1.0
        self.check_cameras = os.environ.get("E2E_CHECK_CAMERAS", "0") == "1"
        min_variance = float(os.environ.get("E2E_CAMERA_MIN_VARIANCE") or "1")
        self.cameras = CameraTracker(min_variance=min_variance) if self.check_cameras else None
        self.rtf_samples = []
        self._rtf_wall = 0.0

    def set_phase(self, phase: str) -> None:
        with self._lock:
            self.phase = phase
            self._phase_sim = None

    def raise_if_crashed(self) -> None:
        if self.crash:
            raise FlightCrash(self.crash, dict(self.snapshot), list(self.samples))

    def start(self) -> None:
        self._open_ros()
        self._thread = threading.Thread(target=self._spin_monitor, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._close_ros()

    def run(self) -> dict:
        """Fly one attempt. Used when px4_control.Drone is not installed."""

        try:
            self._open_ros()
            self._wait_for_estimate()
            self.set_phase("start")
            self._wait_sim(1.0, 30.0)
            self._arm()
            self._engage_offboard()
            self._takeoff()
            # Height has already held. This wait is the hover clock, not a settle.
            self.set_phase("hover")
            self._wait_sim(self.thresholds.hover_s, max(60.0, self.thresholds.hover_s * 8.0))
            self._leg("leg1")
            self._yaw_180()
            self._leg("leg2")
            self._land()
            self._disarm()
        except FlightCrash:
            raise
        finally:
            self._close_ros()
        return self._result(crashed=False)

    @staticmethod
    def _subscribe(node, msg_type, topics, callback, qos) -> None:
        for topic in topics:
            node.create_subscription(msg_type, topic, callback, qos)

    def _open_ros(self) -> None:
        try:
            import rclpy
            from rclpy.parameter import Parameter
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        except ImportError as exc:
            raise FlightSetupError(f"rclpy is not importable ({exc})") from exc
        try:
            from px4_msgs.msg import (
                OffboardControlMode,
                TrajectorySetpoint,
                VehicleAttitude,
                VehicleCommand,
                VehicleLandDetected,
                VehicleLocalPosition,
                VehicleStatus,
            )
        except ImportError as exc:
            raise FlightSetupError(
                f"px4_msgs is not importable ({exc}). Source the PX4 workspace."
            ) from exc

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        rclpy.init(args=None)
        node = rclpy.create_node(
            "e2e_offboard",
            enable_rosout=False,
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self._rclpy = rclpy
        self._node = node
        self._types = {
            "status": VehicleStatus,
            "command": VehicleCommand,
            "offboard": OffboardControlMode,
            "setpoint": TrajectorySetpoint,
            "armed": getattr(VehicleStatus, "ARMING_STATE_ARMED", 2),
            "offboard_mode": getattr(VehicleStatus, "NAVIGATION_STATE_OFFBOARD", 14),
            "arm_cmd": getattr(VehicleCommand, "VEHICLE_CMD_COMPONENT_ARM_DISARM", 400),
            "mode_cmd": getattr(VehicleCommand, "VEHICLE_CMD_DO_SET_MODE", 176),
            "land_cmd": getattr(VehicleCommand, "VEHICLE_CMD_NAV_LAND", 21),
        }
        # PX4 1.17 publishes _vN only when MESSAGE_VERSION is non-zero.
        # vehicle_status and vehicle_local_position are version 1.
        # Attitude, land, and the inbound command topics are version 0,
        # so the plain name is what the autopilot uses. Both names are
        # bound so a publisher on either one is enough.
        self._subscribe(node, VehicleStatus, (
            "/fmu/out/vehicle_status_v1",
            "/fmu/out/vehicle_status",
        ), self._on_status, qos_sub)
        self._subscribe(node, VehicleLocalPosition, (
            "/fmu/out/vehicle_local_position_v1",
            "/fmu/out/vehicle_local_position",
        ), self._on_local, qos_sub)
        self._subscribe(node, VehicleLandDetected, (
            "/fmu/out/vehicle_land_detected_v1",
            "/fmu/out/vehicle_land_detected",
        ), self._on_landed, qos_sub)
        self._subscribe(node, VehicleAttitude, (
            "/fmu/out/vehicle_attitude_v1",
            "/fmu/out/vehicle_attitude",
        ), self._on_attitude, qos_sub)
        try:
            from nav_msgs.msg import Odometry

            node.create_subscription(Odometry, "/ground_truth/odom", self._on_odom, qos_sub)
        except ImportError:
            pass
        if self.cameras is not None:
            from sensor_msgs.msg import Image

            for topic in self.cameras.required:
                node.create_subscription(
                    Image,
                    topic,
                    lambda msg, name=topic: self._on_image(name, msg),
                    qos_sub,
                )
        self._gz_thread = threading.Thread(target=self._gz_loop, daemon=True)
        self._gz_thread.start()
        if self.commanding:
            self._pubs["offboard"] = node.create_publisher(
                OffboardControlMode, "/fmu/in/offboard_control_mode", qos_pub
            )
            self._pubs["setpoint"] = node.create_publisher(
                TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_pub
            )
            self._pubs["command"] = node.create_publisher(
                VehicleCommand, "/fmu/in/vehicle_command", qos_pub
            )
        else:
            node.create_subscription(
                TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self._on_setpoint, qos_sub
            )

    def _close_ros(self) -> None:
        self._stop.set()
        if self._gz_thread is not None:
            self._gz_thread.join(timeout=3.0)
            self._gz_thread = None
        node = self._node
        rclpy = self._rclpy
        self._node = None
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        self._rclpy = None

    def _gz_loop(self) -> None:
        while not self._stop.is_set():
            if self._odom_msg is None:
                pose = read_gz_pose(self.world, self.model, timeout_s=1.0)
                if pose is not None:
                    self._gz = pose
                    self._gz_wall = time.monotonic()
            now = time.monotonic()
            if now - self._rtf_wall >= 5.0:
                self._rtf_wall = now
                rtf = read_gz_rtf(self.world, timeout_s=1.0)
                if rtf is not None:
                    self.rtf_samples.append(rtf)
            time.sleep(0.05)

    def _on_image(self, topic: str, msg) -> None:
        if self.cameras is None:
            return
        self.cameras.add(
            topic,
            str(getattr(msg, "encoding", "")),
            int(getattr(msg, "width", 0)),
            int(getattr(msg, "height", 0)),
            bytes(getattr(msg, "data", b"")),
            time.monotonic(),
        )

    def sensor_fields(self) -> dict:
        fields = {"gz_rtf": rtf_summary(self.rtf_samples)}
        if self.cameras is None:
            return fields
        report = self.cameras.report()
        fields["cameras"] = report
        fields["camera_reason"] = report.get("reason")
        return fields

    def _spin_monitor(self) -> None:
        while not self._stop.is_set():
            try:
                self._tick()
            except FlightCrash as exc:
                self.crash = exc.reason
                self.snapshot = exc.snapshot
                return
            time.sleep(0.05)

    def _on_status(self, msg) -> None:
        self.seen_status = True
        self.nav_state = int(msg.nav_state)
        armed = int(msg.arming_state) == int(self._types["armed"])
        self.armed = armed
        if armed:
            self.was_armed = True
        self.failsafe = bool(getattr(msg, "failsafe", False))
        self.preflight_ok = bool(getattr(msg, "pre_flight_checks_pass", False))

    def _on_local(self, msg) -> None:
        self.px4_n = float(msg.x)
        self.px4_e = float(msg.y)
        self.px4_d = float(msg.z)
        xy_ok = bool(getattr(msg, "xy_valid", True))
        z_ok = bool(getattr(msg, "z_valid", True))
        self.px4_valid = xy_ok and z_ok
        heading = getattr(msg, "heading", None)
        if heading is not None and math.isfinite(float(heading)):
            self.heading = float(heading)
        speed_sq = 0.0
        finite = True
        for value in (getattr(msg, "vx", None), getattr(msg, "vy", None), getattr(msg, "vz", None)):
            if value is None or not math.isfinite(float(value)):
                finite = False
                break
            speed_sq += float(value) ** 2
        if finite:
            self.speed = math.sqrt(speed_sq)
        self._px4_wall = time.monotonic()
        if self.px4_valid and not self.hold_captured and self.commanding:
            self.sp_n = self.px4_n
            self.sp_e = self.px4_e
            self.sp_d = self.px4_d
            self.ground_down = self.px4_d
            if self.heading is not None:
                self.yaw_sp = self.heading
            self.hold_captured = True

    def _on_landed(self, msg) -> None:
        self.landed = bool(getattr(msg, "landed", False))

    def _on_attitude(self, msg) -> None:
        quat = list(getattr(msg, "q", []))
        if len(quat) < 4:
            return
        # PX4 stores the attitude quaternion as w, x, y, z.
        self._attitude_tilt = tilt_deg(float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0]))

    def _on_odom(self, msg) -> None:
        pose = msg.pose.pose
        quat = pose.orientation
        self._odom_msg = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "qx": float(quat.x),
            "qy": float(quat.y),
            "qz": float(quat.z),
            "qw": float(quat.w),
            "yaw": yaw_from_quat(quat.x, quat.y, quat.z, quat.w),
            "tilt_deg": tilt_deg(quat.x, quat.y, quat.z, quat.w),
            "name": "ground_truth",
        }

    def _on_setpoint(self, msg) -> None:
        position = list(msg.position)
        if len(position) >= 3 and all(math.isfinite(float(item)) for item in position[:3]):
            self.sp_n = float(position[0])
            self.sp_e = float(position[1])
            self.sp_d = float(position[2])

    def _sim_s(self) -> float:
        if self._node is None:
            return 0.0
        return self._node.get_clock().now().nanoseconds / 1e9

    def _tick(self) -> None:
        self._refresh_gz()
        if self.commanding and self.phase not in {"land", "disarm"}:
            self._publish_setpoint()
            self._publish_offboard()
        if self.commanding and self.phase == "land" and self.land_fallback:
            self._publish_setpoint()
            self._publish_offboard()
        if self.commanding:
            self._send_phase_command()
        if self._node is not None and self._rclpy is not None:
            self._rclpy.spin_once(self._node, timeout_sec=0.0)
        self._observe()

    def _refresh_gz(self) -> None:
        # /ground_truth/odom wins when that topic is already publishing.
        if self._odom_msg is not None:
            self._gz = dict(self._odom_msg)
            self._gz_wall = time.monotonic()

    def _publish_offboard(self) -> None:
        pub = self._pubs.get("offboard")
        if pub is None:
            return
        msg = self._types["offboard"]()
        msg.timestamp = self._stamp()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        if hasattr(msg, "body_rate"):
            msg.body_rate = False
        pub.publish(msg)

    def _publish_setpoint(self) -> None:
        pub = self._pubs.get("setpoint")
        if pub is None:
            return
        msg = self._types["setpoint"]()
        msg.timestamp = self._stamp()
        msg.position[0] = float(self.sp_n)
        msg.position[1] = float(self.sp_e)
        msg.position[2] = float(self.sp_d)
        for index in range(3):
            msg.velocity[index] = float("nan")
            msg.acceleration[index] = float("nan")
        msg.yaw = float(self.yaw_sp)
        if hasattr(msg, "yawspeed"):
            msg.yawspeed = float("nan")
        pub.publish(msg)

    def _send_phase_command(self) -> None:
        if self.phase == "arm" and not self.armed:
            self._send_command(self._types["arm_cmd"], 1.0)
        elif self.phase == "offboard" and self.nav_state != self._types["offboard_mode"]:
            self._send_command(self._types["mode_cmd"], 1.0, 6.0)
        elif self.phase == "land" and not self.land_fallback:
            self._send_command(self._types["land_cmd"], 0.0)
        elif self.phase == "disarm":
            self._send_command(self._types["arm_cmd"], 0.0)

    def _send_command(self, command: int, param1: float, param2: float = 0.0) -> None:
        pub = self._pubs.get("command")
        if pub is None:
            return
        msg = self._types["command"]()
        msg.timestamp = self._stamp()
        msg.command = int(command)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        pub.publish(msg)

    def _stamp(self) -> int:
        if self._node is None:
            return 0
        return int(self._node.get_clock().now().nanoseconds / 1000)

    def _observe(self) -> None:
        now = time.monotonic()
        gz = self._gz
        # The last pose before takeoff is the ground origin.
        if gz is not None and self.phase == "start":
            self._origin = (gz["x"], gz["y"], gz["z"])
        elif gz is not None and self._origin is None:
            self._origin = (gz["x"], gz["y"], gz["z"])
        height = None
        tilt = getattr(self, "_attitude_tilt", None)
        setpoint_error = None
        if gz is not None:
            origin = self._origin or (gz["x"], gz["y"], gz["z"])
            height = gz["z"] - origin[2]
            tilt = gz["tilt_deg"]
            expected = setpoint_world(self.sp_n, self.sp_e, self.sp_d, origin)
            setpoint_error = math.dist((gz["x"], gz["y"], gz["z"]), expected)
        ages = []
        if self._gz_wall is not None:
            ages.append(now - self._gz_wall)
        if self._px4_wall is not None:
            ages.append(now - self._px4_wall)
        odom_age = min(ages) if ages else None
        obs = {
            "phase": self.phase,
            "tilt_deg": tilt,
            "height_m": height,
            "speed_mps": self.speed,
            "setpoint_error_m": setpoint_error,
            "odom_age_s": odom_age,
            "seen_odom": bool(ages),
            "armed": self.armed,
            "was_armed": self.was_armed,
            "failsafe": self.failsafe,
            "landed": self.landed,
        }
        reason = crash_reason(obs, self.crash_limits)
        self.snapshot = {
            "phase": self.phase,
            "height_m": height,
            "tilt_deg": tilt,
            "armed": self.armed,
            "failsafe": self.failsafe,
            "landed": self.landed,
            "nav_state": self.nav_state,
            "setpoint_error_m": None if setpoint_error is None else round(setpoint_error, 4),
        }
        if reason:
            self.crash = reason
            raise FlightCrash(reason, dict(self.snapshot), list(self.samples))
        self._maybe_record(gz, height, setpoint_error)

    def _maybe_record(self, gz, height, setpoint_error) -> None:
        if gz is None:
            return
        sim = self._sim_s()
        if sim - self._last_record_sim < 0.1 and self.samples:
            return
        self._last_record_sim = sim
        row = {
            "t": round(sim, 3),
            "phase": self.phase,
            "x": gz["x"],
            "y": gz["y"],
            "z": gz["z"],
            "yaw": gz["yaw"],
            "height_m": None if height is None else round(height, 4),
        }
        if self.px4_valid and self._origin is not None:
            row["px4_n"] = self.px4_n
            row["px4_e"] = self.px4_e
            row["px4_d"] = self.px4_d
            row["px4_gt_error_m"] = round(
                px4_gt_error_m(
                    (self.px4_n, self.px4_e, self.px4_d),
                    (gz["x"], gz["y"], gz["z"]),
                    self._origin,
                ),
                4,
            )
        elif setpoint_error is not None:
            row["px4_gt_error_m"] = None
        self.samples.append(row)

    def _wait_sim(self, sim_timeout: float, wall_timeout: float) -> bool:
        finished = self._wait_until(lambda: False, sim_timeout, wall_timeout, finish_on_sim=True)
        if not finished:
            raise FlightCrash("sim_stalled", dict(self.snapshot), list(self.samples))
        return True

    def _wait_until(self, predicate, sim_timeout: float, wall_timeout: float, finish_on_sim: bool = False) -> bool:
        wall_timeout = wall_timeout * self.wall_scale
        start_sim = self._sim_s()
        start_wall = time.monotonic()
        if self._phase_sim is None:
            self._phase_sim = start_sim
        while True:
            self._tick()
            if predicate():
                return True
            sim_dt = self._sim_s() - start_sim
            wall_dt = time.monotonic() - start_wall
            if wall_dt > 20.0 and sim_dt < 0.5:
                raise FlightCrash("sim_stalled", dict(self.snapshot), list(self.samples))
            if finish_on_sim and sim_dt >= sim_timeout:
                return True
            if (not finish_on_sim) and (sim_dt >= sim_timeout or wall_dt >= wall_timeout):
                return False
            if wall_dt >= wall_timeout:
                return False
            time.sleep(0.05)

    def _wait_for_estimate(self) -> None:
        print("phase=preflight", flush=True)
        ok = self._wait_until(
            lambda: self.px4_valid and self._gz is not None and (self.preflight_ok or self.seen_status),
            sim_timeout=40.0,
            wall_timeout=180.0,
        )
        if not ok:
            raise FlightCrash("preflight_timeout", dict(self.snapshot), list(self.samples))

    def _arm(self) -> None:
        self.set_phase("arm")
        print("phase=arm", flush=True)
        ok = self._wait_until(lambda: bool(self.armed), sim_timeout=20.0, wall_timeout=90.0)
        if not ok:
            raise FlightCrash("arm_timeout", dict(self.snapshot), list(self.samples))

    def _engage_offboard(self) -> None:
        self.set_phase("offboard")
        print("phase=offboard", flush=True)
        ok = self._wait_until(
            lambda: self.nav_state == self._types["offboard_mode"],
            sim_timeout=15.0,
            wall_timeout=60.0,
        )
        if not ok:
            raise FlightCrash("offboard_timeout", dict(self.snapshot), list(self.samples))

    def _height_m(self):
        gz = self._gz
        if gz is None or self._origin is None:
            return None
        return gz["z"] - self._origin[2]

    def _position_error_m(self):
        gz = self._gz
        if gz is None or self._origin is None:
            return None
        expected = setpoint_world(self.sp_n, self.sp_e, self.sp_d, self._origin)
        return math.dist((gz["x"], gz["y"], gz["z"]), expected)

    def _yaw_error_deg(self):
        if self.heading is None:
            return None
        return abs(math.degrees(wrap_pi(self.heading - self.yaw_sp)))

    def _wait_hold(self, inside, hold_s: float, sim_timeout: float, wall_timeout: float, label: str) -> bool:
        """Stay inside `inside` for hold_s of sim time. A miss restarts the hold."""

        hold_start = None

        def held():
            nonlocal hold_start
            if not inside():
                hold_start = None
                return False
            now = self._sim_s()
            if hold_start is None:
                hold_start = now
            return (now - hold_start) >= hold_s

        ok = self._wait_until(held, sim_timeout=sim_timeout, wall_timeout=wall_timeout)
        if not ok:
            print(f"phase={self.phase} {label} not settled", flush=True)
        return ok

    def _wait_step_settle(self, error_fn, tolerance: float) -> bool:
        hold = self.thresholds.settle_hold_s
        timeout = self.thresholds.settle_timeout_s

        def inside():
            error = error_fn()
            return error is not None and error <= tolerance

        return self._wait_hold(inside, hold, timeout, max(45.0, timeout * 10.0), "settle")

    def _takeoff(self) -> None:
        self.set_phase("takeoff")
        print("phase=takeoff", flush=True)
        self.sp_d = self.ground_down - self.thresholds.takeoff_height_m
        band = self.thresholds.hover_height_band_m
        target = self.thresholds.takeoff_height_m

        def at_height():
            height = self._height_m()
            return height is not None and abs(height - target) <= band

        # Climb is not a 4 s step. The hover clock starts after this hold.
        self._wait_hold(
            at_height,
            self.thresholds.hover_height_hold_s,
            sim_timeout=40.0,
            wall_timeout=180.0,
            label="height",
        )

    def _leg(self, phase: str) -> None:
        self.set_phase(phase)
        print(f"phase={phase}", flush=True)
        heading = self.heading if self.heading is not None else self.yaw_sp
        distance = self.thresholds.leg_length_m
        self.sp_n += distance * math.cos(heading)
        self.sp_e += distance * math.sin(heading)
        self._wait_step_settle(self._position_error_m, self.thresholds.settle_tolerance_m)

    def _yaw_180(self) -> None:
        self.set_phase("yaw")
        print("phase=yaw", flush=True)
        self.yaw_sp = wrap_pi(self.yaw_sp + math.pi)
        self._wait_step_settle(self._yaw_error_deg, self.thresholds.yaw_settle_deg)

    def _land(self) -> None:
        self.set_phase("land")
        print("phase=land", flush=True)
        start_height = 0.0
        if self.samples and self.samples[-1].get("height_m") is not None:
            start_height = self.samples[-1]["height_m"]
        start_sim = self._sim_s()

        def down():
            if self.landed:
                return True
            if not self.samples:
                return False
            height = self.samples[-1].get("height_m")
            if height is None:
                return False
            if (self._sim_s() - start_sim) > 8.0 and height > start_height - 0.3:
                self.land_fallback = True
                self.sp_d = self.ground_down
            return height <= self.crash_limits.min_height_m

        self._wait_until(down, sim_timeout=40.0, wall_timeout=180.0)

    def _disarm(self) -> None:
        self.set_phase("disarm")
        print("phase=disarm", flush=True)
        self._wait_until(lambda: self.armed is False, sim_timeout=15.0, wall_timeout=60.0)

    def _result(self, crashed: bool) -> dict:
        graded = grade_mission(self.samples, self.thresholds)
        graded["status"] = "passed" if graded["passed"] else "failed"
        graded["driver"] = "px4_offboard"
        graded["samples_count"] = len(self.samples)
        graded["track"] = self.samples
        graded["px4_position_error_m"] = error_stats(self.samples)
        graded["crash_reason"] = self.crash
        graded.update(self.sensor_fields())
        camera_report = graded.get("cameras") or {}
        if self.cameras is not None and not camera_report.get("passed") and not crashed:
            graded["status"] = "failed"
            graded["passed"] = False
            graded["checks"] = list(graded.get("checks") or []) + list(camera_report.get("checks") or [])
        if crashed:
            graded["status"] = "crashed"
            graded["passed"] = False
        return graded


def run_px4_mission(thresholds=None) -> dict:
    watch = FlightWatch(thresholds=thresholds, commanding=True)
    try:
        return watch.run()
    except FlightCrash as exc:
        exc.samples = watch.samples
        exc.extra = watch.sensor_fields()
        raise
