"""TEST SUBSTITUTE for RTAB-Map visual odometry.

Reads Gazebo ``/ground_truth/odom`` and publishes ``/rtabmap/odom`` plus the
``odom`` -> ``base_link`` transform RTAB-Map would publish. It does not look
at images. The odom origin is the spawn position. Orientation stays in
world ENU so the vision bridge's yaw map (ENU psi -> NED pi/2 - psi) is
what the estimator sees. ``RGBD/StartAtOrigin`` on the real node instead
publishes identity at lock; that mode is available here as
``world_axes=False`` and is not what the fake-vision flight uses.

``header.stamp`` stays the ground-truth capture time. Only the publish is
delayed. The bridge has to turn that stamp into PX4 time.

This module imports rclpy only inside :func:`main`, so the pose math can be
tested without ROS.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from px4_control.frames import (
    odom_pose_from_world,
    quat_xyzw_to_rot,
    yaw_ned_from_enu_quat,
)
from px4_control.vision_bridge import LOST_VARIANCE

ODOM_FRAME = "odom"
BODY_FRAME = "base_link"


def _yaw_enu(quat_xyzw: np.ndarray) -> float:
    """ENU heading of a ROS quaternion. 0 faces East, positive toward North."""
    # yaw_ned_from_enu_quat is pi/2 - psi. Invert that.
    return math.pi / 2.0 - yaw_ned_from_enu_quat(quat_xyzw)


def _wrap(angle: float) -> float:
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if wrapped <= -math.pi:
        return math.pi
    return wrapped


def _quat_mul_yaw(quat_xyzw: np.ndarray, yaw: float) -> np.ndarray:
    """Yaw an ENU quaternion by ``yaw`` radians about up."""
    half = 0.5 * float(yaw)
    # Left multiply: yaw about ENU up, not about the body axis.
    x2, y2, z2, w2 = 0.0, 0.0, math.sin(half), math.cos(half)
    x1, y1, z1, w1 = np.asarray(quat_xyzw, dtype=float).reshape(4)
    return np.array([
        w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1,
        w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1,
        w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1,
        w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1,
    ], dtype=float)


@dataclass(frozen=True)
class GroundTruth:
    """One ``/ground_truth/odom`` sample. Pose is world ENU, twist is body FLU."""

    stamp_sec: float
    position_enu: np.ndarray
    quat_xyzw: np.ndarray
    linear_flu: np.ndarray
    angular_flu: np.ndarray


@dataclass(frozen=True)
class RtabOdom:
    """What gets published on ``/rtabmap/odom`` and as ``odom`` -> child TF."""

    capture_sec: float
    publish_sec: float
    position_enu: np.ndarray
    quat_xyzw: np.ndarray
    linear_flu: np.ndarray
    angular_flu: np.ndarray
    pose_covariance: np.ndarray
    twist_covariance: np.ndarray
    frame_id: str
    child_frame_id: str
    lost: bool

    @property
    def delay_sec(self) -> float:
        return float(self.publish_sec) - float(self.capture_sec)


def _covariance(variance: float, lost: bool) -> np.ndarray:
    matrix = np.zeros(36, dtype=float)
    value = LOST_VARIANCE if lost else float(variance)
    for index in (0, 7, 14, 21, 28, 35):
        matrix[index] = value
    return matrix


class FakeRtabmap:
    """Turn ground truth into a delayed, noisy ``/rtabmap/odom`` stream.

    Rate is 10-15 Hz. Delay is 30-60 ms of the same clock as the stamps
    (sim time when the node uses ``/clock``). Faults: ``dropout`` stops
    the feed once the world yaw has moved ``dropout_yaw_deg`` from the
    first sample, ``late`` ages the header beyond the publish delay,
    ``jump`` adds a position step, and ``lost`` publishes RTAB-Map's
    null pose (zero translation, zero quaternion, covariance 9999).
    """

    def __init__(
        self,
        *,
        rate_hz: float = 12.5,
        delay_s: float = 0.045,
        noise_m: float = 0.0,
        noise_yaw_rad: float = 0.0,
        drift_mps: float = 0.0,
        fault: str = "none",
        world_axes: bool = True,
        dropout_yaw_deg: float = 70.0,
        late_extra_s: float = 0.08,
        jump_m: float = 2.0,
        seed: int = 1,
    ) -> None:
        if not 10.0 <= float(rate_hz) <= 15.0:
            raise ValueError(f"rate_hz must be 10-15, got {rate_hz}")
        if not 0.030 <= float(delay_s) <= 0.060:
            raise ValueError(f"delay_s must be 30-60 ms, got {delay_s}")
        self.rate_hz = float(rate_hz)
        self.delay_s = float(delay_s)
        self.noise_m = float(noise_m)
        self.noise_yaw_rad = float(noise_yaw_rad)
        self.drift_mps = float(drift_mps)
        self.fault = str(fault or "none")
        self.world_axes = bool(world_axes)
        self.dropout_yaw_deg = float(dropout_yaw_deg)
        self.late_extra_s = float(late_extra_s)
        self.jump_m = float(jump_m)
        self._rng = np.random.default_rng(seed)
        self._spawn_position: np.ndarray | None = None
        self._spawn_quat: np.ndarray | None = None
        self._spawn_yaw: float | None = None
        self._last_accept: float | None = None
        self._last_body: tuple[np.ndarray, np.ndarray, float] | None = None
        self._drift = np.zeros(3, dtype=float)
        self._cut = False
        self._jumped = False
        self._queue: list[tuple[float, GroundTruth, np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = []

    def push(self, sample: GroundTruth) -> None:
        """Accept one ground-truth pose. Publish happens in :meth:`poll`."""
        if self._cut:
            return
        if self._spawn_position is None:
            self._spawn_position = np.asarray(sample.position_enu, dtype=float).reshape(3).copy()
            self._spawn_quat = np.asarray(sample.quat_xyzw, dtype=float).reshape(4).copy()
            self._spawn_yaw = _yaw_enu(self._spawn_quat)
        period = 1.0 / self.rate_hz
        if self._last_accept is not None and float(sample.stamp_sec) - self._last_accept < period - 1e-6:
            return
        self._last_accept = float(sample.stamp_sec)
        if self.fault == "dropout" and self._yaw_moved(sample) >= self.dropout_yaw_deg:
            self._cut = True
            self._queue.clear()
            return
        position, quat, linear, angular = self._body_odom(sample)
        ready = float(sample.stamp_sec) + self.delay_s
        self._queue.append((ready, sample, position, quat, linear, angular))

    def poll(self, now_sec: float) -> list[RtabOdom]:
        """Messages whose publish time has arrived. Capture stamps are unchanged."""
        if self._cut and self.fault == "dropout":
            self._queue.clear()
            return []
        due: list[RtabOdom] = []
        waiting: list[tuple[float, GroundTruth, np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = []
        for item in self._queue:
            ready, sample, position, quat, linear, angular = item
            if ready > float(now_sec) + 1e-9:
                waiting.append(item)
                continue
            due.append(self._emit(sample, position, quat, linear, angular, float(now_sec)))
        self._queue = waiting
        return due

    def _yaw_moved(self, sample: GroundTruth) -> float:
        if self._spawn_yaw is None:
            return 0.0
        delta = _wrap(_yaw_enu(sample.quat_xyzw) - self._spawn_yaw)
        return abs(math.degrees(delta))

    def _body_odom(self, sample: GroundTruth) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        assert self._spawn_position is not None and self._spawn_quat is not None
        position, quat = odom_pose_from_world(
            sample.position_enu,
            sample.quat_xyzw,
            self._spawn_position,
            self._spawn_quat,
            world_axes=self.world_axes,
        )
        dt = 0.0 if self._last_body is None else float(sample.stamp_sec) - self._last_body[2]
        if dt > 1e-6 and self._last_body is not None:
            self._drift = self._drift + self._drift_step(dt)
        position = position + self._drift
        if self.noise_m > 0.0:
            position = position + self._rng.normal(0.0, self.noise_m, size=3)
        if self.noise_yaw_rad > 0.0:
            quat = _quat_mul_yaw(quat, float(self._rng.normal(0.0, self.noise_yaw_rad)))
        if self.fault == "jump" and not self._jumped and self._last_body is not None:
            position = position + np.array([self.jump_m, 0.0, 0.0], dtype=float)
            self._jumped = True
        linear, angular = self._twist(position, quat, float(sample.stamp_sec), sample)
        self._last_body = (position.copy(), quat.copy(), float(sample.stamp_sec))
        return position, quat, linear, angular

    def _drift_step(self, dt: float) -> np.ndarray:
        if self.drift_mps == 0.0:
            return np.zeros(3, dtype=float)
        # Constant slow bias, not a new random walk each sample, so a test
        # can predict it. Direction is +East in the odom frame. z is never
        # part of the drift, including when drift_mps is non-zero.
        return np.array([self.drift_mps * dt, 0.0, 0.0], dtype=float)

    def _twist(self, position, quat, stamp: float, sample: GroundTruth):
        if self._last_body is None:
            return np.zeros(3), np.asarray(sample.angular_flu, dtype=float).reshape(3)
        previous, previous_quat, previous_stamp = self._last_body
        dt = stamp - previous_stamp
        if dt <= 1e-6:
            return np.zeros(3), np.zeros(3)
        velocity_parent = (position - previous) / dt
        rotation = quat_xyzw_to_rot(quat)
        linear = rotation.T @ velocity_parent
        yaw_rate = _wrap(_yaw_enu(quat) - _yaw_enu(previous_quat)) / dt
        angular = np.array([0.0, 0.0, yaw_rate], dtype=float)
        return linear, angular

    def _emit(self, sample: GroundTruth, position, quat, linear, angular, now_sec: float) -> RtabOdom:
        lost = self.fault == "lost"
        capture = float(sample.stamp_sec)
        if self.fault == "late":
            capture = float(sample.stamp_sec) - self.late_extra_s
        if lost:
            position = np.zeros(3, dtype=float)
            quat = np.zeros(4, dtype=float)
            linear = np.zeros(3, dtype=float)
            angular = np.zeros(3, dtype=float)
        return RtabOdom(
            capture_sec=capture,
            publish_sec=float(now_sec),
            position_enu=np.asarray(position, dtype=float),
            quat_xyzw=np.asarray(quat, dtype=float),
            linear_flu=np.asarray(linear, dtype=float),
            angular_flu=np.asarray(angular, dtype=float),
            pose_covariance=_covariance(0.01, lost),
            twist_covariance=_covariance(0.05, lost),
            frame_id=ODOM_FRAME,
            child_frame_id=BODY_FRAME,
            lost=lost,
        )


def _env_float(name: str, default: float) -> float:
    import os

    raw = os.environ.get(name)
    if raw is None or str(raw).strip() == "":
        return default
    return float(raw)


def main() -> None:
    """ROS node. Ground truth in, ``/rtabmap/odom`` and ``odom`` -> ``base_link`` out."""
    import os

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from tf2_ros import TransformBroadcaster

    # RTAB-Map is started with qos:=2 (sensor data, best effort). The flight
    # subscriber is best effort. A reliable publisher does not connect to it.
    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    delay_s = _env_float("FAKE_VISION_DELAY_MS", 45.0) / 1000.0
    source = FakeRtabmap(
        rate_hz=_env_float("FAKE_VISION_RATE_HZ", 12.5),
        delay_s=delay_s,
        noise_m=_env_float("FAKE_VISION_NOISE_M", 0.0),
        noise_yaw_rad=math.radians(_env_float("FAKE_VISION_NOISE_YAW_DEG", 0.0)),
        drift_mps=_env_float("FAKE_VISION_DRIFT_MPS", 0.0),
        fault=os.environ.get("FAKE_VISION_FAULT", "none") or "none",
        world_axes=os.environ.get("FAKE_VISION_WORLD_AXES", "1") not in {"0", "false", "no"},
        dropout_yaw_deg=_env_float("FAKE_VISION_DROPOUT_YAW_DEG", 70.0),
    )

    class _Node(Node):
        def __init__(self) -> None:
            use_sim = os.environ.get("USE_SIM_TIME", "true").lower() in {"1", "true", "yes"}
            super().__init__(
                "fake_rtabmap",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, use_sim)],
            )
            self.get_logger().warning(
                "TEST SUBSTITUTE: /ground_truth/odom -> /rtabmap/odom "
                f"(delay {source.delay_s * 1000:.0f} ms, fault {source.fault}). Not RTAB-Map."
            )
            self._pub = self.create_publisher(Odometry, "/rtabmap/odom", sensor_qos)
            self._tf = TransformBroadcaster(self)
            self.create_subscription(
                Odometry, "/ground_truth/odom", self._on_gt, QoSProfile(depth=10),
            )
            # Finer than the 30–60 ms hold so a sample is released near the
            # delay instead of up to one coarse tick late.
            self.create_timer(0.005, self._poll)

        def _on_gt(self, msg: Odometry) -> None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            pose = msg.pose.pose
            twist = msg.twist.twist
            source.push(GroundTruth(
                stamp_sec=stamp,
                position_enu=np.array([pose.position.x, pose.position.y, pose.position.z]),
                quat_xyzw=np.array([
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
                ]),
                linear_flu=np.array([twist.linear.x, twist.linear.y, twist.linear.z]),
                angular_flu=np.array([twist.angular.x, twist.angular.y, twist.angular.z]),
            ))

        def _poll(self) -> None:
            now = self.get_clock().now().nanoseconds * 1e-9
            for item in source.poll(now):
                self._publish(item)

        def _publish(self, item: RtabOdom) -> None:
            msg = Odometry()
            sec = int(math.floor(item.capture_sec))
            nsec = int(round((item.capture_sec - sec) * 1e9))
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nsec
            msg.header.frame_id = item.frame_id
            msg.child_frame_id = item.child_frame_id
            msg.pose.pose.position.x = float(item.position_enu[0])
            msg.pose.pose.position.y = float(item.position_enu[1])
            msg.pose.pose.position.z = float(item.position_enu[2])
            msg.pose.pose.orientation.x = float(item.quat_xyzw[0])
            msg.pose.pose.orientation.y = float(item.quat_xyzw[1])
            msg.pose.pose.orientation.z = float(item.quat_xyzw[2])
            msg.pose.pose.orientation.w = float(item.quat_xyzw[3])
            msg.pose.covariance = [float(value) for value in item.pose_covariance]
            msg.twist.twist.linear.x = float(item.linear_flu[0])
            msg.twist.twist.linear.y = float(item.linear_flu[1])
            msg.twist.twist.linear.z = float(item.linear_flu[2])
            msg.twist.twist.angular.x = float(item.angular_flu[0])
            msg.twist.twist.angular.y = float(item.angular_flu[1])
            msg.twist.twist.angular.z = float(item.angular_flu[2])
            msg.twist.covariance = [float(value) for value in item.twist_covariance]
            self._pub.publish(msg)
            # Body TF only. RTAB-Map's frame_id is base_link; the URDF already
            # parents camera_link, so this node does not publish that child.
            if item.lost:
                return
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = BODY_FRAME
            transform.transform.translation.x = msg.pose.pose.position.x
            transform.transform.translation.y = msg.pose.pose.position.y
            transform.transform.translation.z = msg.pose.pose.position.z
            transform.transform.rotation = msg.pose.pose.orientation
            self._tf.sendTransform(transform)

    rclpy.init()
    node = _Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
