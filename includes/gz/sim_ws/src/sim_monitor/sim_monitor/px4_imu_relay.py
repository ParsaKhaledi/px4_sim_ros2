"""Publish PX4's IMU on ROS ``/imu`` for RTAB-Map.

Subscriptions, exported by the default PX4 v1.17 ``dds_topics.yaml`` with no
version suffix (``VehicleAttitude`` has ``MESSAGE_VERSION = 0``):

* ``/fmu/out/sensor_combined`` — gyro and specific force, FRD
* ``/fmu/out/vehicle_attitude`` — quaternion, FRD body to NED world

A ``_vN`` topic is used when one is advertised. Nothing in the PX4 tree is
edited. The publisher is ``sensor_msgs/Imu`` on ``/imu`` with ``frame_id``
``base_link``. PX4's data writer is best-effort and transient-local, so the
subscriptions use that QoS.

Stamps default to the ROS clock at receive time (``IMU_STAMP_MODE=receive``).
That clock is Gazebo sim time when ``use_sim_time`` is set. PX4's own
timestamp is microseconds since boot. RTAB-Map's ``wait_imu_to_init`` compares
IMU stamps with image stamps, so a raw boot timestamp is dropped. See
``imu_stamp_ns``.
"""

from __future__ import annotations

import os

from sim_monitor.checks import match_versioned_topic
from sim_monitor.imu_frames import frd_to_flu, gravity_quaternion_xyzw
from sim_monitor.imu_source import (
    PX4_FRAME,
    PX4_OFFSET,
    PX4_RATE_WARN_HZ,
    diagonal_covariance,
    imu_stamp_ns,
    input_rate_hz,
    measure_px4_offset_ns,
    orientation_covariance,
    px4_sample_us,
    rate_log,
    stamp_mode,
)

SENSOR_COMBINED = "sensor_combined"
VEHICLE_ATTITUDE = "vehicle_attitude"
TOPIC_SUFFIX = {
    SENSOR_COMBINED: "/fmu/out/sensor_combined",
    VEHICLE_ATTITUDE: "/fmu/out/vehicle_attitude",
}
RATE_WINDOW_NS = 2_000_000_000
RATE_LOG_PERIOD_NS = 1_000_000_000


def px4_topic(names: list[str], suffix: str) -> str:
    """Versioned topic when one is advertised, otherwise the v1.17 default."""
    found = match_versioned_topic(names, suffix)
    if found is not None:
        return found
    return "/fmu/out/" + suffix


def split_stamp(stamp_ns: int) -> tuple[int, int]:
    stamp_ns = int(stamp_ns)
    sec = stamp_ns // 1_000_000_000
    nanosec = stamp_ns - sec * 1_000_000_000
    return sec, nanosec


def trim_rate_window(stamps_ns: list[int], now_ns: int, window_ns: int = RATE_WINDOW_NS) -> None:
    stamps_ns.append(int(now_ns))
    while len(stamps_ns) > 2 and now_ns - stamps_ns[0] > window_ns:
        stamps_ns.pop(0)


def main() -> None:
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import Imu

    class Px4ImuRelay(Node):
        def __init__(self) -> None:
            use_sim = os.environ.get("USE_SIM_TIME", "true").lower() in {"1", "true", "yes"}
            super().__init__(
                "px4_imu_relay",
                parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, use_sim)],
            )
            self.declare_parameter("orientation_stddev", 0.02)
            self.declare_parameter("angular_velocity_stddev", 0.01)
            self.declare_parameter("linear_acceleration_stddev", 0.1)
            self.declare_parameter("yaw_variance", 1.0e3)
            self.declare_parameter("rate_warn_hz", PX4_RATE_WARN_HZ)
            self._rate_warn_hz = float(self.get_parameter("rate_warn_hz").value)
            self._mode = stamp_mode()
            self._gyro_cov = diagonal_covariance(self.get_parameter("angular_velocity_stddev").value)
            self._accel_cov = diagonal_covariance(self.get_parameter("linear_acceleration_stddev").value)
            self._orientation_cov = orientation_covariance(
                self.get_parameter("orientation_stddev").value,
                self.get_parameter("yaw_variance").value,
            )
            # PX4 publishes best-effort + transient-local. A volatile subscriber
            # can still connect, but transient-local matches the writer.
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            pub_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.publisher = self.create_publisher(Imu, "/imu", pub_qos)
            self._qos = qos
            self._subscribed: set[str] = set()
            self._offset_ns = None
            self._gyro = (0.0, 0.0, 0.0)
            self._accel = (0.0, 0.0, 0.0)
            self._orientation = None
            self._have_sample = False
            self._rate_stamps: list[int] = []
            self._last_rate_log_ns = None
            # Wall clock. A sim-time timer slows with RTF and stops if /clock stalls.
            self.create_timer(
                1.0, self._discover, clock=Clock(clock_type=ClockType.STEADY_TIME),
            )
            self._discover()
            self.get_logger().info(
                f"relaying PX4 IMU to /imu (frame_id base_link, stamp {self._mode})"
            )

        def _discover(self) -> None:
            try:
                names = [name for name, _types in self.get_topic_names_and_types()]
            except Exception as exc:
                self.get_logger().error(f"topic discovery failed: {exc}")
                return
            for suffix in TOPIC_SUFFIX:
                topic = px4_topic(names, suffix)
                self._subscribe(topic, suffix)

        def _subscribe(self, topic: str, kind: str) -> None:
            if topic in self._subscribed:
                return
            names_and_types = dict(self.get_topic_names_and_types())
            if topic not in names_and_types:
                return
            try:
                message_type = get_message(names_and_types[topic][0])
            except Exception as exc:
                self.get_logger().error(f"cannot load {topic}: {exc}")
                return
            callback = {
                SENSOR_COMBINED: self._on_combined,
                VEHICLE_ATTITUDE: self._on_attitude,
            }[kind]
            self.create_subscription(message_type, topic, callback, self._qos)
            self._subscribed.add(topic)
            self.get_logger().info(f"subscribed to {topic}")

        def _on_attitude(self, msg) -> None:
            q = getattr(msg, "q", None)
            if q is None or len(q) != 4:
                return
            self._orientation = gravity_quaternion_xyzw(q)

        def _on_combined(self, msg) -> None:
            gyro = getattr(msg, "gyro_rad", None)
            accel = getattr(msg, "accelerometer_m_s2", None)
            if gyro is None or accel is None:
                return
            self._gyro = frd_to_flu(gyro)
            self._accel = frd_to_flu(accel)
            self._have_sample = True
            receive_ns = self.get_clock().now().nanoseconds
            sample_us = px4_sample_us(
                int(getattr(msg, "timestamp", 0)),
                getattr(msg, "timestamp_sample", None),
            )
            if self._mode == PX4_OFFSET and self._offset_ns is None:
                self._offset_ns = measure_px4_offset_ns(sample_us, receive_ns)
            stamp_ns = imu_stamp_ns(self._mode, receive_ns, sample_us, self._offset_ns)
            trim_rate_window(self._rate_stamps, receive_ns)
            self._log_rate(receive_ns)
            self._publish(stamp_ns)

        def _log_rate(self, now_ns: int) -> None:
            if self._last_rate_log_ns is not None and now_ns - self._last_rate_log_ns < RATE_LOG_PERIOD_NS:
                return
            entry = rate_log(input_rate_hz(self._rate_stamps), self._rate_warn_hz)
            if entry is None:
                return
            level, text = entry
            self._last_rate_log_ns = now_ns
            if level == "warn":
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)

        def _publish(self, stamp_ns: int) -> None:
            if not self._have_sample:
                return
            message = Imu()
            sec, nanosec = split_stamp(stamp_ns)
            message.header.stamp.sec = sec
            message.header.stamp.nanosec = nanosec
            message.header.frame_id = PX4_FRAME
            gx, gy, gz = self._gyro
            ax, ay, az = self._accel
            message.angular_velocity.x = gx
            message.angular_velocity.y = gy
            message.angular_velocity.z = gz
            message.linear_acceleration.x = ax
            message.linear_acceleration.y = ay
            message.linear_acceleration.z = az
            message.angular_velocity_covariance = self._gyro_cov
            message.linear_acceleration_covariance = self._accel_cov
            if self._orientation is None:
                # ROS convention: covariance[0] < 0 means orientation is unset.
                message.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                message.orientation.w = 1.0
            else:
                x, y, z, w = self._orientation
                message.orientation.x = x
                message.orientation.y = y
                message.orientation.z = z
                message.orientation.w = w
                message.orientation_covariance = self._orientation_cov
            self.publisher.publish(message)

    rclpy.init()
    node = Px4ImuRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
