"""Publish PX4's IMU on ROS ``/imu`` for RTAB-Map.

Subscriptions, all exported by the default PX4 v1.17 ``dds_topics.yaml``
with no version suffix (``VehicleAttitude`` has ``MESSAGE_VERSION = 0``):

* ``/fmu/out/sensor_combined`` — gyro and specific force, FRD
* ``/fmu/out/vehicle_attitude`` — quaternion, FRD body to NED world
* ``/fmu/out/timesync_status`` — companion-minus-PX4 offset

A ``_vN`` topic is used when one is advertised. Nothing in the PX4 tree is
edited. The publisher is ``sensor_msgs/Imu`` on ``/imu`` with ``frame_id``
``base_link``, which is where the flight IMU sits. PX4's data writer is
best-effort and transient-local, so the subscriptions use that QoS.
"""

from __future__ import annotations

import os

from sim_monitor.checks import match_versioned_topic
from sim_monitor.imu_frames import flu_enu_quaternion_xyzw, frd_to_flu
from sim_monitor.imu_source import (
    PX4_FRAME,
    diagonal_covariance,
    ros_stamp_ns,
)

SENSOR_COMBINED = "sensor_combined"
VEHICLE_ATTITUDE = "vehicle_attitude"
TIMESYNC_STATUS = "timesync_status"
TOPIC_SUFFIX = {
    SENSOR_COMBINED: "/fmu/out/sensor_combined",
    VEHICLE_ATTITUDE: "/fmu/out/vehicle_attitude",
    TIMESYNC_STATUS: "/fmu/out/timesync_status",
}


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


def main() -> None:
    import rclpy
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
            self._orientation_cov = diagonal_covariance(self.get_parameter("orientation_stddev").value)
            self._gyro_cov = diagonal_covariance(self.get_parameter("angular_velocity_stddev").value)
            self._accel_cov = diagonal_covariance(self.get_parameter("linear_acceleration_stddev").value)
            # PX4 publishes best-effort + transient-local. A volatile subscriber
            # can still connect, but transient-local matches the writer.
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._pub_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.publisher = self.create_publisher(Imu, "/imu", self._pub_qos)
            self._qos = qos
            self._subscribed: set[str] = set()
            self._offset_us = None
            self._gyro = (0.0, 0.0, 0.0)
            self._accel = (0.0, 0.0, 0.0)
            self._orientation = None
            self._have_sample = False
            self.create_timer(1.0, self._discover)
            self._discover()
            self.get_logger().info("relaying PX4 IMU to /imu (frame_id base_link)")

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
                TIMESYNC_STATUS: self._on_timesync,
            }[kind]
            self.create_subscription(message_type, topic, callback, self._qos)
            self._subscribed.add(topic)
            self.get_logger().info(f"subscribed to {topic}")

        def _on_timesync(self, msg) -> None:
            offset = getattr(msg, "estimated_offset", None)
            if offset is not None:
                self._offset_us = int(offset)

        def _on_attitude(self, msg) -> None:
            q = getattr(msg, "q", None)
            if q is None or len(q) != 4:
                return
            self._orientation = flu_enu_quaternion_xyzw(q)

        def _on_combined(self, msg) -> None:
            gyro = getattr(msg, "gyro_rad", None)
            accel = getattr(msg, "accelerometer_m_s2", None)
            if gyro is None or accel is None:
                return
            self._gyro = frd_to_flu(gyro)
            self._accel = frd_to_flu(accel)
            self._have_sample = True
            timestamp = getattr(msg, "timestamp", 0)
            receive_ns = self.get_clock().now().nanoseconds
            # See ros_stamp_ns: timesync when it lands on this clock, else
            # the receive time (sim time under use_sim_time).
            stamp_ns = ros_stamp_ns(int(timestamp), self._offset_us, receive_ns)
            self._publish(stamp_ns)

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
