# Single-purpose bridge: PX4 uXRCE-DDS IMU data -> sensor_msgs/Imu on /imu.
#
# Subscribes only to /fmu/out/sensor_combined (gyro + accelerometer) and
# /fmu/out/vehicle_attitude (orientation), converts PX4's FRD body / NED world
# convention to ROS's FLU body / ENU world convention, and publishes /imu.
# No odometry subscription, no TF broadcast, no state shared with any other
# node (see docs/plans/px4_indoor_flight_closed-loop.md, Step 1).
import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from px4_msgs.msg import SensorCombined, VehicleAttitude

# Static frame conventions (same as used by px4_ros_com frame_transforms):
# NED_ENU_Q rotates the world frame (NED -> ENU); AIRCRAFT_BASELINK_Q rotates
# the body frame (FRD -> FLU). Both are fixed 180 deg rotations, so applying
# them per-message needs no "zero reference" and cannot race with anything.
NED_ENU_Q = R.from_quat([0.70710678, 0.70710678, 0.0, 0.0])
AIRCRAFT_BASELINK_Q = R.from_quat([1.0, 0.0, 0.0, 0.0])


class PX4ImuBridge(Node):

    def __init__(self):
        # Hard-require sim time so /imu stamps match Gazebo camera stamps.
        # Without this, RTAB-Map cannot interpolate IMU (wall-clock vs sim-time).
        super().__init__(
            'px4_imu_bridge',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.publisher_imu = self.create_publisher(Imu, '/imu', qos_profile_sensor_data)

        self.subscriber_sensor_combined = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.sensor_combined_callback,
            qos_profile_sensor_data)

        self.subscriber_vehicle_attitude = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile_sensor_data)

        self.latest_orientation_flu_enu = None
        self.get_logger().info('px4_imu_bridge started: /fmu/out/{sensor_combined,vehicle_attitude} -> /imu')

    def vehicle_attitude_callback(self, msg):
        q_ned_frd = np.array([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])
        q_flu_enu = NED_ENU_Q * R.from_quat(q_ned_frd) * AIRCRAFT_BASELINK_Q
        self.latest_orientation_flu_enu = q_flu_enu.as_quat()

    def sensor_combined_callback(self, msg):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'base_link'

        # Body-frame vectors only need the FRD -> FLU convention (negate Y/Z),
        # same fixed convention microxrce_offboard.py applies via T_BvBp.
        imu_msg.angular_velocity.x = float(msg.gyro_rad[0])
        imu_msg.angular_velocity.y = float(-msg.gyro_rad[1])
        imu_msg.angular_velocity.z = float(-msg.gyro_rad[2])

        imu_msg.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        imu_msg.linear_acceleration.y = float(-msg.accelerometer_m_s2[1])
        imu_msg.linear_acceleration.z = float(-msg.accelerometer_m_s2[2])

        if self.latest_orientation_flu_enu is not None:
            q = self.latest_orientation_flu_enu
            imu_msg.orientation.x = float(q[0])
            imu_msg.orientation.y = float(q[1])
            imu_msg.orientation.z = float(q[2])
            imu_msg.orientation.w = float(q[3])
        else:
            # No attitude sample yet: mark orientation as unknown (REP-145).
            imu_msg.orientation_covariance[0] = -1.0

        self.publisher_imu.publish(imu_msg)


if __name__ == "__main__":

    rclpy.init(args=None)

    imu_bridge = PX4ImuBridge()

    rclpy.spin(imu_bridge)

    imu_bridge.destroy_node()
    rclpy.shutdown()
