# Amiraqaie: https://github.com/Amiraqaie
import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist

from geometry_msgs.msg import TransformStamped, Twist, PoseWithCovariance
from tf2_ros import TransformBroadcaster
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
import time
import argparse


def str2bool(v):
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--OffboardControllEnable", 
                        type=str2bool , 
                        default=str2bool(True), 
                        help='Enabling offboard controll or just pass odometry')
    
    parser.add_argument("--TakeoffHeight", 
                        default=1.5, 
                        type=float, 
                        help='TakeOff flight height')
    
    args = parser.parse_args()
    return args

camera_info = "/camera/camera_info"

class OffboardControll(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        args = parse_args()

        qos_profile1 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile2 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create Subscribtions nodes
        self.subscriber_vehicle_status = self.create_subscription(VehicleStatus,
                                                                  '/fmu/out/vehicle_status',
                                                                  self.vehicle_status_callback,
                                                                  qos_profile1)
        
        self.subscriber_vio_odometry = self.create_subscription(Odometry,
                                                                '/odom',
                                                                self.slam_odom_callback,
                                                                qos_profile1)
        
        # self.subscriber_localization_odom = self.create_subscription(PoseWithCovariance,
        #                                                         '/localization_pose',
        #                                                         self.slam_localization_odom_callback,
        #                                                         qos_profile1)
        
        self.subscriber_cmd_vel = self.create_subscription(Twist,
                                                           '/cmd_vel',
                                                           self.cmd_vel_callback,
                                                           10)
        
        self.subscriber_camera_timestamp = self.create_subscription(CameraInfo,
                                                                    camera_info,
                                                                    self.timestamp_update,
                                                                    qos_profile1)
        
        self.subscriber_vehicle_odometry = self.create_subscription(VehicleOdometry,
                                                                    '/fmu/out/vehicle_odometry',
                                                                    self.vehicle_odometry_callback,
                                                                    qos_profile1)

        # Create publications nodes
        self.publisher_offboard_control_mode = self.create_publisher(OffboardControlMode,
                                                                      '/fmu/in/offboard_control_mode',
                                                                        qos_profile2)
        
        self.publisher_trajectory_setpoint = self.create_publisher(TrajectorySetpoint,
                                                                    '/fmu/in/trajectory_setpoint', 
                                                                    qos_profile2)
        
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand,
                                                                '/fmu/in/vehicle_command',
                                                                  qos_profile2)
        
        self.publiser_slam_odom = self.create_publisher(VehicleOdometry,
                                                        '/fmu/in/vehicle_visual_odometry',
                                                        qos_profile2)
        
        self.recovery_node_publisher = self.create_publisher(Twist,
                                                             '/cmd_vel', 
                                                             10)

        # define timer for publishing cmd_vel
        self.timer_peroid = 0.05
        self.timer = self.create_timer(self.timer_peroid, self.timer_callback)

        # Define initial parameters
        self._logger.info("initiated the offboard controll node")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.vio_odometry = Odometry()
        self.vehilcle_odometry_zero = None
        self.T_BvBp = np.array([[1, 0, 0], 
                                [0, -1, 0], 
                                [0, 0, -1]])
        self.T_BpBv = np.transpose(self.T_BvBp)
        self.T_RpBp_zero = R.from_quat([0, 0, 0, 1]).as_matrix()
        self.T_RpRv = None
        self.timestamp = None
        self.arm_state = 1
        self.take_off_ground = False
        self.takeOffHeight = -args.TakeoffHeight
        self.not_published_before = True
        self.takeOff_complete = False
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.OffboardControllEnable = args.OffboardControllEnable
        self.IS_ODOM_LOST = False
        self.cmd_vel_timestamp = Clock().now()
        self.cmd_vel_msg = Twist()

    def slam_localization_odom_callback(self, msg):
        self.pose_with_covariance = msg

    def vehicle_status_callback(self, msg):

        # update navigation status
        self.nav_state = msg.nav_state

        # update arm state
        self.arm_state = msg.arming_state

        # arm or takeoff the drone and update the take_off condition
        # based on take_off_complete variable
        if self.arm_state == 1:
            self.arm()
        if (self.arm_state == 2) :
            if not self.takeOff_complete:
                self.take_off_ground = True
                pass

    def slam_odom_callback(self, msg):

        # update the vio properties
        if msg.pose.pose.orientation.x != 0 \
            or msg.pose.pose.orientation.y != 0 \
            or msg.pose.pose.orientation.z != 0 \
            or msg.pose.pose.orientation.w != 0 :
            self.vio_odometry = msg
            self.IS_ODOM_LOST = False
        else:
            # activating recovery mode if ODOM is lost 
            self.IS_ODOM_LOST = True

        # saving vio last timestamp
        self.vio_time_stamp = int(Clock().now().nanoseconds / 1000)

        # print the comparision of slam and vehicle odomtery
        if self.take_off_ground:
            print(time.time(), '   vehicle odometry is : ', self.vehicle_odometry.position[2] , 'VIO odometry is : ', -self.vio_odometry.pose.pose.position.z
            , 'error in fusion is : ', abs(self.vehicle_odometry.position[2] + self.vio_odometry.pose.pose.position.z))

        # calculating the transfor matrixes
        self.q = np.array([self.vio_odometry.pose.pose.orientation.x, 
                           self.vio_odometry.pose.pose.orientation.y, 
                           self.vio_odometry.pose.pose.orientation.z, 
                           self.vio_odometry.pose.pose.orientation.w], 
                           dtype=np.float32)
        
        self.T_RvBv = R.from_quat(self.q).as_matrix()

        x = np.dot(self.T_RpBp_zero, np.transpose(self.T_BvBp))

        y = np.dot(x, self.T_RvBv)

        self.T_RpBp = np.dot(y, self.T_BvBp)

        # publishing the odom to autopilot
        self.callback_loop()

    def cmd_vel_callback(self, msg):

        # getting and publishing the recived cmd_vel to navigate the quadcopter
        self.cmd_vel_timestamp = Clock().now()
        if self.IS_ODOM_LOST:
            self.cmd_vel_msg = Twist()
            self.cmd_vel_msg.angular.z = 0.2
        else:
            self.cmd_vel_msg = msg

    def timestamp_update(self, msg):

        # updating he timestamp 
        self.timestamp = msg.header.stamp

    def vehicle_odometry_callback(self, msg):

        # first get the T_RpBp_zero
        if self.vehilcle_odometry_zero is None:
            self.vehilcle_odometry_zero = msg
            self.fmu_q_zero =np.array([self.vehilcle_odometry_zero.q[1], self.vehilcle_odometry_zero.q[2], self.vehilcle_odometry_zero.q[3], self.vehilcle_odometry_zero.q[0]])
            self.T_RpBp_zero = R.from_quat(self.fmu_q_zero).as_matrix()
            self.T_RpRv = np.dot(self.T_RpBp_zero, np.transpose(self.T_BvBp))
            print(time.time(),"  T_RpBp_zero is going to set equal to : \n", self.T_RpBp_zero)
        else:
            pass

        # then publish update vehicle odometry variable
        self.vehicle_odometry = msg
        self.fmu_q =np.array([self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3], self.vehicle_odometry.q[0]])
        self.T_RpBp_fusion = R.from_quat(self.fmu_q).as_matrix()

        # update current_goal if we are not in OffBoard mode
        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD \
            and self.vehicle_odometry is not None \
            and not self.takeOff_complete: 
            self.current_goal = self.vehicle_odometry

        # publish take of setpoint_trajectory msg
        if self.take_off_ground and self.OffboardControllEnable and (self.current_goal is not None): 
            if self.not_published_before:
                print(time.time(),"  publishing offborad command before takeoff")
                self.publish_setpoints_before_chage_to_offborad()
                self.change_to_offboard()
                self.not_published_before = False
            self.takeOff(self.vehicle_odometry.position[2])

    def timer_callback(self):

        if self.OffboardControllEnable and self.takeOff_complete:
            # transforming cmd_vel to NED frame
            self.linear_Bp = np.array([self.cmd_vel_msg.linear.x, 
                                       -self.cmd_vel_msg.linear.y, 
                                       -self.cmd_vel_msg.linear.z], 
                                       dtype=np.float32)
            self.angular_Bp = np.array([self.cmd_vel_msg.angular.x, 
                                        -self.cmd_vel_msg.angular.y, 
                                        -self.cmd_vel_msg.angular.z], 
                                        dtype=np.float32)
            self.linear_Rp = np.dot(self.T_RpBp_fusion, 
                                    np.transpose(self.linear_Bp))
            self.angular_Rp = np.dot(self.T_RpBp_fusion, 
                                     np.transpose(self.angular_Bp))

            # update cuurent_goal if drone was not in offboard controll mode
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.current_goal = self.vehicle_odometry

            # publish the transformed cmd_vel to autopilot_rtps if recived cmd_vel recently
            if (abs((self.cmd_vel_timestamp.seconds_nanoseconds()[0]+ \
                self.cmd_vel_timestamp.seconds_nanoseconds()[1]/10e8) - \
                (Clock().now().seconds_nanoseconds()[0]+ \
                Clock().now().seconds_nanoseconds()[1]/10e8)) < 0.5):
                self.current_goal.position[0] = self.vehicle_odometry.position[0]
                self.current_goal.position[1] = self.vehicle_odometry.position[1]
                offboard_msg = OffboardControlMode()
                time_stamp = int(Clock().now().nanoseconds / 1000)
                offboard_msg.timestamp = time_stamp
                offboard_msg.position=True
                offboard_msg.velocity=True
                self.publisher_offboard_control_mode.publish(offboard_msg)
                # if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                if True:
                    trajectory_msg = TrajectorySetpoint()
                    trajectory_msg.timestamp = time_stamp
                    trajectory_msg.position[0] = float('nan')
                    trajectory_msg.position[1] = float('nan')
                    trajectory_msg.position[2] = self.current_goal.position[2]
                    # trajectory_msg.position[2] = self.takeOffHeight
                    trajectory_msg.velocity[0] = self.linear_Rp[0]
                    trajectory_msg.velocity[1] = self.linear_Rp[1]
                    trajectory_msg.velocity[2] = float('nan')
                    trajectory_msg.yaw = float('nan')
                    trajectory_msg.yawspeed = self.angular_Rp[2]
                    print(time.time(),"  publishing the recived cmd_vel into px4")
                    self.publisher_trajectory_setpoint.publish(trajectory_msg)

            # publish the position controll command to autopilot_rtps if not recived cmd_vel recently
            elif (abs((self.cmd_vel_timestamp.seconds_nanoseconds()[0]+ \
                  self.cmd_vel_timestamp.seconds_nanoseconds()[1]/10e8) - \
                  (Clock().now().seconds_nanoseconds()[0]+ \
                  Clock().now().seconds_nanoseconds()[1])/10e8) >= 0.5):
                offboard_msg = OffboardControlMode()
                time_stamp = int(Clock().now().nanoseconds / 1000)
                offboard_msg.timestamp = time_stamp
                offboard_msg.position=True
                offboard_msg.velocity=False
                self.publisher_offboard_control_mode.publish(offboard_msg)
                # if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                if True:
                    trajectory_msg = TrajectorySetpoint()
                    trajectory_msg.timestamp = time_stamp
                    trajectory_msg.position[0] = self.current_goal.position[0]
                    trajectory_msg.position[1] = self.current_goal.position[1]
                    trajectory_msg.position[2] = self.current_goal.position[2]
                    # trajectory_msg.position[2] = self.takeOffHeight
                    trajectory_msg.velocity[0] = float('nan')
                    trajectory_msg.velocity[1] = float('nan')
                    trajectory_msg.velocity[2] = float('nan')
                    trajectory_msg.yaw = float('nan')
                    trajectory_msg.yawspeed = float('nan')
                    print(time.time(),"  publishing pose hold mode message into px4")
                    self.publisher_trajectory_setpoint.publish(trajectory_msg)
            else:
                print("What the Fuck!!!")

    def callback_loop(self):

        # calculating odometry in NED frame and publish it to autopilot
        if (self.T_RpRv is not None) and \
            (not np.array_equal(self.T_RpBp_zero, np.ones_like(self.T_RpBp_zero))) and \
            not self.IS_ODOM_LOST:
            self.rtps_odometry_in = VehicleOdometry()
            self.rtps_odometry_in.timestamp = self.vio_time_stamp
            self.rtps_odometry_in.timestamp_sample = self.vio_time_stamp
            self.rtps_odometry_in.pose_frame = 1
            self.r_rv = np.array([self.vio_odometry.pose.pose.position.x, 
                                  self.vio_odometry.pose.pose.position.y, 
                                  self.vio_odometry.pose.pose.position.z])
            self.r_rp = np.dot(self.T_RpRv, np.transpose(self.r_rv))
            self.rtps_odometry_in.position[0] = self.r_rp[0]
            self.rtps_odometry_in.position[1] = self.r_rp[1]
            self.rtps_odometry_in.position[2] = self.r_rp[2]
            q = R.from_matrix(self.T_RpBp).as_quat()
            self.euler = R.from_quat(q).as_euler('xyz', degrees=True)
            self.rtps_odometry_in.q = np.array([q[3], q[0], q[1], q[2]], dtype=np.float32)
            self.rtps_odometry_in.velocity_frame = 3
            self.v_bv = np.array([self.vio_odometry.twist.twist.linear.x, 
                                  self.vio_odometry.twist.twist.linear.y, 
                                  self.vio_odometry.twist.twist.linear.z])
            self.v_bp = np.dot(self.T_BpBv, np.transpose(self.v_bv))
            self.rtps_odometry_in.velocity[0] = self.v_bp[0]
            self.rtps_odometry_in.velocity[1] = self.v_bp[1]
            self.rtps_odometry_in.velocity[2] = self.v_bp[2]
            self.angular_bv = np.array([self.vio_odometry.twist.twist.angular.x, 
                                        self.vio_odometry.twist.twist.angular.y, 
                                        self.vio_odometry.twist.twist.angular.z])
            self.angular_bp = np.dot(self.T_BpBv, np.transpose(self.angular_bv))
            self.rtps_odometry_in.angular_velocity[0] = self.angular_bp[0]
            self.rtps_odometry_in.angular_velocity[1] = self.angular_bp[1]
            self.rtps_odometry_in.angular_velocity[2] = self.angular_bp[2]
            self.vio_pose_variance = np.array([
                self.vio_odometry.pose.covariance[0],
                self.vio_odometry.pose.covariance[7],
                self.vio_odometry.pose.covariance[14]
            ])
            self.vio_orientation_variance = np.array([
                self.vio_odometry.pose.covariance[21],
                self.vio_odometry.pose.covariance[28],
                self.vio_odometry.pose.covariance[35]
            ])
            self.vio_velocity_variance = np.array([
                self.vio_odometry.twist.covariance[0],
                self.vio_odometry.twist.covariance[7],
                self.vio_odometry.twist.covariance[14]
            ])
            # self.vio_angular_variance = np.array([
            #     self.vio_odometry.twist.covariance[21],
            #     self.vio_odometry.twist.covariance[28],
            #     self.vio_odometry.twist.covariance[35]
            # ])

            self.px4_pose_variance = np.dot(self.T_BpBv, np.transpose(self.vio_pose_variance))
            self.px4_orientation_variance = np.dot(self.T_BpBv, np.transpose(self.vio_orientation_variance))
            self.px4_velocity_variance = np.dot(self.T_BpBv, np.transpose(self.vio_velocity_variance))
            # self.px4_angular_variance = np.dot(self.T_BpBv, np.transpose(self.vio_angular_variance))

            self.rtps_odometry_in.position_variance[0] = self.px4_pose_variance[0]
            self.rtps_odometry_in.position_variance[1] = self.px4_pose_variance[1]
            self.rtps_odometry_in.position_variance[2] = self.px4_pose_variance[2]
            self.rtps_odometry_in.orientation_variance[0] = self.px4_orientation_variance[0]
            self.rtps_odometry_in.orientation_variance[1] = self.px4_orientation_variance[1]
            self.rtps_odometry_in.orientation_variance[2] = self.px4_orientation_variance[2]
            self.rtps_odometry_in.velocity_variance[0] = self.px4_velocity_variance[0]
            self.rtps_odometry_in.velocity_variance[1] = self.px4_velocity_variance[1]
            self.rtps_odometry_in.velocity_variance[2] = self.px4_velocity_variance[2]

            # self.rtps_odometry_in.position_variance[0] = self.vio_odometry.pose.covariance[0]
            # self.rtps_odometry_in.position_variance[1] = self.vio_odometry.pose.covariance[0]
            # self.rtps_odometry_in.position_variance[2] = self.vio_odometry.pose.covariance[0]
            # self.rtps_odometry_in.orientation_variance[0] = self.vio_odometry.pose.covariance[-1]
            # self.rtps_odometry_in.orientation_variance[1] = self.vio_odometry.pose.covariance[-1]
            # self.rtps_odometry_in.orientation_variance[2] = self.vio_odometry.pose.covariance[-1]
            # self.rtps_odometry_in.velocity_variance[0] = self.vio_odometry.pose.covariance[0]
            # self.rtps_odometry_in.velocity_variance[1] = self.vio_odometry.pose.covariance[0]
            # self.rtps_odometry_in.velocity_variance[2] = self.vio_odometry.pose.covariance[0]
            self.rtps_odometry_in.reset_counter = 4
            self.publiser_slam_odom.publish(self.rtps_odometry_in)
        elif self.IS_ODOM_LOST:
            print(time.time(),"  ODOMETRY is lost !!!")

    def arm(self):

        # publishing arm command based on uorb instruction
        if self.OffboardControllEnable:
            vehicle_command = VehicleCommand()
            vehicle_command.param1 = 1.0
            vehicle_command.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
            vehicle_command.target_system = 1
            vehicle_command.target_component = 1
            vehicle_command.source_system = 1
            vehicle_command.source_component = 1
            vehicle_command.from_external = True
            vehicle_command.timestamp = int(Clock().now().nanoseconds / 1000)
            self.publisher_vehicle_command.publish(vehicle_command)
            print(time.time(),"  publishing arming command")

    def takeOff(self, z):

        # publishing takeoff commnad to autopilot based on vehicle_odometry pose massage
        if self.OffboardControllEnable:
            print(time.time(),"  takeing off the ground.")
            if (self.takeOffHeight - z) < -0.1 and self.take_off_ground:
                offboard_msg = OffboardControlMode()
                time_stamp = int(Clock().now().nanoseconds / 1000)
                offboard_msg.timestamp = time_stamp
                offboard_msg.position=True
                offboard_msg.velocity=False
                self.publisher_offboard_control_mode.publish(offboard_msg)
                if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    trajectory_msg = TrajectorySetpoint()
                    trajectory_msg.timestamp = time_stamp
                    trajectory_msg.position[0] = self.current_goal.position[0]
                    trajectory_msg.position[1] = self.current_goal.position[1]
                    trajectory_msg.position[2] = self.takeOffHeight 
                    trajectory_msg.velocity[0] = float('nan')
                    trajectory_msg.velocity[1] = float('nan')
                    trajectory_msg.velocity[2] = float('nan')
                    trajectory_msg.yaw = float('nan')
                    trajectory_msg.yawspeed = 0.0
                    self.publisher_trajectory_setpoint.publish(trajectory_msg)
            elif (self.takeOffHeight - z) > 0.1 and self.take_off_ground:
                offboard_msg = OffboardControlMode()
                time_stamp = int(Clock().now().nanoseconds / 1000)
                offboard_msg.timestamp = time_stamp
                offboard_msg.position=True
                offboard_msg.velocity=False
                self.publisher_offboard_control_mode.publish(offboard_msg)
                if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    trajectory_msg = TrajectorySetpoint()
                    trajectory_msg.timestamp = time_stamp
                    trajectory_msg.position[0] = self.current_goal.position[0]
                    trajectory_msg.position[1] = self.current_goal.position[1]
                    trajectory_msg.position[2] = self.takeOffHeight
                    trajectory_msg.velocity[0] = float('nan')
                    trajectory_msg.velocity[1] = float('nan')
                    trajectory_msg.velocity[2] = float('nan')
                    trajectory_msg.yaw = float('nan')
                    trajectory_msg.yawspeed = 0.0
                    self.publisher_trajectory_setpoint.publish(trajectory_msg)
            else:
                self.take_off_ground = False
                self.takeOff_complete = True
                self.current_goal = self.vehicle_odometry
                print(time.time(),"  takeOff completed successfully!!")

    def change_to_offboard(self):

        # changing flight mode to offboard controll mode
        if self.OffboardControllEnable:
            vehicle_command = VehicleCommand()
            vehicle_command.param1 = 1.0
            vehicle_command.param2 = 6.0
            vehicle_command.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
            vehicle_command.target_system = 1
            vehicle_command.target_component = 1
            vehicle_command.from_external = True
            vehicle_command.timestamp = int(Clock().now().nanoseconds / 1000)
            self.publisher_vehicle_command.publish(vehicle_command)

    def publish_setpoints_before_chage_to_offborad(self):

        # publishing 100 massage of setpoint before actual setpoints
        if self.OffboardControllEnable:
            for i in range(100):
                offboard_msg = OffboardControlMode()
                time_stamp = int(Clock().now().nanoseconds / 1000)
                offboard_msg.timestamp = time_stamp
                offboard_msg.position=True
                offboard_msg.velocity=True
                self.publisher_offboard_control_mode.publish(offboard_msg)
                if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    trajectory_msg = TrajectorySetpoint()
                    trajectory_msg.timestamp = time_stamp
                    trajectory_msg.position[0] = float('nan')
                    trajectory_msg.position[1] = float('nan')
                    trajectory_msg.position[2] = float('nan')
                    trajectory_msg.velocity[0] = 0.0
                    trajectory_msg.velocity[1] = 0.0
                    trajectory_msg.velocity[2] = 0.0
                    trajectory_msg.yaw = float('nan')
                    trajectory_msg.yawspeed = 0.0
                    self.publisher_trajectory_setpoint.publish(trajectory_msg)

if __name__ == "__main__":

    # initing the ros client python
    rclpy.init(args=None)

    # creating the offboard node
    time.sleep(1)
    offboard_controll = OffboardControll()

    # spining node
    rclpy.spin(offboard_controll)

    # destroying node
    offboard_controll.destroy_node()
    rclpy.shutdown()
