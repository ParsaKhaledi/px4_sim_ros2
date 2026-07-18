# Amiraqaie: https://github.com/Amiraqaie
import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist

from geometry_msgs.msg import TransformStamped, Twist, TwistStamped, PoseWithCovariance
from tf2_ros import TransformBroadcaster
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
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
                        default=1.0, 
                        type=float, 
                        help='TakeOff flight height')
    
    # Allow --ros-args (e.g. use_sim_time) to pass through unused.
    args, _ = parser.parse_known_args()
    return args


class OffboardControll(Node):

    def __init__(self):
        super().__init__(
            'minimal_publisher',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        args = parse_args()

        qos_profile1 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Must match PX4 uXRCE subscriber QoS (BEST_EFFORT + VOLATILE).
        # TRANSIENT_LOCAL publishers can fail to deliver to PX4's VOLATILE
        # readers and show up as offboard_control_signal_lost.
        qos_profile2 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create Subscribtions nodes
        self.subscriber_vehicle_status = self.create_subscription(VehicleStatus,
                                                                  '/fmu/out/vehicle_status',
                                                                  self.vehicle_status_callback,
                                                                  qos_profile1)
        
        self.subscriber_vio_odometry = self.create_subscription(Odometry,
                                                                '/rtabmap/odom',
                                                                self.slam_odom_callback,
                                                                qos_profile1)
        
        # self.subscriber_localization_odom = self.create_subscription(PoseWithCovariance,
        #                                                         '/localization_pose',
        #                                                         self.slam_localization_odom_callback,
        #                                                         qos_profile1)
        
        # Humble Nav2 publishes geometry_msgs/Twist on /cmd_vel.
        # Jazzy+ can use TwistStamped when enable_stamped_cmd_vel is true;
        # the local validation image is Humble, so subscribe to Twist here.
        self.subscriber_cmd_vel = self.create_subscription(Twist,
                                                           '/cmd_vel',
                                                           self.cmd_vel_callback,
                                                           10)
        
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
        
        # Match /cmd_vel subscription type (Twist on Humble Nav2).
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
        self.arm_state = 1
        self.take_off_ground = False
        self.takeOffHeight = -args.TakeoffHeight
        self.not_published_before = True
        self.takeOff_complete = False
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.OffboardControllEnable = args.OffboardControllEnable
        self.IS_ODOM_LOST = False
        self.cmd_vel_timestamp = self.get_clock().now()
        self.cmd_vel_msg = Twist()
        self._offboard_setpoint_counter = 0
        self.current_goal = None
        self.vehicle_odometry = None
        self._last_takeoff_feedback = None
        self._hover_xy = None  # freeze XY at arm/takeoff start for hold
        self._hold_pose = None  # [x, y, z_ned] after takeoff complete
        self._was_following = False  # true while actively following a cmd_vel trajectory

        # VIO outlier rejection: reject any RTAB-Map sample implying a jump
        # larger than these bounds since the last accepted good sample.
        self._vio_last_good_pos = None
        self._vio_last_good_quat = None
        self._vio_max_jump_m = 0.15
        self._vio_max_angle_deg = 12.0
        self._vio_outlier_count = 0

        # Safety failsafe: if vision has been lost/rejected continuously for
        # too long, PX4 is flying blind on IMU+baro dead-reckoning only (no
        # GPS since it's disabled indoors — see gz_modifications.bash). That
        # free-drifts without bound and previously ended in a real crash
        # after ~100s. Rather than keep commanding a stale hold setpoint
        # against an unknown true position, command LAND once and stop.
        self._vio_last_good_time = None
        self._vio_failsafe_timeout_s = 2.0
        self._vio_failsafe_triggered = False

        # Mode engagement is a BOUNDED, single-shot affair: request OFFBOARD
        # (and arm) only a few times right after takeoff starts, then never
        # again. If we kept re-issuing VEHICLE_CMD_DO_SET_MODE(OFFBOARD) every
        # second (as a previous version of this file did), any RC/pilot mode
        # change (or PX4 failsafe) gets immediately fought and overridden by
        # us, which is exactly why the vehicle kept climbing into the ceiling
        # instead of respecting a mode switch. Once _mode_engage_done is True
        # we must not touch VEHICLE_CMD_DO_SET_MODE again for this flight.
        self._mode_engage_attempts = 0
        self._mode_engage_max_attempts = 5
        self._mode_engage_done = False
        self._last_mode_engage_time = None
        self._arm_attempts = 0
        self._arm_max_attempts = 100
        self._last_arm_time = None

        # Ramp state for a smooth, hard-clamped climb (never commands an
        # altitude setpoint beyond the requested TakeoffHeight).
        self._takeoff_climb_rate = 0.3  # m/s, gentle indoor climb
        self._takeoff_start_time = None
        self._takeoff_start_alt = None

    def slam_localization_odom_callback(self, msg):
        self.pose_with_covariance = msg

    def vehicle_status_callback(self, msg):

        # update navigation status
        self.nav_state = msg.nav_state

        # update arm state
        self.arm_state = msg.arming_state

        # Arming is driven by the offboard pre-stream timer (PX4 example order:
        # stream setpoints → DO_SET_MODE(OFFBOARD) → arm), not from status alone.
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

            # RTAB-Map can occasionally emit a spurious, physically-impossible
            # pose (a bad frame-to-frame registration) that is NOT all-zero,
            # so it slips past the "odom lost" check above. PX4's EKF trusts
            # external vision heavily (EKF2_EVP_NOISE/EKF2_EVA_NOISE are
            # small), so forwarding one bad sample can trigger a large,
            # sudden position/attitude correction — this is what caused the
            # vehicle to spin/drift and drop out of OFFBOARD while the
            # reported altitude briefly showed double-digit garbage values.
            # Reject samples that imply an impossible jump since the last
            # good one and just hold the previous good pose instead.
            p = msg.pose.pose.position
            q_new = np.array([msg.pose.pose.orientation.x,
                               msg.pose.pose.orientation.y,
                               msg.pose.pose.orientation.z,
                               msg.pose.pose.orientation.w], dtype=np.float64)
            is_outlier = False
            if self._vio_last_good_pos is not None:
                jump_m = np.linalg.norm(
                    np.array([p.x, p.y, p.z]) - self._vio_last_good_pos)
                if jump_m > self._vio_max_jump_m:
                    is_outlier = True
                elif self._vio_last_good_quat is not None:
                    rel = R.from_quat(q_new) * R.from_quat(self._vio_last_good_quat).inv()
                    angle_deg = np.degrees(np.linalg.norm(rel.as_rotvec()))
                    if angle_deg > self._vio_max_angle_deg:
                        is_outlier = True

            if is_outlier:
                self._vio_outlier_count += 1
                print(time.time(), "  VIO outlier rejected (bad RTAB-Map "
                      "sample), holding last good pose. count=",
                      self._vio_outlier_count)
            else:
                self.vio_odometry = msg
                self.IS_ODOM_LOST = False
                self._vio_last_good_pos = np.array([p.x, p.y, p.z])
                self._vio_last_good_quat = q_new
                self._vio_last_good_time = self.get_clock().now()
        else:
            # activating recovery mode if ODOM is lost 
            self.IS_ODOM_LOST = True

        # saving vio last timestamp
        self.vio_time_stamp = 0  # PX4 stamps on receive; ROS sim-time lags PX4 hrt (~COM_OF_LOSS_T)

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
        self.cmd_vel_timestamp = self.get_clock().now()
        if self.IS_ODOM_LOST:
            self.cmd_vel_msg = Twist()
            self.cmd_vel_msg.angular.z = 0.2
        else:
            self.cmd_vel_msg = msg

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

    def timer_callback(self):

        # Vision-loss safety failsafe takes priority over everything else:
        # once it fires we stop commanding OFFBOARD setpoints for good and
        # let PX4's own LAND mode (baro + IMU, no vision needed) bring the
        # vehicle down instead of free-drifting on dead-reckoning forever.
        if self._check_vio_failsafe():
            return

        # Always keep an offboard heartbeat while mission control is enabled so
        # PX4 does not latch offboard_control_signal_lost during takeoff.
        if self.OffboardControllEnable and not self.takeOff_complete:
            self._publish_takeoff_setpoint()
            return

        if self.OffboardControllEnable and self.takeOff_complete:
            # If the pilot/RC (or a PX4 failsafe) has taken the vehicle out of
            # OFFBOARD, stop publishing setpoints entirely instead of quietly
            # continuing to stream them — we already stopped re-requesting
            # OFFBOARD mode (see _publish_takeoff_setpoint), and not publishing
            # setpoints here avoids any confusion if the pilot switches back.
            if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                return

            # Prefer frozen takeoff hover pose; fall back to live odom if missing.
            hold = self._hold_pose
            if hold is None and self.vehicle_odometry is not None:
                hold = [
                    float(self.vehicle_odometry.position[0]),
                    float(self.vehicle_odometry.position[1]),
                    self.takeOffHeight,
                ]
                self._hold_pose = hold

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

            # A "new trajectory" means Nav2 is actually commanding motion, not
            # just recently publishing an idle/keep-alive zero Twist. Gate on
            # both recency AND magnitude so an idle-but-recent cmd_vel does not
            # knock us out of position-hold into a driftier velocity setpoint.
            cmd_vel_age_s = abs(
                (self.cmd_vel_timestamp.seconds_nanoseconds()[0] +
                 self.cmd_vel_timestamp.seconds_nanoseconds()[1] / 1e9) -
                (self.get_clock().now().seconds_nanoseconds()[0] +
                 self.get_clock().now().seconds_nanoseconds()[1] / 1e9)
            )
            cmd_vel_is_moving = (
                abs(self.cmd_vel_msg.linear.x) > 0.02 or
                abs(self.cmd_vel_msg.linear.y) > 0.02 or
                abs(self.cmd_vel_msg.linear.z) > 0.02 or
                abs(self.cmd_vel_msg.angular.z) > 0.02
            )
            new_trajectory_active = (cmd_vel_age_s < 0.5) and cmd_vel_is_moving

            if new_trajectory_active:
                # Follow the newly received trajectory via velocity control.
                offboard_msg = OffboardControlMode()
                time_stamp = 0
                offboard_msg.timestamp = time_stamp
                offboard_msg.position=True
                offboard_msg.velocity=True
                self.publisher_offboard_control_mode.publish(offboard_msg)
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.timestamp = time_stamp
                trajectory_msg.position[0] = float('nan')
                trajectory_msg.position[1] = float('nan')
                trajectory_msg.position[2] = hold[2] if hold else self.takeOffHeight
                trajectory_msg.velocity[0] = self.linear_Rp[0]
                trajectory_msg.velocity[1] = self.linear_Rp[1]
                trajectory_msg.velocity[2] = float('nan')
                trajectory_msg.yaw = float('nan')
                trajectory_msg.yawspeed = self.angular_Rp[2]
                print(time.time(),"  publishing the recived cmd_vel into px4")
                self.publisher_trajectory_setpoint.publish(trajectory_msg)
                self._was_following = True

            else:
                # No active new trajectory: keep re-sending the SAME fixed 3D
                # position setpoint (position=True, velocity=False) every
                # tick, and never touch the flight mode. If we just finished
                # following a trajectory, freeze the hold pose at the
                # quadrotor's last actual position instead of snapping back
                # to the original takeoff point.
                if self._was_following and self.vehicle_odometry is not None:
                    hold = [
                        float(self.vehicle_odometry.position[0]),
                        float(self.vehicle_odometry.position[1]),
                        float(self.vehicle_odometry.position[2]),
                    ]
                    self._hold_pose = hold
                    self._was_following = False

                offboard_msg = OffboardControlMode()
                time_stamp = 0
                offboard_msg.timestamp = time_stamp
                offboard_msg.position=True
                offboard_msg.velocity=False
                self.publisher_offboard_control_mode.publish(offboard_msg)
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.timestamp = time_stamp
                trajectory_msg.position[0] = hold[0]
                trajectory_msg.position[1] = hold[1]
                trajectory_msg.position[2] = hold[2]
                trajectory_msg.velocity[0] = float('nan')
                trajectory_msg.velocity[1] = float('nan')
                trajectory_msg.velocity[2] = float('nan')
                trajectory_msg.yaw = float('nan')
                trajectory_msg.yawspeed = float('nan')
                self.publisher_trajectory_setpoint.publish(trajectory_msg)
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
            vehicle_command.timestamp = 0  # PX4 stamps on receive
            self.publisher_vehicle_command.publish(vehicle_command)
            print(time.time(),"  publishing arming command")

    def _check_vio_failsafe(self):
        """Return True (and stop all OFFBOARD setpoint publishing for good)
        once vision has been lost/rejected continuously for too long.

        Without vision, PX4 has no absolute position aiding (GPS is
        disabled indoors, see gz_modifications.bash) and free-drifts on
        IMU+baro dead-reckoning alone. Continuing to command a stale
        position hold against an increasingly wrong estimate is how a
        previous run ended up climbing/tumbling and crashing after ~100s
        of undetected vision loss. Landing uses PX4's own baro-referenced
        altitude control, which stays safe without vision.
        """
        if self._vio_failsafe_triggered:
            return True

        if self.arm_state != 2:
            return False

        # Fall back to the takeoff start time if vision never came up at all
        # (e.g. RTAB-Map never initialized), so a total vision blackout from
        # the very start of the flight also gets caught, not just a loss
        # partway through.
        baseline_time = self._vio_last_good_time or self._takeoff_start_time
        if baseline_time is None:
            return False

        elapsed_s = (self.get_clock().now() - baseline_time).nanoseconds / 1e9
        if elapsed_s > self._vio_failsafe_timeout_s:
            self._vio_failsafe_triggered = True
            print(
                time.time(),
                f"  VIO lost for {elapsed_s:.1f}s (>{self._vio_failsafe_timeout_s:.1f}s "
                "timeout) — commanding LAND as a safety failsafe and "
                "halting all OFFBOARD setpoints"
            )
            self._command_land()
            return True
        return False

    def _command_land(self):
        vehicle_command = VehicleCommand()
        vehicle_command.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        vehicle_command.target_system = 1
        vehicle_command.target_component = 1
        vehicle_command.source_system = 1
        vehicle_command.source_component = 1
        vehicle_command.from_external = True
        vehicle_command.timestamp = 0  # PX4 stamps on receive
        self.publisher_vehicle_command.publish(vehicle_command)

    def _publish_takeoff_setpoint(self):
        """Stream OffboardControlMode + a ramped, hard-clamped takeoff setpoint."""
        if self.vehicle_odometry is None:
            return

        if self.current_goal is None:
            self.current_goal = self.vehicle_odometry

        # Hold XY at first good estimate; climb to takeoff altitude (NED down).
        if self._hover_xy is None and self.arm_state == 2:
            self._hover_xy = (
                float(self.vehicle_odometry.position[0]),
                float(self.vehicle_odometry.position[1]),
            )
        hover_x = self._hover_xy[0] if self._hover_xy else float(self.vehicle_odometry.position[0])
        hover_y = self._hover_xy[1] if self._hover_xy else float(self.vehicle_odometry.position[1])

        target_alt = -self.takeOffHeight  # positive AGL target, e.g. 1.0m
        z = float(self.vehicle_odometry.position[2])
        alt_m = -z

        # Ramp the altitude setpoint up gently instead of jumping straight to
        # the target. min() HARD-CLAMPS the commanded altitude so it can never
        # exceed target_alt, no matter what the ramp timer computes — this is
        # the concrete fix for the vehicle climbing past the target into the
        # ceiling.
        if self.arm_state == 2:
            if self._takeoff_start_time is None:
                self._takeoff_start_time = self.get_clock().now()
                self._takeoff_start_alt = max(alt_m, 0.0)
            elapsed_s = (self.get_clock().now() - self._takeoff_start_time).nanoseconds / 1e9
            ramp_alt = self._takeoff_start_alt + self._takeoff_climb_rate * elapsed_s
            setpoint_alt = min(ramp_alt, target_alt)
        else:
            setpoint_alt = min(alt_m, target_alt)
        setpoint_z = -setpoint_alt

        # timestamp=0: uXRCE replaces with hrt_absolute_time() (see
        # ucdr_deserialize_*). Non-zero ROS sim-time stamps lag PX4 and make
        # COM_OF_LOSS_T treat OffboardControlMode as stale.
        time_stamp = 0
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = time_stamp
        offboard_msg.position = True
        offboard_msg.velocity = False
        self.publisher_offboard_control_mode.publish(offboard_msg)

        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = time_stamp
        trajectory_msg.position[0] = hover_x
        trajectory_msg.position[1] = hover_y
        trajectory_msg.position[2] = setpoint_z
        trajectory_msg.velocity[0] = float('nan')
        trajectory_msg.velocity[1] = float('nan')
        trajectory_msg.velocity[2] = float('nan')
        trajectory_msg.yaw = float('nan')
        trajectory_msg.yawspeed = 0.0
        self.publisher_trajectory_setpoint.publish(trajectory_msg)

        self._offboard_setpoint_counter += 1

        # BOUNDED, single-shot mode engagement. PX4/px4_ros_com example
        # switches to OFFBOARD once after ~1s (>=10 setpoints) of streaming,
        # then arms — it never re-issues DO_SET_MODE afterwards. We mirror
        # that: try up to _mode_engage_max_attempts times (1s apart) if we
        # are not yet in OFFBOARD, then STOP FOREVER so a pilot RC mode
        # change (or PX4 failsafe) is not immediately fought and reverted.
        if not self._mode_engage_done and self._offboard_setpoint_counter >= 11:
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self._mode_engage_done = True
            else:
                now = self.get_clock().now()
                if (self._last_mode_engage_time is None or
                        (now - self._last_mode_engage_time).nanoseconds > 1e9):
                    self._last_mode_engage_time = now
                    self._mode_engage_attempts += 1
                    self.change_to_offboard()
                    if self._mode_engage_attempts >= self._mode_engage_max_attempts:
                        self._mode_engage_done = True
                        print(time.time(), "  gave up requesting OFFBOARD mode after",
                              self._mode_engage_attempts, "attempts; will not retry "
                              "(so RC/pilot mode changes are respected)")

        # Arming has no effect on flight mode, so bounded retries here don't
        # fight a pilot's RC override — safe to keep retrying a bit longer.
        if self.arm_state == 1 and self._arm_attempts < self._arm_max_attempts:
            now = self.get_clock().now()
            if (self._last_arm_time is None or
                    (now - self._last_arm_time).nanoseconds > 1e9):
                self._last_arm_time = now
                self._arm_attempts += 1
                self.arm()

        # Periodic takeoff feedback (altitude AGL ≈ -z in NED).
        now = self.get_clock().now()
        if (self._last_takeoff_feedback is None or
                (now - self._last_takeoff_feedback).nanoseconds > 5e8):
            self._last_takeoff_feedback = now
            print(
                time.time(),
                f"  takeoff feedback: alt={alt_m:.2f}m setpoint={setpoint_alt:.2f}m "
                f"target={target_alt:.2f}m nav_state={self.nav_state} "
                f"armed={self.arm_state} "
                f"offboard={self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD}"
            )

        # Reach ~1m then freeze the hold pose and stop climbing logic. Read
        # the quadrotor's ACTUAL last position (from vehicle_odometry) right
        # now instead of snapping to the idealized (hover_x, hover_y,
        # takeOffHeight) target — real x/y/z can be up to the 0.1 m
        # completion tolerance away from that target, and holding the exact
        # position it is already at avoids a small corrective "snap".
        if self.arm_state == 2 and abs(target_alt - alt_m) <= 0.1:
            self.take_off_ground = False
            self.takeOff_complete = True
            last_x = float(self.vehicle_odometry.position[0])
            last_y = float(self.vehicle_odometry.position[1])
            last_z = float(self.vehicle_odometry.position[2])
            self._hold_pose = [last_x, last_y, last_z]
            print(
                time.time(),
                f"  takeOff completed — holding at last position "
                f"(x={last_x:.2f}, y={last_y:.2f}, z={last_z:.2f})"
            )

    def takeOff(self, z):

        # Keep legacy entry point; timer owns the continuous stream now.
        if self.OffboardControllEnable:
            print(time.time(),"  takeing off the ground.")
            self._publish_takeoff_setpoint()

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
            vehicle_command.timestamp = 0  # PX4 stamps on receive
            self.publisher_vehicle_command.publish(vehicle_command)

    def publish_setpoints_before_chage_to_offborad(self):

        # PX4 needs a pre-stream of BOTH OffboardControlMode and TrajectorySetpoint
        # before VEHICLE_CMD_DO_SET_MODE(OFFBOARD) will stick.
        if self.OffboardControllEnable:
            for i in range(100):
                offboard_msg = OffboardControlMode()
                time_stamp = 0  # PX4 stamps on receive; ROS sim-time lags PX4 hrt (~COM_OF_LOSS_T)
                offboard_msg.timestamp = time_stamp
                offboard_msg.position=True
                offboard_msg.velocity=False
                self.publisher_offboard_control_mode.publish(offboard_msg)
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.timestamp = time_stamp
                if self.current_goal is not None:
                    trajectory_msg.position[0] = self.current_goal.position[0]
                    trajectory_msg.position[1] = self.current_goal.position[1]
                else:
                    trajectory_msg.position[0] = 0.0
                    trajectory_msg.position[1] = 0.0
                trajectory_msg.position[2] = self.takeOffHeight
                trajectory_msg.velocity[0] = float('nan')
                trajectory_msg.velocity[1] = float('nan')
                trajectory_msg.velocity[2] = float('nan')
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
