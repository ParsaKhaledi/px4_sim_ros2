"""Single ROS node that owns every PX4 topic.

OffboardControlMode and TrajectorySetpoint are published together on a fixed
timer. The mode switch waits until that stream has been alive at >= 2 Hz for
a second. Vision samples keep the SLAM stamp and are dropped on tracking loss.
"""

from __future__ import annotations

import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import Trigger

from px4_control.arming import (
    ack_failure_text,
    combine_failure,
    prearm_block_reason,
    sim_preflight_decision,
)
from px4_control.frames import (
    body_horizontal_to_ned,
    body_offset_to_enu,
    enu_to_ned,
    frd_to_flu,
    ned_frd_to_enu_flu_rotation,
    ned_to_enu,
    quat_wxyz_to_rot,
    rot_to_quat_xyzw,
    wrap_pi,
    yaw_ned_from_rotation,
)
from px4_control.geofence import (
    goal_rejection,
    limit_planar_velocity,
    load_wall_file,
    max_speed_along,
    path_rejection,
)
from px4_control.mavlink_params import read_params
from px4_control.mode_switch import NAV_NAMES, OffboardStreamGate, parse_mode
from px4_control.motion import Limits, MotionExecutive, Phase, Snapshot
from px4_control.params_loader import estimation_mode_from_env, normalize_estimation_mode, yaw_rate_from_env
from px4_control.px4_params import READBACK_PARAMS, expected_sim_params, readback_mismatch
from px4_control.topics import load_px4_topics, subscription_names
from px4_control.vision_bridge import OdomSample, VisionOdometryBridge
from px4_control_interfaces.action import GoTo, Hold, Land, Takeoff
from px4_control_interfaces.msg import VehicleState
from px4_control_interfaces.srv import Arm, SetMode
from px4_msgs.msg import (
    FailsafeFlags,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleOdometry,
    VehicleStatus,
)

_NAN = float('nan')


def _px4_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


def _fill3(target, values) -> None:
    for index in range(3):
        target[index] = float(values[index])


class Px4ControlNode(Node):
    """PX4 I/O plus the mission actions."""

    def __init__(self) -> None:
        super().__init__('px4_control')
        self._cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._declare()
        self._estimation_mode = normalize_estimation_mode(self.get_parameter('estimation_mode').value)
        limits = Limits(
            v_xy=float(self.get_parameter('max_horizontal_speed').value),
            v_z=float(self.get_parameter('max_vertical_speed').value),
            a_max=float(self.get_parameter('max_accel').value),
            a_brake=float(self.get_parameter('a_brake').value),
            yaw_rate=math.radians(float(self.get_parameter('max_yaw_rate_deg_s').value)),
            yaw_accel=math.radians(float(self.get_parameter('max_yaw_accel_deg_s2').value)),
            settle_pos=float(self.get_parameter('settle_position_m').value),
            settle_yaw=math.radians(float(self.get_parameter('settle_yaw_deg').value)),
            settle_time=float(self.get_parameter('settle_time_s').value),
            cmd_timeout=float(self.get_parameter('cmd_vel_timeout_s').value),
        )
        self._motion = MotionExecutive(limits)
        self._motion.speed_cap = self._speed_cap
        self._gate = OffboardStreamGate(
            min_hz=float(self.get_parameter('offboard_min_hz').value),
            warmup_s=float(self.get_parameter('offboard_warmup_s').value),
        )
        self._vision = VisionOdometryBridge(
            timeout_s=float(self.get_parameter('vision_timeout_s').value),
            reset_jump_m=float(self.get_parameter('vision_reset_jump_m').value),
            max_variance=float(self.get_parameter('vision_max_variance').value),
        )
        wall_path = str(self.get_parameter('wall_segments_file').value or '')
        loaded = load_wall_file(wall_path)
        self._walls = loaded.walls
        if loaded.missing:
            self.get_logger().warning(f'wall file {loaded.path} is missing; using an empty wall set')
        elif loaded.walls:
            self.get_logger().info(f'loaded {len(loaded.walls)} wall segments from {loaded.path}')
        else:
            self.get_logger().info('no wall file configured; speed is not wall-limited')

        self._pos = None
        self._vel = np.zeros(3)
        self._yaw = 0.0
        self._yaw_rate = 0.0
        self._landed = False
        self._status = None
        self._flags = None
        self._armed = False
        self._nav_state = -1
        self._last_ack = None
        self._px4_us = None
        self._px4_ros_s = None
        self._cmd = None
        self._cmd_time = None
        self._live_topics: dict[str, str] = {}
        self._preflight_client = None
        self._warned_vision_loss = False
        self._topics = load_px4_topics()
        self._param_report: str | None = None
        self._expected_params = expected_sim_params(
            self._estimation_mode,
            ev_ctrl=float(os.environ.get('EKF2_EV_CTRL', '11')),
            ev_delay=float(os.environ.get('EKF2_EV_DELAY', '50')),
        )

        qos = _px4_qos()
        self._offboard_pubs = self._make_publishers(OffboardControlMode, self._topics['offboard_control_mode'], qos)
        self._traj_pubs = self._make_publishers(TrajectorySetpoint, self._topics['trajectory_setpoint'], qos)
        self._cmd_pubs = self._make_publishers(VehicleCommand, self._topics['vehicle_command'], qos)
        self._ev_pubs = self._make_publishers(VehicleOdometry, self._topics['vehicle_visual_odometry'], qos)
        self._subscribe_px4(VehicleStatus, self._topics['vehicle_status'], self._on_status, qos)
        self._subscribe_px4(VehicleOdometry, self._topics['vehicle_odometry'], self._on_odometry, qos)
        self._subscribe_px4(FailsafeFlags, self._topics['failsafe_flags'], self._on_flags, qos)
        self._subscribe_px4(VehicleLandDetected, self._topics['vehicle_land_detected'], self._on_land, qos)
        self._subscribe_px4(VehicleCommandAck, self._topics['vehicle_command_ack'], self._on_ack, qos)
        threading.Thread(target=self._read_back_params, name='px4_param_readback', daemon=True).start()

        cmd_qos = QoSProfile(depth=10)
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value),
            self._on_cmd_vel,
            cmd_qos,
            callback_group=self._cb,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('vision_odom_topic').value),
            self._on_vision,
            cmd_qos,
            callback_group=self._cb,
        )
        self._try_odom_info()

        self._state_pub = self.create_publisher(VehicleState, '/px4_control/state', 10)
        self._odom_pub = self.create_publisher(Odometry, '/px4_control/odom', 10)
        self.create_service(Arm, '/px4_control/arm', self._on_arm, callback_group=self._cb)
        self.create_service(SetMode, '/px4_control/set_mode', self._on_set_mode, callback_group=self._cb)
        self._takeoff_server = ActionServer(
            self, Takeoff, '/px4_control/takeoff', self._exec_takeoff,
            callback_group=self._cb, goal_callback=self._accept, cancel_callback=self._cancel,
        )
        self._land_server = ActionServer(
            self, Land, '/px4_control/land', self._exec_land,
            callback_group=self._cb, goal_callback=self._accept, cancel_callback=self._cancel,
        )
        self._goto_server = ActionServer(
            self, GoTo, '/px4_control/goto', self._exec_goto,
            callback_group=self._cb, goal_callback=self._accept, cancel_callback=self._cancel,
        )
        self._hold_server = ActionServer(
            self, Hold, '/px4_control/hold', self._exec_hold,
            callback_group=self._cb, goal_callback=self._accept, cancel_callback=self._cancel,
        )
        rate = float(self.get_parameter('setpoint_rate_hz').value)
        self.create_timer(1.0 / rate, self._on_timer, callback_group=self._cb)
        self.get_logger().info(
            f'px4_control up, estimation_mode={self._estimation_mode}, '
            f'use_sim_time={self.get_parameter("use_sim_time").value}'
        )

    def _declare(self) -> None:
        defaults = {
            'estimation_mode': estimation_mode_from_env(),
            'setpoint_rate_hz': 20.0,
            'offboard_min_hz': 2.0,
            'offboard_warmup_s': 1.0,
            'cmd_vel_timeout_s': 0.5,
            'cmd_vel_topic': '/cmd_vel',
            'vision_odom_topic': '/rtabmap/odom',
            'vision_odom_info_topic': '/rtabmap/odom_info',
            'vision_timeout_s': 0.3,
            'vision_reset_jump_m': 0.75,
            'vision_max_variance': 25.0,
            'max_horizontal_speed': 1.0,
            'max_vertical_speed': 0.6,
            'max_accel': 1.0,
            'a_brake': 1.0,
            'max_yaw_rate_deg_s': yaw_rate_from_env(),
            'max_yaw_accel_deg_s2': 60.0,
            'settle_position_m': 0.20,
            'settle_yaw_deg': 8.0,
            'settle_time_s': 0.4,
            'drone_radius_m': 0.35,
            'wall_margin_m': 0.40,
            'wall_segments_file': '',
            'preflight_service': '/sim/preflight_check',
            'arm_timeout_s': 25.0,
            'offboard_timeout_s': 10.0,
            'takeoff_timeout_s': 90.0,
            'land_timeout_s': 90.0,
            'goto_timeout_s': 120.0,
        }
        for name, value in defaults.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, value)

    def _make_publishers(self, msg_type, configured: str, qos: QoSProfile):
        names = subscription_names(configured)
        pubs = [self.create_publisher(msg_type, name, qos) for name in names]
        self.get_logger().info(f'publishing {configured} on {names} (best_effort)')
        return pubs

    def _subscribe_px4(self, msg_type, configured: str, callback, qos: QoSProfile) -> None:
        for name in subscription_names(configured):
            self.create_subscription(msg_type, name, lambda msg, topic=name, cb=callback: self._mark_and_call(topic, msg, cb), qos, callback_group=self._cb)

    def _mark_and_call(self, topic: str, msg, callback) -> None:
        base = topic.rsplit('_v', 1)[0] if '_v' in topic.split('/')[-1] else topic
        if self._live_topics.get(base) != topic:
            self._live_topics[base] = topic
            self.get_logger().info(f'using PX4 topic {topic}')
        callback(msg)

    def _try_odom_info(self) -> None:
        try:
            from rtabmap_msgs.msg import OdomInfo
        except ImportError:
            self.get_logger().info('rtabmap_msgs is not installed; vision loss uses the odometry stream itself')
            return
        topic = str(self.get_parameter('vision_odom_info_topic').value)
        self.create_subscription(OdomInfo, topic, self._on_odom_info, 10, callback_group=self._cb)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _now_us(self) -> int:
        now = self._now_s()
        if self._px4_us is None or self._px4_ros_s is None:
            return int(now * 1e6)
        return int(self._px4_us + (now - self._px4_ros_s) * 1e6)

    def _note_px4_time(self, timestamp_us: int) -> None:
        self._px4_us = int(timestamp_us)
        self._px4_ros_s = self._now_s()

    def _on_status(self, msg: VehicleStatus) -> None:
        with self._lock:
            self._note_px4_time(int(msg.timestamp))
            self._status = msg
            self._armed = int(msg.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED)
            self._nav_state = int(msg.nav_state)

    def _on_flags(self, msg: FailsafeFlags) -> None:
        with self._lock:
            self._flags = msg

    def _on_land(self, msg: VehicleLandDetected) -> None:
        with self._lock:
            self._landed = bool(msg.landed)

    def _on_ack(self, msg: VehicleCommandAck) -> None:
        with self._lock:
            self._last_ack = msg

    def _on_odometry(self, msg: VehicleOdometry) -> None:
        rotation = quat_wxyz_to_rot(np.array(msg.q, dtype=float))
        velocity = np.array(msg.velocity, dtype=float)
        if int(msg.velocity_frame) == int(VehicleOdometry.VELOCITY_FRAME_BODY_FRD):
            velocity = rotation @ velocity
        yaw_rate = float(msg.angular_velocity[2])
        with self._lock:
            self._note_px4_time(int(msg.timestamp))
            self._pos = np.array(msg.position, dtype=float)
            self._vel = velocity
            self._yaw = yaw_ned_from_rotation(rotation)
            self._yaw_rate = yaw_rate

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._cmd = msg
            self._cmd_time = self._now_s()

    def _on_vision(self, msg: Odometry) -> None:
        if self._estimation_mode != 'vision':
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = msg.pose.pose
        twist = msg.twist.twist
        sample = OdomSample(
            stamp_sec=stamp,
            position_enu=np.array([pose.position.x, pose.position.y, pose.position.z]),
            quat_xyzw=np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]),
            linear_flu=np.array([twist.linear.x, twist.linear.y, twist.linear.z]),
            angular_flu=np.array([twist.angular.x, twist.angular.y, twist.angular.z]),
            pose_covariance=np.array(msg.pose.covariance, dtype=float),
            twist_covariance=np.array(msg.twist.covariance, dtype=float),
        )
        visual = self._vision.push(sample)
        if visual is None:
            if not self._warned_vision_loss:
                self.get_logger().warning('vision tracking lost; external vision publishing stopped')
                self._warned_vision_loss = True
            return
        self._warned_vision_loss = False
        self._publish_visual(visual)

    def _on_odom_info(self, msg) -> None:
        if bool(getattr(msg, 'lost', False)):
            self._vision.mark_lost()
            if not self._warned_vision_loss:
                self.get_logger().warning('rtabmap odom_info reports tracking lost')
                self._warned_vision_loss = True

    def _publish_visual(self, visual) -> None:
        msg = VehicleOdometry()
        # RTAB-Map's header is sim time. UXRCE_DDS_SYNCT 0 keeps PX4 on that
        # same clock, so the header is the timestamp. Do not overwrite it
        # with the node clock or the wall clock.
        msg.timestamp = max(0, int(round(float(visual.stamp_sec) * 1e6)))
        msg.timestamp_sample = msg.timestamp
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        _fill3(msg.position, visual.position_ned)
        for index in range(4):
            msg.q[index] = float(visual.quaternion_wxyz[index])
        _fill3(msg.velocity, visual.velocity_frd)
        _fill3(msg.angular_velocity, visual.angular_velocity_frd)
        _fill3(msg.position_variance, visual.position_variance)
        _fill3(msg.orientation_variance, visual.orientation_variance)
        _fill3(msg.velocity_variance, visual.velocity_variance)
        msg.reset_counter = int(visual.reset_counter)
        msg.quality = 100
        for pub in self._ev_pubs:
            pub.publish(msg)

    def _snapshot(self) -> Snapshot | None:
        if self._pos is None:
            return None
        return Snapshot(self._pos.copy(), self._vel.copy(), self._yaw, self._yaw_rate, self._landed)

    def _speed_cap(self, position_ned: np.ndarray, direction_ne: np.ndarray) -> float:
        enu = ned_to_enu(position_ned)
        return max_speed_along(
            float(enu[0]),
            float(enu[1]),
            float(direction_ne[1]),
            float(direction_ne[0]),
            self._walls,
            float(self.get_parameter('drone_radius_m').value),
            float(self.get_parameter('wall_margin_m').value),
            float(self.get_parameter('a_brake').value),
        )

    def _on_timer(self) -> None:
        now = self._now_s()
        with self._lock:
            snap = self._snapshot()
            if self._cmd is not None and self._cmd_time is not None and snap is not None and self._motion.accepts_cmd_vel():
                if (now - self._cmd_time) < float(self.get_parameter('cmd_vel_timeout_s').value):
                    v_n, v_e, yaw_rate = body_horizontal_to_ned(
                        self._cmd.linear.x,
                        self._cmd.linear.y,
                        self._cmd.angular.z,
                        snap.yaw_ned,
                    )
                    enu = ned_to_enu(snap.position_ned)
                    v_e, v_n = limit_planar_velocity(
                        v_e,
                        v_n,
                        float(enu[0]),
                        float(enu[1]),
                        self._walls,
                        float(self.get_parameter('drone_radius_m').value),
                        float(self.get_parameter('wall_margin_m').value),
                        float(self.get_parameter('a_brake').value),
                    )
                    self._motion.note_cmd_vel(now, v_n, v_e, yaw_rate)
            if self._estimation_mode == 'vision':
                if self._vision.current(now) is None:
                    self._motion.note_vision_lost(now)
                else:
                    self._motion.note_vision_regained()
            setpoint = self._motion.update(now, snap)
            self._gate.tick(now)
            phase = self._motion.phase
        self._publish_offboard(setpoint)
        self._publish_state(phase)

    def _publish_offboard(self, setpoint) -> None:
        stamp = self._now_us()
        mode = OffboardControlMode()
        mode.timestamp = stamp
        mode.position = bool(setpoint.use_position)
        mode.velocity = True
        mode.acceleration = bool(setpoint.use_position)
        mode.attitude = False
        mode.body_rate = False
        mode.thrust_and_torque = False
        mode.direct_actuator = False
        traj = TrajectorySetpoint()
        traj.timestamp = stamp
        if setpoint.use_position:
            _fill3(traj.position, setpoint.position)
            _fill3(traj.velocity, setpoint.velocity)
            _fill3(traj.acceleration, setpoint.acceleration)
            traj.yaw = float(setpoint.yaw)
        else:
            _fill3(traj.position, (_NAN, _NAN, _NAN))
            _fill3(traj.velocity, setpoint.velocity)
            _fill3(traj.acceleration, (_NAN, _NAN, _NAN))
            traj.yaw = _NAN
        _fill3(traj.jerk, (_NAN, _NAN, _NAN))
        traj.yawspeed = float(setpoint.yaw_rate)
        for pub in self._offboard_pubs:
            pub.publish(mode)
        for pub in self._traj_pubs:
            pub.publish(traj)

    def _publish_state(self, phase: Phase) -> None:
        if self._pos is None:
            return
        # Control uses yaw only. The published ENU pose is level at that yaw.
        enu = ned_to_enu(self._pos)
        vel_enu = ned_to_enu(self._vel)
        quat = rot_to_quat_xyzw(ned_frd_to_enu_flu_rotation(np.array([
            math.cos(self._yaw / 2.0), 0.0, 0.0, math.sin(self._yaw / 2.0)
        ])))
        state = VehicleState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'odom'
        state.arming_state = 0 if self._status is None else int(self._status.arming_state)
        nav_state = self._nav_state if self._nav_state >= 0 else 0
        state.nav_state = int(nav_state)
        state.nav_state_name = NAV_NAMES.get(self._nav_state, str(nav_state))
        state.armed = self._armed
        state.offboard = self._nav_state == int(VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        state.pre_flight_checks_pass = False if self._status is None else bool(self._status.pre_flight_checks_pass)
        state.failsafe = False if self._status is None else bool(self._status.failsafe)
        state.failsafe_reason = prearm_block_reason(self._status, self._flags) or ''
        state.pose_enu.position.x = float(enu[0])
        state.pose_enu.position.y = float(enu[1])
        state.pose_enu.position.z = float(enu[2])
        state.pose_enu.orientation.x = float(quat[0])
        state.pose_enu.orientation.y = float(quat[1])
        state.pose_enu.orientation.z = float(quat[2])
        state.pose_enu.orientation.w = float(quat[3])
        state.twist_enu.linear.x = float(vel_enu[0])
        state.twist_enu.linear.y = float(vel_enu[1])
        state.twist_enu.linear.z = float(vel_enu[2])
        state.yaw_ned = float(self._yaw)
        state.vision_valid = self._vision.tracking and self._estimation_mode == 'vision'
        state.vision_reset_counter = int(self._vision.reset_counter)
        state.estimation_mode = self._estimation_mode
        self._state_pub.publish(state)
        odom = Odometry()
        odom.header = state.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose = state.pose_enu
        # Twist in the message is body FLU, matching nav_msgs convention.
        body = frd_to_flu(quat_wxyz_to_rot(np.array([
            math.cos(self._yaw / 2.0), 0.0, 0.0, math.sin(self._yaw / 2.0)
        ])).T @ self._vel)
        odom.twist.twist.linear.x = float(body[0])
        odom.twist.twist.linear.y = float(body[1])
        odom.twist.twist.linear.z = float(body[2])
        odom.twist.twist.angular.z = float(-self._yaw_rate)
        self._odom_pub.publish(odom)
        if phase != getattr(self, '_logged_phase', None):
            self._logged_phase = phase
            self.get_logger().info(f'motion phase {phase.value}')

    def _send_command(self, command: int, p1: float = 0.0, p2: float = 0.0, p3: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = int(command)
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.param3 = float(p3)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        for pub in self._cmd_pubs:
            pub.publish(msg)

    def _arm_command(self) -> int:
        return int(getattr(VehicleCommand, 'VEHICLE_CMD_COMPONENT_ARM_DISARM', 400))

    def _sim_preflight(self):
        name = str(self.get_parameter('preflight_service').value)
        services = dict(self.get_service_names_and_types())
        if name not in services:
            self.get_logger().warning(f'{name} is not available; continuing with PX4 pre-arm checks only')
            return None
        if 'std_srvs/srv/Trigger' not in services[name]:
            self.get_logger().warning(
                f'{name} has type {services[name]}, expected std_srvs/srv/Trigger; continuing with PX4 checks'
            )
            return sim_preflight_decision(True, False, None, '')
        if self._preflight_client is None:
            self._preflight_client = self.create_client(Trigger, name, callback_group=self._cb)
        if not self._preflight_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f'{name} did not respond; continuing with PX4 pre-arm checks only')
            return None
        future = self._preflight_client.call_async(Trigger.Request())
        deadline = self._deadline(3.0)
        while not future.done() and not self._expired(deadline):
            time.sleep(0.02)
        if not future.done() or future.result() is None:
            self.get_logger().warning(f'{name} timed out; continuing with PX4 pre-arm checks only')
            return None
        response = future.result()
        return sim_preflight_decision(True, True, bool(response.success), str(response.message))

    def _deadline(self, timeout: float) -> float:
        """Node-clock deadline. Sim time when use_sim_time is set."""
        return self._now_s() + float(timeout)

    def _expired(self, deadline: float) -> bool:
        return self._now_s() >= deadline

    def _wait_until(self, predicate, timeout: float) -> bool:
        deadline = self._deadline(timeout)
        while not self._expired(deadline):
            if predicate():
                return True
            time.sleep(0.05)
        return False

    def _do_arm(self, arm: bool, timeout: float):
        from px4_control.arming import ArmDecision

        if arm:
            skipped = self._sim_preflight()
            if skipped is not None and not skipped.success:
                return skipped
            if not self._wait_until(lambda: self._pos is not None and self._status is not None, timeout):
                return ArmDecision(False, 'no vehicle_status or vehicle_odometry received')
            ready = self._wait_until(
                lambda: prearm_block_reason(self._status, self._flags) is None,
                timeout,
            )
            if not ready:
                with self._lock:
                    reason = prearm_block_reason(self._status, self._flags)
                return ArmDecision(False, reason or 'pre_flight_checks_pass is false')
        deadline = self._deadline(timeout)
        command = self._arm_command()
        while not self._expired(deadline):
            self._send_command(command, 1.0 if arm else 0.0)
            time.sleep(0.2)
            with self._lock:
                armed = self._armed
                ack = self._last_ack
                reason = prearm_block_reason(self._status, self._flags)
            if arm and armed:
                return ArmDecision(True, 'armed')
            if not arm and not armed and self._status is not None:
                return ArmDecision(True, 'disarmed')
            if ack is not None and int(ack.command) == command:
                failure = ack_failure_text(int(ack.result))
                if failure and not (arm and armed):
                    return ArmDecision(False, combine_failure(failure, reason))
        with self._lock:
            reason = prearm_block_reason(self._status, self._flags)
        return ArmDecision(False, reason or ('timed out waiting to arm' if arm else 'timed out waiting to disarm'))

    def _ensure_offboard(self, timeout: float):
        from px4_control.arming import ArmDecision

        deadline = self._deadline(timeout)
        while not self._expired(deadline):
            with self._lock:
                ready = self._gate.ready(self._now_s())
                reason = self._gate.reason(self._now_s())
                already = self._nav_state == int(VehicleStatus.NAVIGATION_STATE_OFFBOARD)
            if already:
                return ArmDecision(True, 'offboard')
            if ready:
                break
            time.sleep(0.05)
        else:
            return ArmDecision(False, reason or 'offboard stream is not warm')
        mode = parse_mode('OFFBOARD')
        send_until = self._deadline(timeout)
        while not self._expired(send_until):
            self._send_command(int(VehicleCommand.VEHICLE_CMD_DO_SET_MODE), mode.param1, mode.param2, mode.param3)
            time.sleep(0.2)
            with self._lock:
                if self._nav_state == mode.nav_state:
                    return ArmDecision(True, 'offboard')
                ack = self._last_ack
            if ack is not None and int(ack.command) == int(VehicleCommand.VEHICLE_CMD_DO_SET_MODE):
                failure = ack_failure_text(int(ack.result))
                if failure:
                    return ArmDecision(False, failure)
        return ArmDecision(False, 'timed out waiting for OFFBOARD')

    def _on_arm(self, request: Arm.Request, response: Arm.Response) -> Arm.Response:
        decision = self._do_arm(bool(request.arm), float(self.get_parameter('arm_timeout_s').value))
        response.success = decision.success
        response.message = decision.message
        self.get_logger().info(f'arm request arm={request.arm}: {decision.message}')
        return response

    def _on_set_mode(self, request: SetMode.Request, response: SetMode.Response) -> SetMode.Response:
        try:
            mode = parse_mode(request.mode)
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
            return response
        if mode.name == 'OFFBOARD':
            with self._lock:
                reason = self._gate.reason(self._now_s())
            if reason:
                response.success = False
                response.message = reason
                return response
        self._send_command(int(VehicleCommand.VEHICLE_CMD_DO_SET_MODE), mode.param1, mode.param2, mode.param3)
        ok = self._wait_until(lambda: self._nav_state == mode.nav_state, float(self.get_parameter('offboard_timeout_s').value))
        response.success = ok
        response.message = mode.name if ok else f'timed out waiting for {mode.name}'
        return response

    def _accept(self, _goal):
        return GoalResponse.ACCEPT

    def _cancel(self, _goal):
        return CancelResponse.ACCEPT

    def _read_back_params(self) -> None:
        """Log EKF and arming parameters and refuse to fly if they are wrong."""
        expected = self._expected_params
        names = list(READBACK_PARAMS)
        host = os.environ.get('PX4_MAVLINK_HOST', '127.0.0.1')
        port = int(os.environ.get('PX4_MAVLINK_PORT', '18570'))
        try:
            actual = read_params(names, host=host, port=port)
        except OSError as exc:
            message = f'could not read PX4 parameters on {host}:{port}: {exc}'
            self.get_logger().error(message)
            self._param_report = message
            return
        for name in names:
            if name in actual:
                self.get_logger().info(f'param {name}={actual[name]:g} expected {expected[name]:g}')
            else:
                self.get_logger().error(f'param {name} had no MAVLink reply')
        mismatch = readback_mismatch(actual, expected)
        if mismatch:
            self.get_logger().error(mismatch)
            self._param_report = mismatch
            return
        self.get_logger().info('PX4 parameter read-back matches the sim profile')
        self._param_report = ''

    def _prepare_flight(self, timeout: float):
        from px4_control.arming import ArmDecision

        if not self._wait_until(lambda: self._param_report is not None, min(float(timeout), 20.0)):
            return ArmDecision(False, 'timed out reading PX4 parameters back (EKF2_EV_CTRL, EKF2_GPS_CTRL, EKF2_HGT_REF)')
        if self._param_report:
            return ArmDecision(False, self._param_report)
        armed = self._do_arm(True, timeout)
        if not armed.success:
            return armed
        return self._ensure_offboard(timeout)

    def _wait_goal(self, goal_handle, goal_id: int, timeout: float, feedback_builder):
        deadline = self._deadline(timeout)
        while rclpy.ok() and not self._expired(deadline):
            if goal_handle.is_cancel_requested:
                with self._lock:
                    self._motion.abort('canceled')
                goal_handle.canceled()
                return False, 'canceled'
            with self._lock:
                status = self._motion.poll(goal_id)
                snap = self._snapshot()
            goal_handle.publish_feedback(feedback_builder(snap))
            if status.done:
                if status.success:
                    goal_handle.succeed()
                else:
                    goal_handle.abort()
                return status.success, status.message
            time.sleep(0.05)
        with self._lock:
            self._motion.abort('timed out')
        goal_handle.abort()
        return False, 'timed out'

    def _altitude(self, snap: Snapshot | None) -> float:
        if snap is None:
            return 0.0
        ground = self._motion._ground_d
        if ground is None:
            return float(-snap.position_ned[2])
        return float(-(snap.position_ned[2] - ground))

    def _exec_takeoff(self, goal_handle):
        result = Takeoff.Result()
        height = float(goal_handle.request.height)
        if height <= 0.0:
            result.success = False
            result.message = 'height must be positive'
            goal_handle.abort()
            return result
        timeout = float(self.get_parameter('takeoff_timeout_s').value)
        ready = self._prepare_flight(float(self.get_parameter('arm_timeout_s').value))
        if not ready.success:
            result.success = False
            result.message = ready.message
            goal_handle.abort()
            return result
        with self._lock:
            snap = self._snapshot()
            if snap is None:
                goal_id = None
            else:
                goal_id = self._motion.request_takeoff(self._now_s(), snap, height)
        if goal_id is None:
            result.success = False
            result.message = 'no vehicle_odometry received'
            goal_handle.abort()
            return result
        self.get_logger().info(f'takeoff to {height:.2f} m')

        def feedback(snap):
            msg = Takeoff.Feedback()
            msg.altitude = self._altitude(snap)
            return msg

        ok, message = self._wait_goal(goal_handle, goal_id, timeout, feedback)
        result.success = ok
        result.message = message
        return result

    def _exec_land(self, goal_handle):
        result = Land.Result()
        timeout = float(self.get_parameter('land_timeout_s').value)
        with self._lock:
            snap = self._snapshot()
            goal_id = None if snap is None else self._motion.request_land(self._now_s(), snap)
        if goal_id is None:
            result.success = False
            result.message = 'no vehicle_odometry received'
            goal_handle.abort()
            return result
        self.get_logger().info('land')

        def feedback(snap):
            msg = Land.Feedback()
            msg.altitude = self._altitude(snap)
            return msg

        ok, message = self._wait_goal(goal_handle, goal_id, timeout, feedback)
        if ok:
            disarm = self._do_arm(False, float(self.get_parameter('arm_timeout_s').value))
            if not disarm.success:
                ok = False
                message = disarm.message
                # The action was already marked succeeded inside _wait_goal.
                # The result message still reports the disarm failure.
        result.success = ok
        result.message = message
        return result

    def _exec_hold(self, goal_handle):
        result = Hold.Result()
        duration = float(goal_handle.request.duration_sec)
        with self._lock:
            snap = self._snapshot()
            goal_id = None if snap is None else self._motion.request_hold(self._now_s(), snap, duration)
        if goal_id is None:
            result.success = False
            result.message = 'no vehicle_odometry received'
            goal_handle.abort()
            return result
        started = self._now_s()

        def feedback(_snap):
            msg = Hold.Feedback()
            msg.elapsed_sec = float(self._now_s() - started)
            return msg

        timeout = max(30.0, duration + 60.0)
        ok, message = self._wait_goal(goal_handle, goal_id, timeout, feedback)
        result.success = ok
        result.message = message
        return result

    def _exec_goto(self, goal_handle):
        result = GoTo.Result()
        request = goal_handle.request
        with self._lock:
            snap = self._snapshot()
            if snap is None:
                goal_id = None
                reject = 'no vehicle_odometry received'
            else:
                goal_id, reject = self._start_goto(request, snap)
        if goal_id is None or reject:
            result.success = False
            result.message = reject or 'goto rejected'
            goal_handle.abort()
            return result

        def feedback(snap):
            msg = GoTo.Feedback()
            if snap is None:
                msg.distance_remaining = 0.0
                msg.yaw_error_rad = 0.0
                return msg
            # Feedback is informational. The executive owns the real target.
            msg.distance_remaining = 0.0
            msg.yaw_error_rad = 0.0
            return msg

        ok, message = self._wait_goal(
            goal_handle,
            goal_id,
            float(self.get_parameter('goto_timeout_s').value),
            feedback,
        )
        result.success = ok
        result.message = message
        return result

    def _start_goto(self, request, snap: Snapshot):
        """Must be called with ``self._lock`` held."""
        current_enu = ned_to_enu(snap.position_ned)
        pure_yaw = (
            int(request.frame) == int(GoTo.Goal.FRAME_BODY)
            and bool(request.yaw_valid)
            and abs(float(request.x)) < 1e-6
            and abs(float(request.y)) < 1e-6
            and abs(float(request.z)) < 1e-6
        )
        if pure_yaw:
            goal_id = self._motion.request_turn(self._now_s(), snap, float(request.yaw))
            status = self._motion.poll(goal_id)
            if status.done and not status.success:
                return None, status.message
            return goal_id, ''
        if int(request.frame) == int(GoTo.Goal.FRAME_BODY):
            offset = body_offset_to_enu(float(request.x), float(request.y), float(request.z), snap.yaw_ned)
            target_enu = current_enu + offset
            yaw = None if not request.yaw_valid else wrap_pi(snap.yaw_ned - float(request.yaw))
        else:
            target_enu = np.array([float(request.x), float(request.y), float(request.z)], dtype=float)
            yaw = None if not request.yaw_valid else float(request.yaw)
        radius = float(self.get_parameter('drone_radius_m').value)
        margin = float(self.get_parameter('wall_margin_m').value)
        reason = path_rejection(
            float(current_enu[0]), float(current_enu[1]),
            float(target_enu[0]), float(target_enu[1]),
            self._walls, radius, margin,
        )
        if reason is None:
            reason = goal_rejection(float(target_enu[0]), float(target_enu[1]), self._walls, radius, margin)
        if reason:
            return None, reason
        target_ned = enu_to_ned(target_enu)
        goal_id = self._motion.request_goto(self._now_s(), snap, target_ned, yaw)
        status = self._motion.poll(goal_id)
        if status.done and not status.success:
            return None, status.message
        return goal_id, ''


def main() -> None:
    rclpy.init()
    node = Px4ControlNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
