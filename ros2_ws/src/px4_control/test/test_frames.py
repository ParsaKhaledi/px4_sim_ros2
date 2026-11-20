"""Frame and covariance conversions. These tests do not start ROS or Gazebo."""

import math

import numpy as np
import pytest

from px4_control.frames import (
    R_FRD_FROM_FLU,
    R_NED_FROM_ENU,
    CameraMount,
    attitude_from_enu_quat,
    body_from_mounted_pose,
    body_horizontal_to_ned,
    body_offset_to_enu,
    camera_from_body_pose,
    command_age,
    diagonal_variances,
    enu_to_ned,
    enu_yaw_to_ned,
    flu_to_frd,
    ned_to_enu,
    quat_wxyz_to_rot,
    quat_xyzw_to_rot,
    rotate_covariance,
    rot_to_quat_wxyz,
    rot_to_quat_xyzw,
    rpy_to_rot,
    yaw_ned_from_enu_quat,
)


def test_enu_yaw_converts_to_ned():
    # ENU yaw 0 faces East, which is NED yaw +90 degrees.
    assert enu_yaw_to_ned(0.0) == pytest.approx(math.pi / 2.0)
    # Facing north in ENU is +90 degrees, the identity heading in NED.
    assert enu_yaw_to_ned(math.pi / 2.0) == pytest.approx(0.0, abs=1e-9)


def test_enu_ned_roundtrip():
    enu = np.array([1.0, 2.0, 3.0])
    ned = enu_to_ned(enu)
    assert ned == pytest.approx(np.array([2.0, 1.0, -3.0]))
    assert ned_to_enu(ned) == pytest.approx(enu)


def test_covariance_rotation_stays_positive():
    # The old offboard script multiplied the variance vector by diag(1, -1, -1),
    # which made the y and z variances negative. R Σ R.T must not do that.
    sigma = np.array([
        [2.0, 0.5, 0.0],
        [0.5, 3.0, 0.0],
        [0.0, 0.0, 4.0],
    ])
    rotated = rotate_covariance(sigma, R_NED_FROM_ENU)
    assert rotated == pytest.approx(R_NED_FROM_ENU @ sigma @ R_NED_FROM_ENU.T)
    assert not np.allclose(rotated, sigma)
    assert diagonal_variances(rotated) == pytest.approx(np.array([3.0, 2.0, 4.0]))
    assert np.all(np.linalg.eigvalsh(rotated) >= -1e-9)
    bad = R_FRD_FROM_FLU @ np.array([2.0, 3.0, 4.0])
    assert bad[1] < 0.0 and bad[2] < 0.0


def test_body_covariance_signs_square_away():
    sigma = np.diag([1.5, 2.5, 3.5])
    rotated = rotate_covariance(sigma, R_FRD_FROM_FLU)
    assert diagonal_variances(rotated) == pytest.approx(np.array([1.5, 2.5, 3.5]))
    assert flu_to_frd(np.array([1.0, 2.0, 3.0])) == pytest.approx(np.array([1.0, -2.0, -3.0]))


def test_level_north_is_ned_identity_yaw():
    # ROS yaw +90 deg about ENU up points the nose north.
    half = math.pi / 4.0
    quat_xyzw = np.array([0.0, 0.0, math.sin(half), math.cos(half)])
    attitude = attitude_from_enu_quat(quat_xyzw)
    assert attitude is not None
    forward_ned = attitude.rotation[:, 0]
    assert forward_ned == pytest.approx(np.array([1.0, 0.0, 0.0]), abs=1e-6)
    assert attitude.yaw == pytest.approx(0.0, abs=1e-6)
    assert yaw_ned_from_enu_quat(quat_xyzw) == pytest.approx(0.0, abs=1e-6)


def test_identity_enu_faces_east():
    attitude = attitude_from_enu_quat(np.array([0.0, 0.0, 0.0, 1.0]))
    assert attitude is not None
    forward_ned = attitude.rotation[:, 0]
    assert forward_ned == pytest.approx(np.array([0.0, 1.0, 0.0]), abs=1e-6)
    assert attitude.yaw == pytest.approx(math.pi / 2.0, abs=1e-6)


def test_quaternion_matrix_roundtrip():
    yaw = 0.7
    rotation = quat_xyzw_to_rot(np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]))
    quat = rot_to_quat_wxyz(rotation)
    recovered = quat_xyzw_to_rot(np.array([quat[1], quat[2], quat[3], quat[0]]))
    assert recovered == pytest.approx(rotation, abs=1e-6)


def test_cmd_vel_uses_yaw_only_user_formula():
    psi = 0.4
    vx, vy, wz = 0.5, -0.2, 0.3
    vy_right = -vy
    v_n = vx * math.cos(psi) - vy_right * math.sin(psi)
    v_e = vx * math.sin(psi) + vy_right * math.cos(psi)
    got_n, got_e, yawspeed = body_horizontal_to_ned(vx, vy, wz, psi)
    assert got_n == pytest.approx(v_n)
    assert got_e == pytest.approx(v_e)
    assert yawspeed == pytest.approx(-wz)
    # Facing north, left is west.
    north, east, _rate = body_horizontal_to_ned(0.0, 1.0, 0.0, 0.0)
    assert north == pytest.approx(0.0)
    assert east == pytest.approx(-1.0)


def test_body_offset_matches_velocity_rotation():
    yaw = 0.3
    offset = body_offset_to_enu(1.2, -0.4, 0.5, yaw)
    v_n, v_e, _rate = body_horizontal_to_ned(1.2, -0.4, 0.0, yaw)
    assert offset == pytest.approx(np.array([v_e, v_n, 0.5]))


def test_command_age_is_a_plain_subtraction():
    # The old hold branch divided (seconds + nanoseconds) by 1e9.
    assert command_age(10.0, 10.4) == pytest.approx(0.4)
    assert command_age(10.0, 10.6) >= 0.5


def test_zero_quaternion_is_unusable():
    assert attitude_from_enu_quat(np.zeros(4)) is None


def test_tilted_attitude_matrix_matches_quaternion():
    rng = np.random.default_rng(7)
    for _ in range(30):
        axis = rng.normal(size=3)
        axis = axis / np.linalg.norm(axis)
        angle = float(rng.uniform(-math.pi, math.pi))
        skew = np.array([
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ])
        rotation = np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)
        attitude = attitude_from_enu_quat(rot_to_quat_xyzw(rotation))
        assert attitude is not None
        assert quat_wxyz_to_rot(attitude.quaternion_wxyz) == pytest.approx(attitude.rotation, abs=1e-6)
        assert quat_xyzw_to_rot(rot_to_quat_xyzw(attitude.rotation)) == pytest.approx(
            attitude.rotation, abs=1e-6
        )


def test_camera_mount_round_trip():
    # The bridge does not apply this. RTAB-Map already publishes the body.
    mount = CameraMount()
    position = np.array([0.4, -0.2, 1.5])
    quat = rot_to_quat_xyzw(rpy_to_rot(0.2, -0.3, 0.7))
    linear = np.array([0.3, -0.1, 0.05])
    angular = np.array([0.02, -0.04, 0.2])
    cam_p, cam_q, cam_v, cam_w = camera_from_body_pose(position, quat, linear, angular, mount)
    back_p, back_q, back_v, back_w = body_from_mounted_pose(cam_p, cam_q, cam_v, cam_w, mount)
    assert back_p == pytest.approx(position, abs=1e-9)
    assert quat_xyzw_to_rot(back_q) == pytest.approx(quat_xyzw_to_rot(quat), abs=1e-9)
    assert back_v == pytest.approx(linear, abs=1e-9)
    assert back_w == pytest.approx(angular, abs=1e-9)
