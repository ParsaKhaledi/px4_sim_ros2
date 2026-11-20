"""ENU/NED, FLU/FRD, yaw, quaternion order, and covariance. No simulator."""

from __future__ import annotations

import math

import numpy as np
import pytest

from px4_control.frames import (
    R_FRD_FROM_FLU,
    R_NED_FROM_ENU,
    CameraMount,
    attitude_from_enu_quat,
    body_from_mounted_pose,
    camera_from_body_pose,
    diagonal_variances,
    enu_to_ned,
    enu_yaw_to_ned,
    flu_to_frd,
    frd_to_flu,
    ned_to_enu,
    quat_wxyz_to_rot,
    quat_xyzw_to_rot,
    rotate_covariance,
    rot_to_quat_wxyz,
    rot_to_quat_xyzw,
    rpy_to_rot,
)
from px4_control.vision_bridge import OdomSample, VisionOdometryBridge


def _yaw_quat(yaw: float) -> np.ndarray:
    return rot_to_quat_xyzw(rpy_to_rot(0.0, 0.0, yaw))


def _sample(position, quat, linear=None, angular=None, child="base_link") -> OdomSample:
    zeros = np.zeros(3)
    return OdomSample(
        stamp_sec=1.0,
        position_enu=np.asarray(position, dtype=float),
        quat_xyzw=np.asarray(quat, dtype=float),
        linear_flu=zeros if linear is None else np.asarray(linear, dtype=float),
        angular_flu=zeros if angular is None else np.asarray(angular, dtype=float),
        pose_covariance=np.diag([0.1, 0.2, 0.3, 0.01, 0.01, 0.01]).reshape(36),
        twist_covariance=np.diag([0.05, 0.06, 0.07, 0.01, 0.01, 0.01]).reshape(36),
        child_frame_id=child,
    )


def test_position_and_velocity_axes():
    assert enu_to_ned(np.array([1.0, 2.0, 3.0])) == pytest.approx([2.0, 1.0, -3.0])
    assert ned_to_enu(enu_to_ned(np.array([1.0, 2.0, 3.0]))) == pytest.approx([1.0, 2.0, 3.0])
    assert flu_to_frd(np.array([1.0, 2.0, 3.0])) == pytest.approx([1.0, -2.0, -3.0])
    assert frd_to_flu(flu_to_frd(np.array([4.0, -5.0, 6.0]))) == pytest.approx([4.0, -5.0, 6.0])
    bridge = VisionOdometryBridge()
    visual = bridge.push(_sample([1.0, 2.0, 3.0], _yaw_quat(0.0), [1.0, 2.0, 3.0], [0.1, 0.2, 0.3]))
    assert visual is not None
    assert visual.position_ned == pytest.approx([2.0, 1.0, -3.0])
    assert visual.velocity_frd == pytest.approx([1.0, -2.0, -3.0])
    assert visual.angular_velocity_frd == pytest.approx([0.1, -0.2, -0.3])


def test_yaw_and_quaternion_order():
    # ENU yaw +90 deg faces north, which is NED yaw 0.
    north = _yaw_quat(math.pi / 2.0)
    assert enu_yaw_to_ned(math.pi / 2.0) == pytest.approx(0.0, abs=1e-9)
    attitude = attitude_from_enu_quat(north)
    assert attitude is not None
    assert attitude.yaw == pytest.approx(0.0, abs=1e-6)
    # Facing north is the NED identity, so the PX4 quaternion is w=1.
    assert attitude.quaternion_wxyz == pytest.approx([1.0, 0.0, 0.0, 0.0], abs=1e-6)
    # ROS stores xyzw. PX4 stores wxyz. Same rotation, w in the other slot.
    matrix = quat_xyzw_to_rot(north)
    xyzw = rot_to_quat_xyzw(matrix)
    wxyz = rot_to_quat_wxyz(matrix)
    assert wxyz[0] == pytest.approx(xyzw[3])
    assert wxyz[1:] == pytest.approx(xyzw[:3])
    assert list(wxyz) != pytest.approx(list(xyzw))
    # Identity ENU faces East: NED yaw +90 deg, not the NED identity quaternion.
    east = attitude_from_enu_quat(np.array([0.0, 0.0, 0.0, 1.0]))
    assert east is not None
    assert east.yaw == pytest.approx(math.pi / 2.0, abs=1e-6)
    assert abs(east.quaternion_wxyz[0]) == pytest.approx(math.sqrt(0.5), abs=1e-6)
    assert east.quaternion_wxyz != pytest.approx([1.0, 0.0, 0.0, 0.0], abs=1e-3)


def test_round_trip_and_tilted_attitudes_agree():
    rng = np.random.default_rng(7)
    for _ in range(30):
        position = rng.normal(size=3)
        assert ned_to_enu(enu_to_ned(position)) == pytest.approx(position)
        velocity = rng.normal(size=3)
        assert frd_to_flu(flu_to_frd(velocity)) == pytest.approx(velocity)
        axis = rng.normal(size=3)
        axis = axis / np.linalg.norm(axis)
        angle = float(rng.uniform(-math.pi, math.pi))
        skew = np.array([
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ])
        rotation = np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)
        quat = rot_to_quat_xyzw(rotation)
        attitude = attitude_from_enu_quat(quat)
        assert attitude is not None
        assert quat_wxyz_to_rot(attitude.quaternion_wxyz) == pytest.approx(attitude.rotation, abs=1e-6)
        assert quat_xyzw_to_rot(rot_to_quat_xyzw(attitude.rotation)) == pytest.approx(attitude.rotation, abs=1e-6)


def test_covariance_is_rotated_not_copied():
    sigma = np.array([
        [1.0, 0.4, 0.0],
        [0.4, 4.0, 0.0],
        [0.0, 0.0, 9.0],
    ])
    rotated = rotate_covariance(sigma, R_NED_FROM_ENU)
    assert rotated == pytest.approx(R_NED_FROM_ENU @ sigma @ R_NED_FROM_ENU.T)
    assert diagonal_variances(rotated) == pytest.approx([4.0, 1.0, 9.0])
    assert not np.allclose(rotated, sigma)
    # Element-wise sign flips of a variance vector go negative. The rotation does not.
    flipped = R_FRD_FROM_FLU @ np.array([1.0, 4.0, 9.0])
    assert flipped[1] < 0.0 and flipped[2] < 0.0
    body = rotate_covariance(np.diag([1.0, 4.0, 9.0]), R_FRD_FROM_FLU)
    assert diagonal_variances(body) == pytest.approx([1.0, 4.0, 9.0])
    assert np.all(np.linalg.eigvalsh(body) > 0.0)


def test_camera_mount_round_trip_through_the_bridge():
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

    bridge = VisionOdometryBridge(camera_mount=mount)
    body = bridge.push(_sample(position, quat, linear, angular, child="base_link"))
    bridge_cam = VisionOdometryBridge(camera_mount=mount)
    housed = bridge_cam.push(_sample(cam_p, cam_q, cam_v, cam_w, child="camera_link"))
    assert body is not None and housed is not None
    assert housed.position_ned == pytest.approx(body.position_ned, abs=1e-8)
    assert housed.quaternion_wxyz == pytest.approx(body.quaternion_wxyz, abs=1e-8)
    assert housed.velocity_frd == pytest.approx(body.velocity_frd, abs=1e-8)
    assert housed.yaw_ned == pytest.approx(body.yaw_ned, abs=1e-8)
