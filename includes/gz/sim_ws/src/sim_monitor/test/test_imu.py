"""IMU frame conversion, source selection, and SDF camera transforms."""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim_monitor.camera_extrinsics import (
    OPTICAL_RPY,
    REFERENCE_X500,
    REPO_GZ,
    camera_transforms,
)
from sim_monitor.checks import expected_imu_hz, minimum_rate_hz
from sim_monitor.imu_frames import flu_enu_quaternion, frd_to_flu, quat_from_rpy, quat_to_rot
from sim_monitor.imu_source import (
    bridge_gazebo_imu,
    configured_imu_publishers,
    default_tf_pairs,
    diagonal_covariance,
    normalize_imu_source,
    optical_frame,
    ros_stamp_ns,
)
from sim_monitor.px4_imu_relay import px4_topic

G = 9.80665


def _by_child(transforms, child):
    matches = [item for item in transforms if item.child == child]
    assert len(matches) == 1, child
    return matches[0]


def test_identity_attitude_points_north_and_level():
    rotation = quat_to_rot(flu_enu_quaternion((1.0, 0.0, 0.0, 0.0)))
    np.testing.assert_allclose(rotation[:, 0], [0.0, 1.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(rotation[:, 2], [0.0, 0.0, 1.0], atol=1e-9)


def test_ninety_degree_yaw_points_east():
    q_ned = quat_from_rpy(0.0, 0.0, math.pi / 2.0)
    rotation = quat_to_rot(flu_enu_quaternion(q_ned))
    np.testing.assert_allclose(rotation[:, 0], [1.0, 0.0, 0.0], atol=1e-9)


def test_specific_force_at_rest_is_plus_g_up():
    assert frd_to_flu((1.0, 2.0, 3.0)) == (1.0, -2.0, -3.0)
    flu = frd_to_flu((0.0, 0.0, -G))
    np.testing.assert_allclose(flu, (0.0, 0.0, G), atol=1e-9)


def test_only_one_imu_publisher_per_mode():
    oak = configured_imu_publishers("oak")
    px4 = configured_imu_publishers("px4")
    assert oak == ("ros_gz_bridge",)
    assert px4 == ("px4_imu_relay",)
    assert bridge_gazebo_imu("oak") is True
    assert bridge_gazebo_imu("px4") is False
    assert bridge_gazebo_imu("") is True
    assert normalize_imu_source(None) == "oak"
    sim = (REPO_GZ / "config_gz_bridge_sim.yaml").read_text(encoding="utf-8")
    imu = (REPO_GZ / "config_gz_bridge_imu.yaml").read_text(encoding="utf-8")
    assert 'ros_topic_name: "/imu"' not in sim
    assert sim.count('ros_topic_name: "/imu"') == 0
    assert imu.count('ros_topic_name: "/imu"') == 1


def test_px4_rate_and_optical_frame():
    assert expected_imu_hz({"IMU_SOURCE": "oak", "IMU_RATE_HZ": "200"}) == 200.0
    assert expected_imu_hz({"IMU_SOURCE": "px4"}) == 100.0
    assert minimum_rate_hz("imu", {"IMU_SOURCE": "px4", "PREFLIGHT_MIN_IMU_HZ": "80"}) == 80.0
    assert "camera_rgb_frame" in default_tf_pairs({"CameraType": "rgbd"})
    assert "stereo_left_camera_frame" in default_tf_pairs({"CameraType": "stereo"})
    assert default_tf_pairs({"PREFLIGHT_TF_PAIRS": "world:spawn"}) == "world:spawn"
    assert optical_frame({"CameraType": "rgbd"}) == "camera_rgb_optical_frame"
    assert optical_frame({"CameraType": "stereo"}) == "stereo_left_camera_optical_frame"
    assert optical_frame({"PREFLIGHT_OPTICAL_FRAME": "custom"}) == "custom"


def test_stamp_uses_timesync_only_on_the_ros_clock():
    receive = 5_000_000_000
    assert ros_stamp_ns(4_000_000, None, receive) == receive
    # 1.5 s of PX4 time plus a 3.5 s offset lands on the receive time.
    assert ros_stamp_ns(1_500_000, 3_500_000, receive) == receive
    # A wall-clock offset is not sim time, so the receive time is kept.
    wall_offset_us = 1_700_000_000_000_000
    assert ros_stamp_ns(1_500_000, wall_offset_us, receive) == receive


def test_px4_topic_prefers_a_version_suffix():
    names = ["/fmu/out/sensor_combined", "/fmu/out/sensor_combined_v1"]
    assert px4_topic(names, "sensor_combined") == "/fmu/out/sensor_combined_v1"
    assert px4_topic([], "vehicle_attitude") == "/fmu/out/vehicle_attitude"


def test_covariance_is_diagonal():
    cov = diagonal_covariance(0.1)
    assert cov[0] == cov[4] == cov[8]
    assert abs(cov[0] - 0.01) < 1e-12
    assert cov[1] == 0.0


def test_rgbd_static_tf_matches_the_sdf():
    x500 = REFERENCE_X500.read_text(encoding="utf-8")
    oak = (REPO_GZ / "models" / "OakD-Lite-rgbd" / "model.sdf").read_text(encoding="utf-8")
    transforms = camera_transforms(x500, oak, {})
    children = [item.child for item in transforms]
    assert len(children) == len(set(children))
    mount = _by_child(transforms, "OakD-Lite/base_link")
    assert mount.parent == "base_link"
    np.testing.assert_allclose(mount.xyz, (0.12, 0.03, 0.242))
    np.testing.assert_allclose(mount.rpy, (0.0, 0.0, 0.0))
    rgb = _by_child(transforms, "camera_rgb_frame")
    assert rgb.parent == "OakD-Lite/base_link"
    np.testing.assert_allclose(rgb.xyz, (0.01233, -0.03, 0.01878))
    np.testing.assert_allclose(rgb.rpy, (0.0, 0.3, 0.0))
    optical = _by_child(transforms, "camera_rgb_optical_frame")
    assert optical.parent == "camera_rgb_frame"
    np.testing.assert_allclose(optical.rpy, OPTICAL_RPY)
    imu = _by_child(transforms, "imu_link")
    np.testing.assert_allclose(imu.xyz, (0.01233, -0.03, 0.014))
    np.testing.assert_allclose(imu.rpy, (0.0, 0.0, 0.0))
    depth = _by_child(transforms, "depth_camera_frame")
    np.testing.assert_allclose(depth.rpy, (0.0, 0.3, 0.0))


def test_mount_env_overrides_the_sdf_pitch():
    x500 = REFERENCE_X500.read_text(encoding="utf-8")
    oak = (REPO_GZ / "models" / "OakD-Lite-rgbd" / "model.sdf").read_text(encoding="utf-8")
    transforms = camera_transforms(
        x500,
        oak,
        {"CAM_PITCH_DEG": "17", "CAM_X": "0.2"},
    )
    mount = _by_child(transforms, "OakD-Lite/base_link")
    np.testing.assert_allclose(mount.xyz, (0.2, 0.03, 0.242))
    np.testing.assert_allclose(mount.rpy[1], math.radians(17.0))
    rgb = _by_child(transforms, "camera_rgb_frame")
    np.testing.assert_allclose(rgb.rpy, (0.0, 0.3, 0.0))


def test_stereo_static_tf_uses_the_stereo_model():
    x500 = REFERENCE_X500.read_text(encoding="utf-8")
    oak = (REPO_GZ / "models" / "OakD-Lite-stereo" / "model.sdf").read_text(encoding="utf-8")
    transforms = camera_transforms(x500, oak, {})
    children = {item.child for item in transforms}
    assert "camera_rgb_frame" not in children
    left = _by_child(transforms, "stereo_left_camera_frame")
    right = _by_child(transforms, "stereo_right_camera_frame")
    np.testing.assert_allclose(left.xyz, (0.1233, 0.037, 0.013))
    np.testing.assert_allclose(right.xyz, (0.1233, -0.037, 0.013))
    np.testing.assert_allclose(left.rpy, (0.0, 0.3, 0.0))
    optical = _by_child(transforms, "stereo_left_camera_optical_frame")
    assert optical.parent == "stereo_left_camera_frame"
    assert _by_child(transforms, "imu_link").parent == "OakD-Lite/base_link"
