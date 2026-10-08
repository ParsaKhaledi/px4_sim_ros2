"""IMU frame conversion, stamps, and the absence of camera static TFs."""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

REPO_GZ = Path(__file__).resolve().parents[4]
PACKAGE = Path(__file__).resolve().parents[1]

from sim_monitor.checks import expected_imu_hz, minimum_rate_hz
from sim_monitor.imu_frames import (
    flu_enu_quaternion,
    frd_to_flu,
    gravity_quaternion_xyzw,
    quat_from_rpy,
    quat_to_rot,
    yaw_from_quat,
)
from sim_monitor.imu_source import (
    PX4_OFFSET,
    RECEIVE,
    bridge_gazebo_imu,
    configured_imu_publishers,
    default_tf_pairs,
    imu_stamp_ns,
    input_rate_hz,
    measure_px4_offset_ns,
    normalize_imu_source,
    optical_frame,
    orientation_covariance,
    px4_sample_us,
    rate_log,
)
from sim_monitor.px4_imu_relay import px4_topic

G = 9.80665
# Frames this stack must not publish. Ground-truth TF (base_link_gt) is separate.
CAMERA_FRAMES = (
    "camera_link",
    "imu_link",
    "optical_frame",
    "OakD-Lite",
    "camera_rgb_frame",
)


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


def test_gravity_quaternion_keeps_roll_pitch_and_drops_yaw():
    level = flu_enu_quaternion((1.0, 0.0, 0.0, 0.0))
    assert abs(yaw_from_quat(level) - math.pi / 2.0) < 1e-9
    gravity = gravity_quaternion_xyzw((1.0, 0.0, 0.0, 0.0))
    w = gravity[3]
    assert abs(yaw_from_quat((w, gravity[0], gravity[1], gravity[2]))) < 1e-9
    rotation = quat_to_rot((w, gravity[0], gravity[1], gravity[2]))
    np.testing.assert_allclose(rotation[:, 2], [0.0, 0.0, 1.0], atol=1e-9)
    covariance = orientation_covariance(0.02)
    assert covariance[8] == 1.0e3
    assert abs(covariance[0] - 0.0004) < 1e-12
    assert covariance[4] == covariance[0]


def test_only_one_imu_publisher_per_mode():
    assert configured_imu_publishers("oak") == ("ros_gz_bridge",)
    assert configured_imu_publishers("px4") == ("px4_imu_relay",)
    assert bridge_gazebo_imu("oak") is True
    assert bridge_gazebo_imu("px4") is False
    assert bridge_gazebo_imu("") is True
    assert normalize_imu_source(None) == "oak"
    sim = (REPO_GZ / "config_gz_bridge_sim.yaml").read_text(encoding="utf-8")
    imu = (REPO_GZ / "config_gz_bridge_imu.yaml").read_text(encoding="utf-8")
    assert 'ros_topic_name: "/imu"' not in sim
    assert imu.count('ros_topic_name: "/imu"') == 1


def test_px4_rate_minimum_and_optical_frame():
    assert expected_imu_hz({"IMU_SOURCE": "oak", "IMU_RATE_HZ": "200"}) == 200.0
    assert minimum_rate_hz("imu", {"IMU_SOURCE": "px4"}) == 80.0
    assert minimum_rate_hz("imu", {"IMU_SOURCE": "px4", "PREFLIGHT_MIN_IMU_HZ": "60"}) == 60.0
    assert "camera_rgb_frame" in default_tf_pairs({"CameraType": "rgbd"})
    assert "stereo_left_camera_frame" in default_tf_pairs({"CameraType": "stereo"})
    assert default_tf_pairs({"PREFLIGHT_TF_PAIRS": "world:spawn"}) == "world:spawn"
    assert optical_frame({"CameraType": "rgbd"}) == "camera_rgb_optical_frame"
    assert optical_frame({"CameraType": "stereo"}) == "stereo_left_camera_optical_frame"
    assert optical_frame({"PREFLIGHT_OPTICAL_FRAME": "custom"}) == "custom"


def test_receive_stamp_is_the_default_and_raw_boot_time_is_not_published():
    receive = 5_000_000_000
    sample_us = 1_500_000
    assert imu_stamp_ns(RECEIVE, receive, sample_us, offset_ns=0) == receive
    assert imu_stamp_ns("", receive, sample_us, offset_ns=None) == receive
    # A raw boot timestamp is a different epoch from /clock.
    assert sample_us * 1000 != receive
    offset = measure_px4_offset_ns(sample_us, receive)
    assert imu_stamp_ns(PX4_OFFSET, receive, sample_us, offset) == receive
    later_us = sample_us + 20_000
    later_receive = receive + 50_000_000
    assert imu_stamp_ns(PX4_OFFSET, later_receive, later_us, offset) == later_us * 1000 + offset
    assert px4_sample_us(10, None) == 10
    assert px4_sample_us(10, 7) == 7


def test_sensor_combined_rate_is_sim_time_and_warns_below_80():
    start = 1_000_000_000
    fast = [start + i * 10_000_000 for i in range(11)]
    assert abs(input_rate_hz(fast) - 100.0) < 1e-6
    slow = [start + i * 20_000_000 for i in range(6)]
    assert abs(input_rate_hz(slow) - 50.0) < 1e-6
    assert rate_log(80.0) == ("info", "sensor_combined 80.0 Hz sim")
    level, text = rate_log(50.0)
    assert level == "warn"
    assert "below 80 Hz" in text
    level, text = rate_log(70.0, warn_below=60.0)
    assert level == "info"
    assert rate_log(None) is None


def test_px4_topic_prefers_a_version_suffix():
    names = ["/fmu/out/sensor_combined", "/fmu/out/sensor_combined_v1"]
    assert px4_topic(names, "sensor_combined") == "/fmu/out/sensor_combined_v1"
    assert px4_topic([], "vehicle_attitude") == "/fmu/out/vehicle_attitude"


def test_no_camera_static_transforms_are_published():
    urdf = (REPO_GZ / "x500_tf_publisher" / "x500_urdf.urdf").read_text(encoding="utf-8")
    helpers = (REPO_GZ / "startFiles" / "gz_start_sim_helpers.sh").read_text(encoding="utf-8")
    assert "camera_tf" not in helpers
    for name in ("camera_link", "imu_link", "optical_frame", "OakD-Lite"):
        assert name not in urdf
    assert not (PACKAGE / "sim_monitor" / "camera_tf.py").exists()
    assert not (PACKAGE / "sim_monitor" / "camera_extrinsics.py").exists()
    for path in (PACKAGE / "sim_monitor").glob("*.py"):
        text = path.read_text(encoding="utf-8")
        if "sendTransform" not in text and "StaticTransformBroadcaster" not in text:
            continue
        for name in CAMERA_FRAMES:
            assert name not in text
