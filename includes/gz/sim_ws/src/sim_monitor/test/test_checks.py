"""Preflight helper tests."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim_monitor.preflight_check import PoseOrigins, camera_info_topic, default_camera_topic
from sim_monitor.spawn_frame import DEFAULT_POSE, spawn_ground_pose
from sim_monitor.checks import (
    RateTracker,
    check_min,
    check_rate,
    default_min_rtf,
    expected_sensor_hz,
    gated_rate_hz,
    distance_sensor_result,
    distance_sensor_topic,
    ground_truth_leak_from_info,
    lidar_down_enabled,
    gz_topic_publishers,
    match_versioned_topic,
    minimum_rate_hz,
    parse_spawn_pose,
    px4_vision_covariance_topic,
    px4_vision_model_name,
    quat_from_rpy,
    relative_position_error,
    summarize,
    topic_rate_line,
    with_rtf_hint,
)
from sim_monitor.preflight_check import check_ground_truth_vision_leak
from sim_monitor.real_time_factor import iter_real_time_factors, parse_stats_message, windowed_rtf


def test_spawn_pose_default():
    x, y, z, roll, pitch, yaw = parse_spawn_pose(DEFAULT_POSE)
    assert (x, y, z) == (-3.0, -1.6, 0.15)
    assert yaw == 3.14
    ground = spawn_ground_pose((x, y, z, roll, pitch, yaw))
    assert ground[2] == 0.0
    assert ground[:2] == (-3.0, -1.6)
    assert ground[5] == 3.14
    assert ground[3] == 0.0 and ground[4] == 0.0
    qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)
    assert abs(qx * qx + qy * qy + qz * qz + qw * qw - 1.0) < 1e-9


def test_pose_error_uses_ground_truth_when_rtabmap_starts():
    origins = PoseOrigins()
    origins.ground_truth((0.0, 0.0, 0.15))
    origins.ground_truth((0.0, 0.0, 0.044))
    origins.rtab((0.0, 0.0, 0.0))
    error = relative_position_error(
        origins.rtab_position, origins.rtab_origin, origins.gt_position, origins.gt_origin,
    )
    assert abs(error) < 1e-9
    assert origins.gt_origin == (0.0, 0.0, 0.044)

    origins.ground_truth((1.0, 0.0, 0.044))
    origins.rtab((1.0, 0.0, 0.0))
    origins.rtab((0.0, 0.0, 0.0))
    error = relative_position_error(
        origins.rtab_position, origins.rtab_origin, origins.gt_position, origins.gt_origin,
    )
    assert abs(error) < 1e-9
    assert origins.gt_origin == (1.0, 0.0, 0.044)

    origins.lost(True)
    origins.ground_truth((2.0, 0.0, 0.05))
    origins.rtab((0.0, 0.0, 0.0))
    assert origins.gt_origin == (2.0, 0.0, 0.05)
    assert origins.rtab_origin == (0.0, 0.0, 0.0)


def test_stereo_camera_topic_replaces_the_rgb_default():
    assert default_camera_topic({}) == "/camera/rgb/image_raw"
    assert default_camera_topic({"CameraType": "rgbd"}) == "/camera/rgb/image_raw"
    assert default_camera_topic({"CameraType": "stereo"}) == "/camera/stereo/left/image_raw"
    assert default_camera_topic({
        "CameraType": "stereo",
        "PREFLIGHT_CAMERA_TOPIC": "/camera/rgb/image_raw",
    }) == "/camera/stereo/left/image_raw"
    assert default_camera_topic({
        "CameraType": "stereo",
        "PREFLIGHT_CAMERA_TOPIC": "/custom/image",
    }) == "/custom/image"


def test_relative_pose_error_at_rest():
    error = relative_position_error((1.0, 2.0, 0.05), (1.0, 2.0, 0.0), (-3.0, -1.6, 0.02), (-3.0, -1.6, 0.0))
    assert abs(error - 0.03) < 1e-9


def test_versioned_topic_prefers_highest():
    names = [
        "/fmu/out/vehicle_status",
        "/fmu/out/vehicle_status_v1",
        "/fmu/out/vehicle_status_v2",
        "/fmu/out/estimator_status_flags",
    ]
    assert match_versioned_topic(names, "vehicle_status") == "/fmu/out/vehicle_status_v2"
    assert match_versioned_topic(names, "estimator_status_flags") == "/fmu/out/estimator_status_flags"


def test_summarize_requires_every_check():
    ok, message = summarize([(True, "PASS a"), (False, "FAIL b")])
    assert ok is False
    assert message == "PASS a\nFAIL b"
    ok, message = summarize([(True, "PASS a"), (True, "PASS b")])
    assert ok is True
    ok, message = summarize([(True, "PASS a"), (False, "SKIP gz CLI is absent")])
    assert ok is True
    assert message.splitlines()[1].startswith("SKIP")


def test_ground_truth_vision_topic_publisher_parse():
    quiet = """Publishers [Address, Message Type]:
  No publishers

Subscribers [Address, Message Type]:
  tcp://172.17.0.2:45941, gz.msgs.OdometryWithCovariance
"""
    leaked = """Publishers [Address, Message Type]:
  tcp://172.17.0.2:40001, gz.msgs.OdometryWithCovariance
  tcp://172.17.0.2:40002, gz.msgs.OdometryWithCovariance

Subscribers [Address, Message Type]:
  tcp://172.17.0.2:45941, gz.msgs.OdometryWithCovariance
"""
    assert gz_topic_publishers(quiet) == []
    assert gz_topic_publishers(leaked) == [
        "tcp://172.17.0.2:40001, gz.msgs.OdometryWithCovariance",
        "tcp://172.17.0.2:40002, gz.msgs.OdometryWithCovariance",
    ]
    assert px4_vision_model_name({}) == "x500_depth_0"
    assert px4_vision_model_name({"PX4_SIM_MODEL": "gz_x500_depth"}) == "x500_depth_0"
    assert px4_vision_model_name({"PX4_GZ_MODEL": "x500_depth", "PX4_INSTANCE": "1"}) == "x500_depth_1"
    assert px4_vision_model_name({"PX4_GZ_MODEL": "x500_depth_0"}) == "x500_depth_0"
    assert px4_vision_covariance_topic("x500_depth_0") == "/model/x500_depth_0/odometry_with_covariance"

    ok, text = ground_truth_leak_from_info("", gz_cli=False, model_name="x500_depth_0")
    assert ok is True
    assert text.startswith("SKIP no ground-truth leak to PX4 vision")
    assert "PASS" not in text

    ok, text = ground_truth_leak_from_info(quiet, gz_cli=True, model_name="x500_depth_0")
    assert ok is True
    assert text.startswith("PASS no ground-truth leak to PX4 vision")
    assert "/model/x500_depth_0/odometry_with_covariance has no publisher" in text

    ok, text = ground_truth_leak_from_info(leaked, gz_cli=True, model_name="x500_depth_0")
    assert ok is False
    assert text.startswith("FAIL no ground-truth leak to PX4 vision")
    assert "has a publisher tcp://172.17.0.2:40001" in text
    assert "EKF2" in text


def test_distance_sensor_check_reports_range_or_skips_when_unpublished():
    assert lidar_down_enabled({}) is True
    assert lidar_down_enabled({"LIDAR_DOWN": "0"}) is False
    assert distance_sensor_result(enabled=False, topic=None, distance_m=None) is None
    names = ["/fmu/in/distance_sensor", "/fmu/out/vehicle_status", "/fmu/out/distance_sensor_v1"]
    assert distance_sensor_topic(["/fmu/in/distance_sensor"]) is None
    assert distance_sensor_topic(names) == "/fmu/out/distance_sensor_v1"
    ok, text = distance_sensor_result(enabled=True, topic=None, distance_m=None)
    assert ok is True
    assert text.startswith("SKIP distance_sensor")
    assert "/fmu/out/distance_sensor" in text
    assert "dds_topics.yaml" in text
    ok, text = distance_sensor_result(enabled=True, topic="/fmu/out/distance_sensor", distance_m=0.2)
    assert ok is True
    assert text.startswith("PASS distance_sensor: 0.200 m within 0.1-0.5 m")
    ok, text = distance_sensor_result(enabled=True, topic="/fmu/out/distance_sensor", distance_m=0.1)
    assert ok is True
    ok, text = distance_sensor_result(enabled=True, topic="/fmu/out/distance_sensor", distance_m=0.5)
    assert ok is True
    ok, text = distance_sensor_result(enabled=True, topic="/fmu/out/distance_sensor", distance_m=0.05)
    assert ok is False
    assert "0.050 m outside 0.1-0.5 m" in text
    ok, text = distance_sensor_result(enabled=True, topic="/fmu/out/distance_sensor", distance_m=None)
    assert ok is False
    assert "no message yet" in text


def test_ground_truth_leak_check_skips_without_gz(monkeypatch):
    monkeypatch.setattr("sim_monitor.preflight_check.shutil.which", lambda _name: None)
    ok, text = check_ground_truth_vision_leak({})
    assert ok is True
    assert text.startswith("SKIP no ground-truth leak to PX4 vision")
    assert "PASS" not in text


def test_rate_tracker():
    tracker = RateTracker(2.0)
    for stamp in (0.0, 0.1, 0.2, 0.3):
        tracker.add(stamp)
    assert abs(tracker.hz(0.3) - 10.0) < 1e-6


def test_rtf_parser():
    text = "sim_time {\n  sec: 2\n  nsec: 500000000\n}\nreal_time_factor: 0.03\niterations: 10\n"
    assert iter_real_time_factors(text) == 0.03
    assert iter_real_time_factors("no factor here") is None
    sim_time, gz_rtf = parse_stats_message(text)
    assert gz_rtf == 0.03
    assert abs(sim_time - 2.5) < 1e-9


def test_windowed_rtf_ignores_the_gz_field():
    # 1.0 s of sim time across 5.0 s of wall time is 0.2, whatever gz printed.
    samples = [(0.0, 0.0), (5.0, 1.0)]
    assert abs(windowed_rtf(samples, 5.0) - 0.2) < 1e-9
    longer = [(0.0, 0.0), (2.0, 0.4), (6.0, 1.0), (10.0, 2.0)]
    # Window keeps the samples whose wall age is <= 5 s: (6, 1) and (10, 2).
    assert abs(windowed_rtf(longer, 5.0) - 0.25) < 1e-9
    assert windowed_rtf([(1.0, 1.0)], 5.0) is None


def test_stopped_topic_is_stale_against_the_clock():
    tracker = RateTracker(2.0)
    for index in range(11):
        stamp = index * 0.05
        tracker.add(stamp, stamp)
    # Samples run at 20 Hz through t=0.5. Ending the window on the last
    # sample still reports ~20 Hz and would pass a 10 Hz gate.
    assert tracker.hz_sim(tracker.sim_times[-1]) > 10.0
    ok, text = topic_rate_line("/imu", tracker, 2.0, 2.0, 10.0, 1.0)
    assert ok is False
    assert "stale: last message 1.500 s sim ago" in text
    live = RateTracker(2.0)
    for index in range(21):
        stamp = index * 0.05
        live.add(stamp, stamp)
    ok, text = topic_rate_line("/imu", live, 1.0, 1.0, 10.0, 1.0)
    assert ok is True
    assert "stale" not in text


def test_camera_info_topic_replaces_the_image():
    assert camera_info_topic("/camera/rgb/image_raw", {}) == "/camera/rgb/camera_info"
    assert camera_info_topic("/camera/stereo/left/image_raw", {}) == "/camera/stereo/left/camera_info"
    assert camera_info_topic(
        "/camera/rgb/image_raw",
        {"PREFLIGHT_CAMERA_INFO_TOPIC": "/custom/camera_info"},
    ) == "/custom/camera_info"


def test_sim_rate_and_wall_fallback():
    tracker = RateTracker(2.0)
    for index in range(5):
        tracker.add(wall_s := index * 0.5, sim_s := index * 0.1)
    assert abs(tracker.hz_wall(2.0) - 2.0) < 1e-6
    assert abs(tracker.hz_sim(0.4) - 10.0) < 1e-6
    assert abs(gated_rate_hz(None, 2.0, 0.2) - 10.0) < 1e-9
    ok, text = check_rate("/ground_truth/odom", 50.0, 10.4, 20.0)
    assert ok is True
    assert "/ground_truth/odom: 50.0 Hz sim (10.4 Hz wall)" in text


def test_rtf_failure_hints_unless_vision_profile_is_cpu():
    ok, text = check_min("real_time_factor", 0.125, 0.15, "")
    assert ok is False
    hinted = with_rtf_hint(ok, text, {})
    assert hinted == (
        "FAIL real_time_factor: 0.125  >= 0.150; "
        "on CPU-only machines set VISION_PROFILE=cpu "
        "(measured ~0.45 vs 0.125 for full on apt_world)"
    )
    assert "\n" not in hinted
    assert with_rtf_hint(ok, text, {"VISION_PROFILE": "full"}) == hinted
    assert with_rtf_hint(ok, text, {"VISION_PROFILE": "cpu"}) == text
    passed, pass_text = check_min("real_time_factor", 0.50, 0.15, "")
    assert passed is True
    assert with_rtf_hint(passed, pass_text, {}) == pass_text


def test_preflight_thresholds_follow_profile_and_overrides():
    assert expected_sensor_hz("camera", {}) == 30.0
    assert expected_sensor_hz("imu", {"VISION_PROFILE": "cpu"}) == 100.0
    assert expected_sensor_hz("camera", {"VISION_PROFILE": "cpu"}) == 10.0
    assert expected_sensor_hz("camera", {"CAM_RATE_HZ": "15"}) == 15.0
    assert minimum_rate_hz("camera", {"VISION_PROFILE": "full"}) == 15.0
    assert minimum_rate_hz("imu", {"VISION_PROFILE": "cpu", "PREFLIGHT_MIN_IMU_HZ": "80"}) == 80.0
    assert minimum_rate_hz("imu", {"IMU_SOURCE": "px4"}) == 80.0
    assert minimum_rate_hz("imu", {"IMU_SOURCE": "px4", "PREFLIGHT_MIN_IMU_HZ": "60"}) == 60.0
    assert minimum_rate_hz("imu", {"IMU_SOURCE": "oak", "IMU_RATE_HZ": "80"}) == 40.0
    assert default_min_rtf({"HEADLESS_SOFTWARE": "1"}) == 0.15
    assert default_min_rtf({}) == 0.8
    assert default_min_rtf({"HEADLESS_SOFTWARE": "1", "PREFLIGHT_MIN_RTF": "0.5"}) == 0.5
