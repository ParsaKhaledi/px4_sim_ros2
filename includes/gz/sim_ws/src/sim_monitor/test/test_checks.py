"""Preflight helper tests."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim_monitor.checks import (
    RateTracker,
    check_rate,
    default_min_rtf,
    expected_sensor_hz,
    gated_rate_hz,
    match_versioned_topic,
    minimum_rate_hz,
    parse_spawn_pose,
    quat_from_rpy,
    relative_position_error,
    summarize,
)
from sim_monitor.real_time_factor import iter_real_time_factors, parse_stats_message, windowed_rtf


def test_spawn_pose_default():
    x, y, z, roll, pitch, yaw = parse_spawn_pose("-3,-1.6,0,0,0,3.14")
    assert (x, y, z) == (-3.0, -1.6, 0.0)
    assert yaw == 3.14
    qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)
    assert abs(qx * qx + qy * qy + qz * qz + qw * qw - 1.0) < 1e-9


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


def test_preflight_thresholds_follow_profile_and_overrides():
    assert expected_sensor_hz("camera", {}) == 30.0
    assert expected_sensor_hz("imu", {"VISION_PROFILE": "cpu"}) == 100.0
    assert expected_sensor_hz("camera", {"VISION_PROFILE": "cpu"}) == 10.0
    assert expected_sensor_hz("camera", {"CAM_RATE_HZ": "15"}) == 15.0
    assert minimum_rate_hz("camera", {"VISION_PROFILE": "full"}) == 15.0
    assert minimum_rate_hz("imu", {"VISION_PROFILE": "cpu", "PREFLIGHT_MIN_IMU_HZ": "80"}) == 80.0
    assert default_min_rtf({"HEADLESS_SOFTWARE": "1"}) == 0.15
    assert default_min_rtf({}) == 0.8
    assert default_min_rtf({"HEADLESS_SOFTWARE": "1", "PREFLIGHT_MIN_RTF": "0.5"}) == 0.5
