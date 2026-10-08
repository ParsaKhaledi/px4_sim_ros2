"""Preflight helper tests."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim_monitor.checks import (
    RateTracker,
    match_versioned_topic,
    parse_spawn_pose,
    quat_from_rpy,
    relative_position_error,
    summarize,
)
from sim_monitor.real_time_factor import iter_real_time_factors


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
    text = "sim_time {\n  sec: 2\n}\nreal_time_factor: 0.83\niterations: 10\n"
    assert iter_real_time_factors(text) == 0.83
    assert iter_real_time_factors("no factor here") is None
