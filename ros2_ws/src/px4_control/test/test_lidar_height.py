"""Lidar-versus-EKF2 height guard, no ROS."""

import math

import pytest

from px4_control.lidar_height import (
    NO_DISTANCE_SENSOR_LOG,
    LidarHeightGuard,
    lidar_height_log,
    tilt_compensated_height,
)


def test_level_range_is_the_height():
    assert tilt_compensated_height(2.0, 1.0) == pytest.approx(2.0)


def test_tilt_uses_the_body_down_cosine():
    cosine = math.cos(0.2) * math.cos(-0.1)
    assert tilt_compensated_height(2.0, cosine) == pytest.approx(2.0 * cosine)


def test_upward_beam_is_not_a_height():
    assert tilt_compensated_height(2.0, -0.2) is None
    assert tilt_compensated_height(-1.0, 1.0) is None


def test_gap_must_last_more_than_the_duration():
    guard = LidarHeightGuard(0.3, 0.5)
    assert guard.observe(0.0, 2.5, 2.0) is None
    assert guard.observe(0.5, 2.5, 2.0) is None
    reason = guard.observe(0.51, 2.5, 2.0)
    assert reason is not None
    assert '0.500' in reason or '0.510' in reason
    assert 'EKF2 height above ground' in reason
    assert 'lidar height' in reason
    assert lidar_height_log(reason).startswith('lidar height abort:')
    assert guard.observe(1.0, 3.0, 2.0) is None


def test_a_sample_back_inside_the_tolerance_resets_the_timer():
    guard = LidarHeightGuard(0.3, 0.5)
    assert guard.observe(0.0, 2.5, 2.0) is None
    assert guard.observe(0.4, 2.1, 2.0) is None
    assert guard.observe(0.9, 2.5, 2.0) is None
    assert guard.observe(1.4, 2.5, 2.0) is None
    assert guard.observe(1.41, 2.5, 2.0) is not None


def test_no_distance_sensor_logs_once():
    guard = LidarHeightGuard()
    assert guard.absence(0.0, False) is None
    assert guard.absence(0.9, False) is None
    assert guard.absence(1.0, False) == NO_DISTANCE_SENSOR_LOG
    assert guard.absence(5.0, False) is None
    # A range that shows up later is still checked.
    assert guard.observe(5.0, 3.0, 2.0) is None
    assert guard.observe(5.6, 3.0, 2.0) is not None


def test_thresholds_reject_non_positive_values():
    with pytest.raises(ValueError):
        LidarHeightGuard(0.0, 0.5)
    with pytest.raises(ValueError):
        LidarHeightGuard(0.3, 0.0)
