"""Lidar-versus-EKF2 height guard, no ROS."""

import math

import pytest

from px4_control.lidar_height import (
    NO_DISTANCE_SENSOR_LOG,
    LidarHeightGuard,
    height_above_floor,
    lidar_height_log,
    tilt_compensated_height,
)


def _feed(guard, time_s, range_m, ekf, cosine=1.0, min_d=0.2, max_d=10.0, quality=1):
    return guard.observe(time_s, range_m, cosine, ekf, min_d, max_d, quality)


def test_level_range_is_the_height():
    assert tilt_compensated_height(2.0, 1.0) == pytest.approx(2.0)


def test_tilt_uses_the_body_down_cosine():
    cosine = math.cos(0.2) * math.cos(-0.1)
    assert tilt_compensated_height(2.0, cosine) == pytest.approx(2.0 * cosine)


def test_mount_offset_is_subtracted_after_tilt_compensation():
    cosine = math.cos(0.2)
    assert height_above_floor(2.19, cosine, 0.19) == pytest.approx(2.19 * cosine - 0.19)


def test_upward_beam_is_not_a_height():
    assert tilt_compensated_height(2.0, -0.2) is None
    assert height_above_floor(2.0, -0.2, 0.19) is None
    assert tilt_compensated_height(-1.0, 1.0) is None


def test_on_the_ground_below_min_range_does_not_abort():
    guard = LidarHeightGuard()
    for time_s in (0.0, 0.3, 0.7, 1.2):
        assert _feed(guard, time_s, 0.15, ekf=5.0, min_d=0.2, max_d=10.0) is None
    assert guard.armed is False
    assert guard.fired is False


def test_valid_agreement_does_not_abort():
    guard = LidarHeightGuard()
    cosine = math.cos(0.15)
    # Sensor 0.19 m above the reference, reference 2.0 m above the floor.
    range_m = (2.0 + 0.19) / cosine
    for time_s in (0.0, 0.3, 0.7, 1.2):
        assert _feed(guard, time_s, range_m, ekf=2.0, cosine=cosine) is None
    assert guard.armed is True
    assert guard.fired is False


def test_four_tenths_disagreement_for_six_tenths_aborts():
    guard = LidarHeightGuard()
    range_m = 2.0 + 0.19
    assert _feed(guard, 0.0, range_m, ekf=2.4) is None
    assert guard.armed is True
    assert _feed(guard, 0.5, range_m, ekf=2.4) is None
    reason = _feed(guard, 0.6, range_m, ekf=2.4)
    assert reason is not None
    assert '0.400 m' in reason
    assert '0.600 s' in reason
    assert 'mount offset 0.190 m' in reason
    assert 'EKF2 height above floor' in reason
    assert lidar_height_log(reason).startswith('lidar height abort:')
    assert _feed(guard, 1.2, range_m, ekf=4.0) is None


def test_out_of_range_readings_are_ignored():
    guard = LidarHeightGuard()
    for time_s in (0.0, 0.6, 1.2):
        assert _feed(guard, time_s, 12.0, ekf=0.0, max_d=8.0) is None
        assert _feed(guard, time_s, 2.5, ekf=0.0, quality=0) is None
    assert guard.armed is False
    assert guard.fired is False
    # A quality-0 gap clears an open breach, so it does not finish the 0.5 s.
    range_m = 2.0 + 0.19
    assert _feed(guard, 2.0, range_m, ekf=2.4) is None
    assert _feed(guard, 2.4, range_m, ekf=2.4, quality=0) is None
    assert _feed(guard, 2.8, range_m, ekf=2.4) is None
    assert guard.fired is False


def test_gap_must_last_more_than_the_duration():
    guard = LidarHeightGuard(0.3, 0.5)
    range_m = 2.5 + 0.19
    assert _feed(guard, 0.0, range_m, 2.0) is None
    assert _feed(guard, 0.5, range_m, 2.0) is None
    reason = _feed(guard, 0.51, range_m, 2.0)
    assert reason is not None
    assert '0.510 s' in reason
    assert guard.observe(1.0, range_m, 1.0, 2.0, 0.2, 10.0, 1) is None


def test_a_sample_back_inside_the_tolerance_resets_the_timer():
    guard = LidarHeightGuard(0.3, 0.5)
    assert _feed(guard, 0.0, 2.5 + 0.19, 2.0) is None
    assert _feed(guard, 0.4, 2.1 + 0.19, 2.0) is None
    assert _feed(guard, 0.9, 2.5 + 0.19, 2.0) is None
    assert _feed(guard, 1.4, 2.5 + 0.19, 2.0) is None
    assert _feed(guard, 1.41, 2.5 + 0.19, 2.0) is not None


def test_no_distance_sensor_logs_once():
    guard = LidarHeightGuard()
    assert guard.absence(0.0, False) is None
    assert guard.absence(0.9, False) is None
    assert guard.absence(1.0, False) == NO_DISTANCE_SENSOR_LOG
    assert guard.absence(5.0, False) is None
    assert _feed(guard, 5.0, 3.0 + 0.19, 2.0) is None
    assert _feed(guard, 5.6, 3.0 + 0.19, 2.0) is not None


def test_thresholds_reject_non_positive_values():
    with pytest.raises(ValueError):
        LidarHeightGuard(0.0, 0.5)
    with pytest.raises(ValueError):
        LidarHeightGuard(0.3, 0.0)
    with pytest.raises(ValueError):
        LidarHeightGuard(mount_offset_m=-0.1)
