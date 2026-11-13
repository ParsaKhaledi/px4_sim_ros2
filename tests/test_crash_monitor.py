import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "e2e"))

from crash_monitor import crash_reason, load_crash_thresholds, should_retry  # noqa: E402


def _obs(**overrides):
    base = {
        "phase": "hover",
        "tilt_deg": 2.0,
        "height_m": 2.0,
        "speed_mps": 0.1,
        "setpoint_error_m": 0.05,
        "odom_age_s": 0.1,
        "seen_odom": True,
        "armed": True,
        "was_armed": True,
        "failsafe": False,
        "landed": False,
    }
    base.update(overrides)
    return base


def test_healthy_hover_is_not_a_crash():
    assert crash_reason(_obs()) is None


def test_tilt_trips_once_airborne():
    assert crash_reason(_obs(tilt_deg=75)) == "tilt"


def test_tilt_on_the_ground_is_ignored():
    assert crash_reason(_obs(phase="preflight", tilt_deg=75)) is None


def test_low_altitude_and_impact_while_airborne():
    assert crash_reason(_obs(height_m=0.05)) == "low_altitude"
    assert crash_reason(_obs(height_m=0.4, speed_mps=4.0)) == "ground_impact"


def test_land_phase_may_be_low_and_disarmed():
    assert crash_reason(_obs(phase="land", height_m=0.02, landed=True, armed=False)) is None
    assert crash_reason(_obs(phase="disarm", armed=False, height_m=0.02, landed=True)) is None


def test_unexpected_disarm_failsafe_and_land():
    assert crash_reason(_obs(armed=False)) == "unexpected_disarm"
    assert crash_reason(_obs(failsafe=True)) == "failsafe"
    assert crash_reason(_obs(landed=True)) == "unexpected_land"


def test_setpoint_divergence_and_odom_timeout():
    assert crash_reason(_obs(setpoint_error_m=3.0)) == "setpoint_divergence"
    assert crash_reason(_obs(phase="takeoff", setpoint_error_m=3.0)) is None
    assert crash_reason(_obs(odom_age_s=9.0)) == "odometry_timeout"
    assert crash_reason(_obs(phase="preflight", seen_odom=False, odom_age_s=None)) is None


def test_thresholds_come_from_env():
    limits = load_crash_thresholds({"E2E_CRASH_TILT_DEG": "30", "E2E_MAX_RETRIES": "4"})
    assert limits.tilt_deg == 30
    assert limits.max_retries == 4
    assert crash_reason(_obs(tilt_deg=40), limits) == "tilt"


def test_retry_budget():
    assert should_retry(2, attempt=1, max_retries=2)
    assert should_retry(137, attempt=2, max_retries=2)
    assert not should_retry(2, attempt=3, max_retries=2)
    assert not should_retry(1, attempt=1, max_retries=2)
    assert not should_retry(0, attempt=1, max_retries=2)
