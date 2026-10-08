"""Pre-arm checks and the text returned when arming is refused.

``/sim/preflight_check`` is optional. The caller treats a missing service as
a warning and continues. A failed check returns that service's message
unchanged. PX4 flag names are returned when the commander is not ready.
"""

from __future__ import annotations

from dataclasses import dataclass

# VehicleCommandAck result values from px4_msgs v1.17.
ACK_ACCEPTED = 0
ACK_TEMPORARILY_REJECTED = 1
ACK_DENIED = 2
ACK_UNSUPPORTED = 3
ACK_FAILED = 4
ACK_IN_PROGRESS = 5
ACK_CANCELLED = 6

_ACK_NAMES = {
    ACK_ACCEPTED: 'ACCEPTED',
    ACK_TEMPORARILY_REJECTED: 'TEMPORARILY_REJECTED',
    ACK_DENIED: 'DENIED',
    ACK_UNSUPPORTED: 'UNSUPPORTED',
    ACK_FAILED: 'FAILED',
    ACK_IN_PROGRESS: 'IN_PROGRESS',
    ACK_CANCELLED: 'CANCELLED',
}

# Boolean FailsafeFlags that mean "not healthy" when true. Mode-requirement
# bitmasks are integers and are not listed here.
_FAILSAFE_BOOLS = (
    'angular_velocity_invalid',
    'attitude_invalid',
    'local_altitude_invalid',
    'local_position_invalid',
    'local_position_invalid_relaxed',
    'local_velocity_invalid',
    'global_position_invalid',
    'global_position_invalid_relaxed',
    'auto_mission_missing',
    'offboard_control_signal_lost',
    'home_position_invalid',
    'manual_control_signal_lost',
    'gcs_connection_lost',
    'battery_low_remaining_time',
    'battery_unhealthy',
    'geofence_breached',
    'mission_failure',
    'vtol_fixed_wing_system_failure',
    'wind_limit_exceeded',
    'flight_time_limit_exceeded',
    'position_accuracy_low',
    'navigator_failure',
    'fd_critical_failure',
    'fd_esc_arming_failure',
    'fd_imbalanced_prop',
    'fd_motor_failure',
)


@dataclass(frozen=True)
class ArmDecision:
    success: bool
    message: str


def failsafe_reasons(flags: object | None) -> list[str]:
    """Names of failsafe booleans that are currently set."""
    if flags is None:
        return []
    reasons: list[str] = []
    for name in _FAILSAFE_BOOLS:
        if bool(getattr(flags, name, False)):
            reasons.append(name)
    battery = getattr(flags, 'battery_warning', 0)
    try:
        battery_value = int(battery)
    except (TypeError, ValueError):
        battery_value = 0
    if battery_value > 0:
        reasons.append(f'battery_warning={battery_value}')
    return reasons


def _magnetometer_arming_text(estimator: object | None) -> str | None:
    """Spell out a missing or failed compass. ``None`` when the estimator is quiet."""
    if estimator is None:
        return None
    if bool(getattr(estimator, 'cs_mag_fault', False)):
        return (
            'Magnetometer failed (estimator cs_mag_fault). The compass is '
            'unhealthy, so arming is denied while a magnetometer is required.'
        )
    bad = [
        name
        for name in ('fs_bad_mag_x', 'fs_bad_mag_y', 'fs_bad_mag_z', 'fs_bad_mag_decl')
        if bool(getattr(estimator, name, False))
    ]
    if bad:
        return (
            'Magnetometer fusion failed (' + ', '.join(bad) + '). Arming is '
            'denied while the compass measurement is unusable.'
        )
    fused = any(
        bool(getattr(estimator, name, False))
        for name in ('cs_mag_hdg', 'cs_mag_3d', 'cs_mag')
    )
    yaw_from_vision = bool(getattr(estimator, 'cs_ev_yaw', False))
    yaw_aligned = bool(getattr(estimator, 'cs_yaw_align', False))
    if fused or yaw_from_vision or yaw_aligned:
        return None
    return (
        'Magnetometer missing. No compass fusion is active and yaw is not '
        'aligned. PX4 denies arming when the world has no magnetometer plugin '
        '(Preflight Fail: Compass Sensor missing, No valid data from Compass, '
        'or Found 0 compass).'
    )


def prearm_block_reason(
    status: object | None,
    flags: object | None,
    estimator: object | None = None,
) -> str | None:
    """Why PX4 should not be armed, or ``None`` when the commander looks ready.

    Informational failsafe bits are included only when the pre-flight checks
    have not passed or the commander is already in failsafe. A healthy SITL
    vehicle can report ``manual_control_signal_lost`` and still be armable.
    ``gcs_connection_lost`` is spelled out: with the airframe default
    ``NAV_DLL_ACT`` of 2, headless SITL cannot arm until a GCS heartbeat
    arrives. The sim profile sets ``NAV_DLL_ACT`` to 0.
    """
    if status is None:
        return 'no vehicle_status received'
    checks_pass = bool(getattr(status, 'pre_flight_checks_pass', False))
    in_failsafe = bool(getattr(status, 'failsafe', False))
    if checks_pass and not in_failsafe:
        return None
    reasons = failsafe_reasons(flags)
    if 'gcs_connection_lost' in reasons:
        reasons = [item for item in reasons if item != 'gcs_connection_lost']
        reasons.append(
            'No connection to the GCS. NAV_DLL_ACT defaults to 2 and then '
            'blocks arming without a QGroundControl heartbeat. The sim '
            'profile sets NAV_DLL_ACT 0'
        )
    mag = _magnetometer_arming_text(estimator)
    if mag:
        reasons.append(mag)
    if not checks_pass:
        reasons.append('pre_flight_checks_pass is false')
    if in_failsafe:
        reasons.append('vehicle_status.failsafe is true')
    return '; '.join(reasons) if reasons else 'pre_flight_checks_pass is false'


def ack_failure_text(result: int | None) -> str | None:
    """Human-readable command ack, or ``None`` while the command can still succeed.

    ``TEMPORARILY_REJECTED`` is a retry, not a final answer. PX4 uses it
    when disarm arrives before the land detector has latched.
    """
    if result is None or result in (ACK_ACCEPTED, ACK_IN_PROGRESS, ACK_TEMPORARILY_REJECTED):
        return None
    name = _ACK_NAMES.get(int(result), 'UNKNOWN')
    return f'vehicle_command_ack result {int(result)} ({name})'


def combine_failure(*parts: str | None) -> str:
    text = '; '.join(part for part in parts if part)
    return text or 'arming failed'


def sim_preflight_decision(available: bool, type_ok: bool, success: bool | None, message: str) -> ArmDecision | None:
    """Decide what to do with ``/sim/preflight_check``.

    ``None`` means the check was skipped and PX4 checks should continue.
    A returned decision with ``success`` false must be returned to the caller
    with ``message`` unchanged.
    """
    if not available:
        return None
    if not type_ok:
        return None
    if success:
        return None
    return ArmDecision(False, message)
