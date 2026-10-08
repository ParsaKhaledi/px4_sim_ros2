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


def prearm_block_reason(status: object | None, flags: object | None) -> str | None:
    """Why PX4 should not be armed, or ``None`` when the commander looks ready.

    Informational failsafe bits are included only when the pre-flight checks
    have not passed or the commander is already in failsafe. A healthy SITL
    vehicle can report ``manual_control_signal_lost`` and still be armable.
    """
    if status is None:
        return 'no vehicle_status received'
    checks_pass = bool(getattr(status, 'pre_flight_checks_pass', False))
    in_failsafe = bool(getattr(status, 'failsafe', False))
    if checks_pass and not in_failsafe:
        return None
    reasons = failsafe_reasons(flags)
    if not checks_pass:
        reasons.append('pre_flight_checks_pass is false')
    if in_failsafe:
        reasons.append('vehicle_status.failsafe is true')
    return '; '.join(reasons) if reasons else 'pre_flight_checks_pass is false'


def ack_failure_text(result: int | None) -> str | None:
    """Human-readable command ack, or ``None`` while the command can still succeed."""
    if result is None or result in (ACK_ACCEPTED, ACK_IN_PROGRESS):
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
