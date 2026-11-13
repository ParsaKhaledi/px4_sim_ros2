"""Crash rules for one flight sample.

The monitor is pure data in, reason out. The ROS node fills the observation
dict. A reason means the attempt is over and the sim should be restarted.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

AIRBORNE_PHASES = {"hover", "leg1", "yaw", "leg2"}
GROUND_PHASES = {"start", "preflight", "land", "disarm"}
RETRY_EXIT_CODES = {2, 134, 137, 139}


@dataclass
class CrashThresholds:
    tilt_deg: float = 60.0
    min_height_m: float = 0.20
    impact_speed_mps: float = 2.5
    divergence_m: float = 1.5
    odom_timeout_s: float = 4.0
    max_retries: int = 2


def load_crash_thresholds(env=None) -> CrashThresholds:
    source = os.environ if env is None else env

    def number(name: str, default: float) -> float:
        raw = source.get(name)
        if raw is None or raw == "":
            return default
        return float(raw)

    retries = number("E2E_MAX_RETRIES", 2)
    return CrashThresholds(
        tilt_deg=number("E2E_CRASH_TILT_DEG", 60.0),
        min_height_m=number("E2E_CRASH_MIN_HEIGHT_M", 0.20),
        impact_speed_mps=number("E2E_CRASH_IMPACT_SPEED_MPS", 2.5),
        divergence_m=number("E2E_CRASH_DIVERGENCE_M", 1.5),
        odom_timeout_s=number("E2E_ODOM_TIMEOUT_S", 4.0),
        max_retries=int(retries),
    )


def crash_reason(obs: dict, thresholds: CrashThresholds | None = None) -> str | None:
    """Return a short reason, or None when the sample looks healthy.

    `obs` keys: phase, tilt_deg, height_m, speed_mps, setpoint_error_m,
    odom_age_s, armed, was_armed, failsafe, landed, seen_odom.
    Missing keys are treated as "not enough data", not as a crash.
    """

    thresholds = thresholds or load_crash_thresholds()
    phase = str(obs.get("phase") or "")
    airborne = phase in AIRBORNE_PHASES

    tilt = obs.get("tilt_deg")
    if tilt is not None and phase not in {"start", "preflight"}:
        if float(tilt) > thresholds.tilt_deg:
            return "tilt"

    if obs.get("seen_odom") and phase not in {"start", "preflight"}:
        age = obs.get("odom_age_s")
        if age is not None and float(age) > thresholds.odom_timeout_s:
            return "odometry_timeout"

    if obs.get("failsafe") and phase not in {"land", "disarm"}:
        return "failsafe"

    if (
        obs.get("was_armed")
        and obs.get("armed") is False
        and phase not in {"land", "disarm"}
    ):
        return "unexpected_disarm"

    if airborne and obs.get("landed"):
        return "unexpected_land"

    height = obs.get("height_m")
    speed = obs.get("speed_mps")
    if airborne and height is not None and float(height) < thresholds.min_height_m:
        return "low_altitude"
    near_ground = (
        height is not None and float(height) < thresholds.min_height_m * 3.0
    )
    if airborne and near_ground and speed is not None:
        if float(speed) > thresholds.impact_speed_mps:
            return "ground_impact"

    if phase in AIRBORNE_PHASES:
        error = obs.get("setpoint_error_m")
        if error is not None and float(error) > thresholds.divergence_m:
            return "setpoint_divergence"
    return None


def should_retry(exit_code: int, attempt: int, max_retries: int) -> bool:
    """True when this finished attempt should be followed by a sim restart.

    `attempt` is 1-based. `max_retries` is how many restarts are allowed
    after the first try, so the last try is attempt 1 + max_retries.
    """

    if exit_code not in RETRY_EXIT_CODES:
        return False
    return attempt <= max_retries
