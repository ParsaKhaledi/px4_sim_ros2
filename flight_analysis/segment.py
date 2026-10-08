"""Split a flight into legs wherever the commanded setpoint changes."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from flight_analysis.frames import wrap_pi
from flight_analysis.log import Track


# Setpoint rate above this is a new command, not hover jitter. A 0.30 m
# leg still counts even when the command slides over a couple of seconds.
SETPOINT_SPEED_M_S = 0.05
YAW_RATE_RAD_S = math.radians(5.0)
# A steady gap shorter than this is still part of the same move.
MIN_HOLD_S = 0.4
# Labels only. Pass/fail bands come from the E2E limits, not from these.
HORIZONTAL_LEG_M = 0.05
VERTICAL_LEG_M = 0.15
YAW_LEG_RAD = math.radians(20.0)
HOVER_MIN_S = 2.0
AIRBORNE_UP_M = 0.5


@dataclass(frozen=True)
class Leg:
    """One commanded target, from the start of the change through the hold."""

    index: int
    kind: str
    t_start_s: float
    t_arrived_s: float
    t_end_s: float
    start_north_m: float
    start_east_m: float
    start_down_m: float
    target_north_m: float
    target_east_m: float
    target_down_m: float
    start_yaw_rad: float
    target_yaw_rad: float


def segment_command(command: Track) -> list[Leg]:
    """Return one leg per commanded target.

    A leg starts when ``trajectory_setpoint`` (or the local-position setpoint)
    moves faster than a small deadband, and it ends at the next such change.
    The steady samples after the move are part of that leg: that is the hold
    the vehicle was told to fly.
    """

    count = int(command.time_s.shape[0])
    if count == 0:
        return []
    if count == 1:
        return [_make_leg(command, 0, 0, 0, 0, None)]

    changing = _changing_mask(command)
    runs = _merge_short_holds(_runs(changing), command.time_s)
    spans = _leg_spans(runs)
    legs: list[Leg] = []
    previous_target: tuple[float, float, float, float] | None = None
    for start, arrived, end in spans:
        leg = _make_leg(command, len(legs), start, arrived, end, previous_target)
        legs.append(leg)
        previous_target = (
            leg.target_north_m,
            leg.target_east_m,
            leg.target_down_m,
            leg.target_yaw_rad,
        )
    return legs


def _changing_mask(command: Track) -> np.ndarray:
    """True on samples that begin a fast change from the previous sample."""

    count = int(command.time_s.shape[0])
    changing = np.zeros(count, dtype=bool)
    for index in range(1, count):
        dt = float(command.time_s[index] - command.time_s[index - 1])
        if dt <= 0.0:
            continue
        dn = float(command.north_m[index] - command.north_m[index - 1])
        de = float(command.east_m[index] - command.east_m[index - 1])
        dd = float(command.down_m[index] - command.down_m[index - 1])
        dyaw = abs(wrap_pi(float(command.yaw_rad[index] - command.yaw_rad[index - 1])))
        speed = math.sqrt(dn * dn + de * de + dd * dd) / dt
        if speed >= SETPOINT_SPEED_M_S or dyaw / dt >= YAW_RATE_RAD_S:
            changing[index] = True
    return changing


def _runs(changing: np.ndarray) -> list[tuple[int, int, bool]]:
    """Collapse consecutive samples into ``(start, end, is_changing)`` runs."""

    runs: list[tuple[int, int, bool]] = []
    start = 0
    for index in range(1, len(changing) + 1):
        if index == len(changing) or bool(changing[index]) != bool(changing[start]):
            runs.append((start, index - 1, bool(changing[start])))
            start = index
    return runs


def _merge_short_holds(
    runs: list[tuple[int, int, bool]],
    time_s: np.ndarray,
) -> list[tuple[int, int, bool]]:
    """Treat a brief pause inside a slide as part of the same move."""

    if not runs:
        return []
    merged: list[tuple[int, int, bool]] = [runs[0]]
    for start, end, is_changing in runs[1:]:
        prev_start, prev_end, prev_changing = merged[-1]
        duration = float(time_s[end] - time_s[start])
        if not is_changing and duration < MIN_HOLD_S and prev_changing:
            merged[-1] = (prev_start, end, True)
            continue
        if is_changing and prev_changing:
            merged[-1] = (prev_start, end, True)
            continue
        merged.append((start, end, is_changing))
    return merged


def _leg_spans(runs: list[tuple[int, int, bool]]) -> list[tuple[int, int, int]]:
    """Pair each move with the hold that follows it.

    The opening steady command, before anything moves, is its own leg.
    Each span is ``(start_index, arrived_index, end_index)``.
    """

    spans: list[tuple[int, int, int]] = []
    index = 0
    while index < len(runs):
        start, end, is_changing = runs[index]
        if not is_changing:
            spans.append((start, start, end))
            index += 1
            continue
        if index + 1 < len(runs) and not runs[index + 1][2]:
            hold_start, hold_end, _hold = runs[index + 1]
            spans.append((start, hold_start, hold_end))
            index += 2
            continue
        spans.append((start, end, end))
        index += 1
    return spans


def _make_leg(
    command: Track,
    index: int,
    start: int,
    arrived: int,
    end: int,
    previous: tuple[float, float, float, float] | None,
) -> Leg:
    """Build one leg. The target is the median command on the steady part."""

    steady = slice(arrived, end + 1)
    target_n = float(np.median(command.north_m[steady]))
    target_e = float(np.median(command.east_m[steady]))
    target_d = float(np.median(command.down_m[steady]))
    target_yaw = float(np.median(command.yaw_rad[steady]))
    arrived = _arrival_index(
        command, start, end, target_n, target_e, target_d, target_yaw
    )
    if previous is None:
        start_n, start_e, start_d, start_yaw = target_n, target_e, target_d, target_yaw
    else:
        start_n, start_e, start_d, start_yaw = previous
    duration = float(command.time_s[end] - command.time_s[start])
    kind = _kind(
        target_n - start_n,
        target_e - start_e,
        target_d - start_d,
        wrap_pi(target_yaw - start_yaw),
        -target_d,
        duration,
    )
    return Leg(
        index=index,
        kind=kind,
        t_start_s=float(command.time_s[start]),
        t_arrived_s=float(command.time_s[arrived]),
        t_end_s=float(command.time_s[end]),
        start_north_m=start_n,
        start_east_m=start_e,
        start_down_m=start_d,
        target_north_m=target_n,
        target_east_m=target_e,
        target_down_m=target_d,
        start_yaw_rad=start_yaw,
        target_yaw_rad=target_yaw,
    )


def _arrival_index(
    command: Track,
    start: int,
    end: int,
    target_n: float,
    target_e: float,
    target_d: float,
    target_yaw: float,
) -> int:
    """First sample whose command is already on the leg target.

    A step is there on the first new sample. A ramp is there only once the
    slide has finished, so the settle clock does not start early.
    """

    for index in range(start, end + 1):
        dn = float(command.north_m[index] - target_n)
        de = float(command.east_m[index] - target_e)
        dd = float(command.down_m[index] - target_d)
        dyaw = abs(wrap_pi(float(command.yaw_rad[index] - target_yaw)))
        if math.sqrt(dn * dn + de * de + dd * dd) <= 0.02 and dyaw <= math.radians(2.0):
            return index
    return end


def _kind(
    dn: float,
    de: float,
    dd: float,
    dyaw: float,
    target_up_m: float,
    duration_s: float,
) -> str:
    """Name the leg from what the command asked for, in NED.

    Climb is a negative change in down. A long steady command above half a
    metre is a hover; a short one is only a preflight hold.
    """

    horizontal = math.hypot(dn, de)
    if horizontal < HORIZONTAL_LEG_M and abs(dd) < VERTICAL_LEG_M and abs(dyaw) < YAW_LEG_RAD:
        if target_up_m >= AIRBORNE_UP_M and duration_s >= HOVER_MIN_S:
            return "hover"
        return "hold"
    if abs(dyaw) >= YAW_LEG_RAD and horizontal < HORIZONTAL_LEG_M * 1.6:
        return "yaw"
    if abs(dd) >= VERTICAL_LEG_M and horizontal < HORIZONTAL_LEG_M * 1.6:
        return "takeoff" if dd < 0.0 else "land"
    if horizontal >= HORIZONTAL_LEG_M:
        return "translate"
    return "hover" if duration_s >= HOVER_MIN_S else "hold"
