"""Grade an out-and-back flight against ground-truth samples.

Samples are dicts with keys x, y, z, yaw (degrees, ENU), t (sim seconds)
and phase. Phases used here: start, takeoff, hover, leg1, yaw, leg2, land.
The last start sample is the ground origin, so a pose taken while the
model is still being spawned does not set the hover height.

A step passes settle only when the track itself stays inside the band
for the hold, and that hold finishes before the timeout. The driver
waiting is not enough.
"""

from __future__ import annotations

import math
import os
from dataclasses import asdict, dataclass


@dataclass
class Thresholds:
    takeoff_height_m: float = 2.0
    hover_s: float = 10.0
    leg_length_m: float = 0.3
    hover_drift_m: float = 0.05
    leg_tolerance_m: float = 0.03
    yaw_tolerance_deg: float = 5.0
    yaw_settle_deg: float = 3.0
    return_tolerance_m: float = 0.05
    height_tolerance_m: float = 0.10
    overshoot_m: float = 0.03
    settle_tolerance_m: float = 0.02
    settle_hold_s: float = 1.0
    settle_timeout_s: float = 4.0
    hover_height_band_m: float = 0.05
    hover_height_hold_s: float = 2.0


def load_thresholds(env=None) -> Thresholds:
    source = os.environ if env is None else env

    def number(name: str, default: float) -> float:
        raw = source.get(name)
        if raw is None or raw == "":
            return default
        return float(raw)

    return Thresholds(
        takeoff_height_m=number("E2E_TAKEOFF_HEIGHT_M", 2.0),
        hover_s=number("E2E_HOVER_S", 10.0),
        leg_length_m=number("E2E_LEG_LENGTH_M", 0.3),
        hover_drift_m=number("E2E_HOVER_DRIFT_M", 0.05),
        leg_tolerance_m=number("E2E_LEG_TOLERANCE_M", 0.03),
        yaw_tolerance_deg=number("E2E_YAW_TOLERANCE_DEG", 5.0),
        yaw_settle_deg=number("E2E_YAW_SETTLE_DEG", 3.0),
        return_tolerance_m=number("E2E_RETURN_TOLERANCE_M", 0.05),
        height_tolerance_m=number("E2E_HEIGHT_TOLERANCE_M", 0.10),
        overshoot_m=number("E2E_OVERSHOOT_M", 0.03),
        settle_tolerance_m=number("E2E_SETTLE_TOLERANCE_M", 0.02),
        settle_hold_s=number("E2E_SETTLE_HOLD_S", 1.0),
        settle_timeout_s=number("E2E_SETTLE_TIMEOUT_S", 4.0),
        hover_height_band_m=number("E2E_HOVER_HEIGHT_BAND_M", 0.05),
        hover_height_hold_s=number("E2E_HOVER_HEIGHT_HOLD_S", 2.0),
    )


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny, cosy))


def wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def horizontal(left, right) -> float:
    return math.hypot(left[0] - right[0], left[1] - right[1])


def _phase(samples, name):
    rows = [row for row in samples if row.get("phase") == name]
    return rows


def _check(name, passed, measured, limit, detail=""):
    return {
        "name": name,
        "passed": bool(passed),
        "measured": measured,
        "limit": limit,
        "detail": detail,
    }


def _timed(rows) -> bool:
    return bool(rows) and all(row.get("t") is not None for row in rows)


def _unwrap(angles: list[float]) -> list[float]:
    if not angles:
        return []
    out = [angles[0]]
    for angle in angles[1:]:
        out.append(out[-1] + wrap_deg(angle - out[-1]))
    return out


def _settle_time(rows, t0: float, error_at, tolerance: float, hold_s: float, timeout_s: float):
    """Sim time from t0 until a hold finishes inside the band, or None.

    The hold has to end by t0 + timeout_s. Every sample in the window has
    to be inside the band, and the samples have to cover that window.
    """

    if not _timed(rows):
        return None
    ordered = sorted(rows, key=lambda row: float(row["t"]))
    deadline = t0 + timeout_s
    for row in ordered:
        start = float(row["t"])
        end = start + hold_s
        if end > deadline + 1e-9:
            break
        window = [item for item in ordered if start - 1e-9 <= float(item["t"]) <= end + 1e-9]
        if not window:
            continue
        if float(window[-1]["t"]) + 1e-9 < end:
            continue
        if all(error_at(item) <= tolerance for item in window):
            return end - t0
    return None


def _leg_axis(yaw_deg: float):
    """Unit step in Gazebo ENU. Heading 0 points north (+y)."""

    heading = math.radians(yaw_deg)
    return math.sin(heading), math.cos(heading)


def _along(row, start_xy, axis) -> float:
    return (row["x"] - start_xy[0]) * axis[0] + (row["y"] - start_xy[1]) * axis[1]


def _grade_hover_clock(checks, takeoff, hover, origin_z: float, thresholds: Thresholds) -> None:
    """The 10 s hover starts only after height has held the band."""

    target_z = origin_z + thresholds.takeoff_height_m
    band = thresholds.hover_height_band_m
    hold = thresholds.hover_height_hold_s
    ready = False
    detail = "height was not held before the hover clock"
    if _timed(takeoff) and _timed(hover):
        hover_t0 = float(hover[0]["t"])
        window = [row for row in takeoff if float(row["t"]) >= hover_t0 - hold - 1e-9]
        if window:
            span = float(window[-1]["t"]) - float(window[0]["t"])
            in_band = all(abs(row["z"] - target_z) <= band for row in window)
            covers = span + 0.25 >= hold and float(window[-1]["t"]) >= hover_t0 - 0.25
            ready = in_band and covers
            detail = f"held {span:.2f} s before hover"
            if not in_band:
                detail = "height left the band before the hover clock"
    checks.append(_check("hover_ready", ready, round(hold if ready else 0.0, 4), hold, detail))

    duration = 0.0
    if _timed(hover) and len(hover) >= 2:
        duration = float(hover[-1]["t"]) - float(hover[0]["t"])
    checks.append(
        _check(
            "hover_duration",
            duration + 0.25 >= thresholds.hover_s,
            round(duration, 4),
            thresholds.hover_s,
        )
    )


def _grade_leg(checks, name: str, rows, start_pose, thresholds: Thresholds) -> None:
    if not rows or start_pose is None:
        checks.append(_check(name, False, None, thresholds.leg_length_m, "no samples"))
        checks.append(_check(f"{name}_overshoot", False, None, thresholds.overshoot_m, "no samples"))
        checks.append(_check(f"{name}_settle", False, None, thresholds.settle_timeout_s, "no samples"))
        return

    start_xy = (start_pose["x"], start_pose["y"])
    axis = _leg_axis(start_pose["yaw"])
    target = (
        start_xy[0] + thresholds.leg_length_m * axis[0],
        start_xy[1] + thresholds.leg_length_m * axis[1],
        start_pose["z"],
    )
    distance = horizontal((rows[-1]["x"], rows[-1]["y"]), start_xy)
    leg_error = abs(distance - thresholds.leg_length_m)
    checks.append(
        _check(
            name,
            leg_error <= thresholds.leg_tolerance_m,
            round(distance, 4),
            thresholds.leg_length_m,
            f"error {leg_error:.4f} m",
        )
    )

    peak = max(_along(row, start_xy, axis) for row in rows)
    overshoot = max(0.0, peak - thresholds.leg_length_m)
    checks.append(
        _check(
            f"{name}_overshoot",
            overshoot <= thresholds.overshoot_m,
            round(overshoot, 4),
            thresholds.overshoot_m,
        )
    )

    def position_error(row) -> float:
        return math.dist((row["x"], row["y"], row["z"]), target)

    t0 = float(rows[0]["t"]) if rows[0].get("t") is not None else None
    settle = None if t0 is None else _settle_time(
        rows,
        t0,
        position_error,
        thresholds.settle_tolerance_m,
        thresholds.settle_hold_s,
        thresholds.settle_timeout_s,
    )
    checks.append(
        _check(
            f"{name}_settle",
            settle is not None and settle <= thresholds.settle_timeout_s,
            None if settle is None else round(settle, 4),
            thresholds.settle_timeout_s,
            "inside ±"
            f"{thresholds.settle_tolerance_m:.2f} m for {thresholds.settle_hold_s:.0f} s",
        )
    )


def _grade_yaw(checks, rows, start_pose, thresholds: Thresholds) -> None:
    if not rows or start_pose is None:
        checks.append(_check("yaw_overshoot", False, None, thresholds.yaw_tolerance_deg, "no yaw samples"))
        checks.append(_check("yaw_settle", False, None, thresholds.settle_timeout_s, "no yaw samples"))
        return

    unwrapped = _unwrap([start_pose["yaw"]] + [row["yaw"] for row in rows])
    target = unwrapped[0] + 180.0
    overshoot = max(0.0, max(unwrapped[1:]) - target)
    checks.append(
        _check(
            "yaw_overshoot",
            overshoot <= thresholds.yaw_tolerance_deg,
            round(overshoot, 4),
            thresholds.yaw_tolerance_deg,
        )
    )

    target_yaw = wrap_deg(start_pose["yaw"] + 180.0)

    def yaw_error(row) -> float:
        return abs(wrap_deg(row["yaw"] - target_yaw))

    t0 = float(rows[0]["t"]) if rows[0].get("t") is not None else None
    settle = None if t0 is None else _settle_time(
        rows,
        t0,
        yaw_error,
        thresholds.yaw_settle_deg,
        thresholds.settle_hold_s,
        thresholds.settle_timeout_s,
    )
    checks.append(
        _check(
            "yaw_settle",
            settle is not None and settle <= thresholds.settle_timeout_s,
            None if settle is None else round(settle, 4),
            thresholds.settle_timeout_s,
            f"held ±{thresholds.yaw_settle_deg:.0f} deg for {thresholds.settle_hold_s:.0f} s",
        )
    )


def grade_mission(samples, thresholds: Thresholds | None = None) -> dict:
    thresholds = thresholds or load_thresholds()
    checks = []
    start = _phase(samples, "start")
    takeoff = _phase(samples, "takeoff")
    hover = _phase(samples, "hover")
    leg1 = _phase(samples, "leg1")
    yaw_rows = _phase(samples, "yaw")
    leg2 = _phase(samples, "leg2")
    land = _phase(samples, "land")

    if not start or not hover:
        checks.append(_check("samples", False, 0, 1, "need start and hover samples"))
        return {"passed": False, "checks": checks, "thresholds": asdict(thresholds)}

    origin = (start[-1]["x"], start[-1]["y"], start[-1]["z"])
    hover_pose = hover[-1]
    height_error = abs(hover_pose["z"] - origin[2] - thresholds.takeoff_height_m)
    checks.append(
        _check(
            "height",
            height_error <= thresholds.height_tolerance_m,
            round(height_error, 4),
            thresholds.height_tolerance_m,
        )
    )
    _grade_hover_clock(checks, takeoff, hover, origin[2], thresholds)

    drift = max(horizontal((row["x"], row["y"]), (hover[0]["x"], hover[0]["y"])) for row in hover)
    checks.append(
        _check(
            "hover_drift",
            drift <= thresholds.hover_drift_m,
            round(drift, 4),
            thresholds.hover_drift_m,
        )
    )

    _grade_leg(checks, "leg1", leg1, hover_pose, thresholds)
    _grade_yaw(checks, yaw_rows, hover_pose, thresholds)
    yaw_pose = yaw_rows[-1] if yaw_rows else None
    _grade_leg(checks, "leg2", leg2, yaw_pose, thresholds)

    if land:
        back = horizontal((land[-1]["x"], land[-1]["y"]), (origin[0], origin[1]))
        checks.append(
            _check(
                "return_to_start",
                back <= thresholds.return_tolerance_m,
                round(back, 4),
                thresholds.return_tolerance_m,
            )
        )
    else:
        checks.append(
            _check("return_to_start", False, None, thresholds.return_tolerance_m, "no land samples")
        )

    return {
        "passed": all(item["passed"] for item in checks),
        "checks": checks,
        "thresholds": asdict(thresholds),
    }
