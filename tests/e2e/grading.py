"""Grade an out-and-back flight against ground-truth samples.

Samples are dicts with keys x, y, z, yaw (degrees, ENU) and phase.
Phases used here: start, hover, leg1, yaw, leg2, land.
The last start sample is the ground origin, so a pose taken while the
model is still being spawned does not set the hover height.
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
    leg_tolerance_m: float = 0.05
    yaw_tolerance_deg: float = 5.0
    yaw_settle_deg: float = 3.0
    return_tolerance_m: float = 0.05
    height_tolerance_m: float = 0.10


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
        leg_tolerance_m=number("E2E_LEG_TOLERANCE_M", 0.05),
        yaw_tolerance_deg=number("E2E_YAW_TOLERANCE_DEG", 5.0),
        yaw_settle_deg=number("E2E_YAW_SETTLE_DEG", 3.0),
        return_tolerance_m=number("E2E_RETURN_TOLERANCE_M", 0.05),
        height_tolerance_m=number("E2E_HEIGHT_TOLERANCE_M", 0.10),
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


def grade_mission(samples, thresholds: Thresholds | None = None) -> dict:
    thresholds = thresholds or load_thresholds()
    checks = []
    start = _phase(samples, "start")
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
    hover_z = hover[0]["z"]
    height_error = abs(hover_pose["z"] - origin[2] - thresholds.takeoff_height_m)
    checks.append(
        _check(
            "height",
            height_error <= thresholds.height_tolerance_m,
            round(height_error, 4),
            thresholds.height_tolerance_m,
        )
    )

    drift = max(horizontal((row["x"], row["y"]), (hover[0]["x"], hover[0]["y"])) for row in hover)
    checks.append(
        _check(
            "hover_drift",
            drift <= thresholds.hover_drift_m,
            round(drift, 4),
            thresholds.hover_drift_m,
        )
    )

    if leg1:
        distance = horizontal(
            (leg1[-1]["x"], leg1[-1]["y"]),
            (hover_pose["x"], hover_pose["y"]),
        )
        leg_error = abs(distance - thresholds.leg_length_m)
        checks.append(
            _check(
                "leg1",
                leg_error <= thresholds.leg_tolerance_m,
                round(distance, 4),
                thresholds.leg_length_m,
                f"error {leg_error:.4f} m",
            )
        )
    else:
        checks.append(_check("leg1", False, None, thresholds.leg_length_m, "no leg1 samples"))

    if yaw_rows:
        target = hover_pose["yaw"] + 180.0
        final = yaw_rows[-1]["yaw"]
        yaw_error = abs(wrap_deg(final - target))
        settle_band = max(
            abs(wrap_deg(row["yaw"] - final)) for row in yaw_rows[-5:]
        )
        checks.append(
            _check(
                "yaw",
                yaw_error <= thresholds.yaw_tolerance_deg
                and settle_band <= thresholds.yaw_settle_deg,
                round(yaw_error, 3),
                thresholds.yaw_tolerance_deg,
                f"settle_band {settle_band:.3f} deg",
            )
        )
    else:
        checks.append(_check("yaw", False, None, thresholds.yaw_tolerance_deg, "no yaw samples"))

    if leg2 and yaw_rows:
        distance = horizontal(
            (leg2[-1]["x"], leg2[-1]["y"]),
            (yaw_rows[-1]["x"], yaw_rows[-1]["y"]),
        )
        leg_error = abs(distance - thresholds.leg_length_m)
        checks.append(
            _check(
                "leg2",
                leg_error <= thresholds.leg_tolerance_m,
                round(distance, 4),
                thresholds.leg_length_m,
                f"error {leg_error:.4f} m",
            )
        )
    else:
        checks.append(_check("leg2", False, None, thresholds.leg_length_m, "no leg2 samples"))

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
