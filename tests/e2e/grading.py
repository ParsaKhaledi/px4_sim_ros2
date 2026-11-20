"""Grade a flight against ground-truth samples.

Samples are dicts with keys x, y, z, yaw (degrees, ENU), t (sim seconds)
and phase. Phases used here: start, takeoff, hover, leg1, yaw, leg2, land.
Height is base_link above the on-ground pre-arm sample. ``z`` on the track
is the Gazebo model origin (``/ground_truth/odom``; ``base_link_gt`` is not
a link), and ``x500_base`` places base_link 0.24 m above that origin. The
offset is added to both the sample and the pre-arm pose, so a level climb
is unchanged. The land phase ends while the vehicle is still descending;
the landing height is the settled pose (the disarm sample when there is one).

grade_hover scores the headless hover. grade_mission also scores the
out-and-back legs. The flight level uses grade_hover.

A step passes settle only when the track itself stays inside the band
for the hold, and that hold finishes before the timeout. The driver
waiting is not enough.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass
from pathlib import Path


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


# x500_base model pose is ``0 0 0.24``. base_link, the baro, the IMU and the
# GPS sit on that link with no further pose. /ground_truth/odom asks for
# base_link_gt, which the SDF does not define, so the recorded z is the
# model origin.
BASE_LINK_ABOVE_MODEL_ORIGIN_M = 0.24
_PREARM_PHASES = {"preflight", "start", "offboard"}


def _prearm_sample(samples):
    """Last on-ground sample before arm. Spawn motion is not this pose."""

    rows = [row for row in samples if row.get("phase") in _PREARM_PHASES]
    if rows:
        return rows[-1]
    start = _phase(samples, "start")
    return start[-1] if start else None


def _base_link_z(row) -> float:
    """Model-origin z lifted to base_link. Level flight: +0.24 m."""

    return float(row["z"]) + BASE_LINK_ABOVE_MODEL_ORIGIN_M


def _above_prearm(row, prearm) -> float:
    """base_link height relative to this track's own pre-arm base_link."""

    return _base_link_z(row) - _base_link_z(prearm)


def _ekf_above_prearm(row, prearm):
    """EKF local down, zeroed on this track's pre-arm EKF sample.

    ``px4_d`` is NED down of the EKF position. Its origin is the baro
    reference at init (EKF2_HGT_REF=0), so the raw value is not a height
    above the ground. Up is the negation of the change from pre-arm.
    """

    if row is None or prearm is None:
        return None
    if row.get("px4_d") is None or prearm.get("px4_d") is None:
        return None
    return -(float(row["px4_d"]) - float(prearm["px4_d"]))


def _settled_landing(samples):
    """Pose after the vehicle has reached the ground, not the land-phase gate."""

    disarm = _phase(samples, "disarm")
    if disarm:
        return disarm[-1]
    land = _phase(samples, "land")
    return land[-1] if land else None


def flight_height_reference(samples) -> dict | None:
    """Every height as base_link above that source's pre-arm value."""

    prearm = _prearm_sample(samples)
    if prearm is None:
        return None
    hover = _phase(samples, "hover")
    land = _phase(samples, "land")
    settled = _settled_landing(samples)
    hover_row = hover[-1] if hover else None
    land_row = land[-1] if land else None

    def gt(row):
        if row is None:
            return None
        return round(_above_prearm(row, prearm), 4)

    def ekf(row):
        value = _ekf_above_prearm(row, prearm)
        return None if value is None else round(value, 4)

    return {
        "frame": "base_link",
        "zero": "each source's own on-ground pre-arm value",
        "base_link_above_model_origin_m": BASE_LINK_ABOVE_MODEL_ORIGIN_M,
        "prearm_phase": prearm.get("phase"),
        "ground_truth": {
            "point": (
                "Gazebo model origin from /ground_truth/odom. "
                "robot_base_frame base_link_gt is not a link in the x500 SDF, "
                "so the pose is the model origin, 0.24 m below base_link."
            ),
            "prearm_model_origin_z_m": round(float(prearm["z"]), 4),
            "prearm_base_link_z_m": round(_base_link_z(prearm), 4),
            "hover_above_prearm_m": gt(hover_row),
            "landing_phase_exit_above_prearm_m": gt(land_row),
            "landing_settled_above_prearm_m": gt(settled),
        },
        "ekf": {
            "point": (
                "vehicle_local_position.z, NED down. The IMU and baro are on "
                "base_link with no extra pose, so this is the EKF's base_link "
                "estimate. The origin is the baro height at initialization, "
                "not the model origin; a resting value near -0.24 m only means "
                "that run's origin landed near the model origin."
            ),
            "prearm_down_m": None if prearm.get("px4_d") is None else round(float(prearm["px4_d"]), 4),
            "hover_above_prearm_m": ekf(hover_row),
            "landing_settled_above_prearm_m": ekf(settled),
        },
    }


def _leg_axis(yaw_deg: float):
    """Unit step in Gazebo ENU. Yaw 0 points east (+x), yaw 90 points north (+y)."""

    heading = math.radians(yaw_deg)
    return math.cos(heading), math.sin(heading)


def _along(row, start_xy, axis) -> float:
    return (row["x"] - start_xy[0]) * axis[0] + (row["y"] - start_xy[1]) * axis[1]


def _grade_hover_clock(checks, takeoff, hover, prearm, thresholds: Thresholds) -> None:
    """The 10 s hover starts only after base_link height has held the band."""

    target = thresholds.takeoff_height_m
    band = thresholds.hover_height_band_m
    hold = thresholds.hover_height_hold_s
    ready = False
    detail = "height was not held before the hover clock"
    if prearm is not None and _timed(takeoff) and _timed(hover):
        hover_t0 = float(hover[0]["t"])
        window = [row for row in takeoff if float(row["t"]) >= hover_t0 - hold - 1e-9]
        if window:
            span = float(window[-1]["t"]) - float(window[0]["t"])
            errors = [abs(_above_prearm(row, prearm) - target) for row in window]
            in_band = all(error <= band for error in errors)
            covers = span + 0.25 >= hold and float(window[-1]["t"]) >= hover_t0 - 0.25
            ready = in_band and covers
            detail = f"held {span:.2f} s before hover, max error {max(errors):.4f} m"
            if not in_band:
                detail = f"height left the band before the hover clock, max error {max(errors):.4f} m"
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
            f"{thresholds.settle_tolerance_m:.2f} m for {_fmt_seconds(thresholds.settle_hold_s)}",
        )
    )


def _fmt_seconds(value: float) -> str:
    """Format a hold duration without rounding 0.5 s down to \"0 s\"."""
    if abs(float(value) - round(float(value))) < 1e-9:
        return f"{float(value):.0f} s"
    return f"{float(value):.1f} s"


def yaw_turn_report(samples) -> dict:
    """Spawn yaw, takeoff yaw, and the turn relative to takeoff.

    The final error is the wrap-safe distance of that change from 180 deg.
    Absolute heading is recorded so a log can be read, and it is not the
    quantity the final_yaw check compares to the limit. A +180 deg change
    and a -180 deg change are the same heading.
    """

    start = _phase(samples, "start")
    takeoff = _phase(samples, "takeoff")
    yaw_rows = _phase(samples, "yaw")
    spawn = float(start[0]["yaw"]) if start else None
    if takeoff:
        takeoff_yaw = float(takeoff[-1]["yaw"])
    elif start:
        takeoff_yaw = float(start[-1]["yaw"])
    else:
        takeoff_yaw = None
    final = float(yaw_rows[-1]["yaw"]) if yaw_rows else None
    change = None
    peak = None
    error = None
    if takeoff_yaw is not None and yaw_rows:
        series = _unwrap([takeoff_yaw] + [float(row["yaw"]) for row in yaw_rows])
        deltas = [value - series[0] for value in series[1:]]
        change = deltas[-1]
        peak = min(deltas)
        error = abs(wrap_deg(change - 180.0))

    def _round(value):
        return None if value is None else round(float(value), 4)

    return {
        "spawn_yaw_deg": _round(spawn),
        "takeoff_yaw_deg": _round(takeoff_yaw),
        "final_yaw_deg": _round(final),
        "yaw_change_deg": _round(change),
        "peak_change_deg": _round(peak),
        "yaw_error_deg": _round(error),
        "reference": "yaw change from takeoff versus a 180 deg turn, wrap-safe",
    }


def _grade_final_yaw(checks, samples, thresholds: Thresholds) -> dict:
    report = yaw_turn_report(samples)
    error = report["yaw_error_deg"]
    limit = thresholds.yaw_tolerance_deg
    checks.append(
        _check(
            "final_yaw",
            error is not None and error <= limit,
            error,
            limit,
            (
                f"spawn {report['spawn_yaw_deg']} deg, "
                f"takeoff {report['takeoff_yaw_deg']} deg, "
                f"change {report['yaw_change_deg']} deg, "
                f"error vs 180 deg {report['yaw_error_deg']} deg "
                "(change from takeoff, not absolute heading)"
            ),
        )
    )
    return report


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
            f"held ±{thresholds.yaw_settle_deg:.0f} deg for {_fmt_seconds(thresholds.settle_hold_s)}",
        )
    )


_OUT_AND_BACK = {"leg1", "yaw", "leg2"}


def _result(checks, thresholds: Thresholds) -> dict:
    return {
        "passed": bool(checks) and all(item["passed"] for item in checks),
        "checks": checks,
        "thresholds": asdict(thresholds),
    }


def _samples_from_payload(payload) -> list:
    if isinstance(payload, list):
        return payload
    if not isinstance(payload, dict):
        raise ValueError("hover track must be a list of samples or a JSON object")
    for key in ("track", "samples"):
        rows = payload.get(key)
        if isinstance(rows, list):
            return rows
    attempts = payload.get("attempts")
    if isinstance(attempts, list):
        for attempt in reversed(attempts):
            if not isinstance(attempt, dict):
                continue
            for key in ("track", "samples"):
                rows = attempt.get(key)
                if isinstance(rows, list):
                    return rows
    raise ValueError("hover track JSON has no samples")


def load_hover_track(env=None) -> list:
    """Read the one headless hover. This does not start the simulator."""

    source = os.environ if env is None else env
    path = source.get("E2E_HOVER_JSON") or source.get("E2E_RESULT_PATH") or ""
    if not path:
        flight_dir = source.get("E2E_FLIGHT_DIR") or ""
        if flight_dir:
            candidate = Path(flight_dir) / "trajectory.json"
            if candidate.is_file():
                path = str(candidate)
    if not path:
        raise ValueError(
            "No hover track. Set E2E_HOVER_JSON to the headless hover samples. "
            "This level does not start the simulator."
        )
    file = Path(path)
    if not file.is_file():
        raise ValueError(f"Hover track not found: {file}")
    try:
        payload = json.loads(file.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"Hover track is not JSON: {file}") from exc
    return _samples_from_payload(payload)


def _grade_hover_checks(samples, thresholds: Thresholds) -> list:
    checks = []
    start = _phase(samples, "start")
    takeoff = _phase(samples, "takeoff")
    hover = _phase(samples, "hover")
    if not start or not hover:
        checks.append(_check("samples", False, 0, 1, "need start and hover samples"))
        return checks

    origin = (start[-1]["x"], start[-1]["y"], start[-1]["z"])
    prearm = _prearm_sample(samples)
    hover_pose = hover[-1]
    height_error = abs(_above_prearm(hover_pose, prearm) - thresholds.takeoff_height_m)
    checks.append(
        _check(
            "height",
            height_error <= thresholds.height_tolerance_m,
            round(height_error, 4),
            thresholds.height_tolerance_m,
            "base_link above its pre-arm height",
        )
    )
    _grade_hover_clock(checks, takeoff, hover, prearm, thresholds)

    drift = max(horizontal((row["x"], row["y"]), (hover[0]["x"], hover[0]["y"])) for row in hover)
    checks.append(
        _check(
            "hover_drift",
            drift <= thresholds.hover_drift_m,
            round(drift, 4),
            thresholds.hover_drift_m,
        )
    )
    return checks


def grade_hover(samples, thresholds: Thresholds | None = None) -> dict:
    """Grade the headless hover. Legs, yaw, and the return are not this flight."""

    thresholds = thresholds or load_thresholds()
    if any(row.get("phase") in _OUT_AND_BACK for row in samples):
        checks = [
            _check(
                "hover_only",
                False,
                1,
                0,
                "this flight is a hover, not an out-and-back",
            )
        ]
        return _result(checks, thresholds)
    return _result(_grade_hover_checks(samples, thresholds), thresholds) | {
        "height_reference": flight_height_reference(samples),
    }


def yaw_passed_command_direction(samples, minimum_deg: float = 90.0) -> dict:
    """True when ENU yaw passes ``minimum_deg`` toward the commanded 180.

    The offboard yaw step adds +pi to the NED yaw setpoint. NED yaw
    positive is clockwise from above, so ENU yaw decreases. A turn that
    goes the other way can still finish 180 deg away; this check requires
    the path to pass the 90 deg mark in the commanded direction.
    """
    hover = _phase(samples, "hover")
    yaw_rows = _phase(samples, "yaw")
    if not hover or not yaw_rows:
        return _check("yaw_pass_90", False, None, minimum_deg, "no hover or yaw samples")
    start = float(hover[-1]["yaw"])
    unwrapped = _unwrap([start] + [float(row["yaw"]) for row in yaw_rows])
    deltas = [value - unwrapped[0] for value in unwrapped[1:]]
    if not deltas:
        return _check("yaw_pass_90", False, None, minimum_deg, "no yaw motion")
    # Commanded ENU change is negative.
    reached = min(deltas)
    wrong = max(deltas)
    passed = reached <= -abs(minimum_deg) and wrong <= 15.0
    return _check(
        "yaw_pass_90",
        passed,
        round(reached, 4),
        -abs(minimum_deg),
        "ENU yaw must pass -90 deg (NED +180 command) and not reverse",
    )


def grade_dropout(samples, hold_m: float = 0.5) -> dict:
    """Brake, hold, and land after a vision cut. Legs and yaw are not graded."""
    checks = []
    brake = _phase(samples, "vision_brake")
    checks.append(_check("brake", bool(brake), len(brake), 1, "vision_brake phase"))
    if brake:
        origin = (brake[0]["x"], brake[0]["y"])
        drift = max(horizontal((row["x"], row["y"]), origin) for row in brake)
        checks.append(
            _check("hold", drift <= hold_m, round(drift, 4), hold_m, "metres moved during brake")
        )
    else:
        checks.append(_check("hold", False, None, hold_m, "no brake samples"))
    settled = _settled_landing(samples)
    prearm = _prearm_sample(samples)
    if settled is not None and prearm is not None:
        height = abs(_above_prearm(settled, prearm))
        checks.append(
            _check(
                "land",
                height <= 0.5,
                round(height, 4),
                0.5,
                "settled base_link above its pre-arm height",
            )
        )
    else:
        checks.append(_check("land", False, None, 0.5, "no land samples"))
    return {
        "passed": bool(checks) and all(item["passed"] for item in checks),
        "checks": checks,
        "thresholds": {"dropout_hold_m": hold_m},
        "height_reference": flight_height_reference(samples),
    }


def grade_mission(samples, thresholds: Thresholds | None = None) -> dict:
    thresholds = thresholds or load_thresholds()
    checks = _grade_hover_checks(samples, thresholds)
    if any(item["name"] == "samples" and not item["passed"] for item in checks):
        return {
            "passed": False,
            "checks": checks,
            "thresholds": asdict(thresholds),
            "height_reference": flight_height_reference(samples),
        }

    start = _phase(samples, "start")
    hover = _phase(samples, "hover")
    leg1 = _phase(samples, "leg1")
    yaw_rows = _phase(samples, "yaw")
    leg2 = _phase(samples, "leg2")
    land = _phase(samples, "land")
    origin = (start[-1]["x"], start[-1]["y"], start[-1]["z"])
    hover_pose = hover[-1]

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

    yaw_report = _grade_final_yaw(checks, samples, thresholds)
    return {
        "passed": all(item["passed"] for item in checks),
        "checks": checks,
        "thresholds": asdict(thresholds),
        "yaw": yaw_report,
        "height_reference": flight_height_reference(samples),
    }
