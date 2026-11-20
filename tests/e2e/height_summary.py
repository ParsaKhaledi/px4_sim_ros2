#!/usr/bin/env python3
"""Baro, vision, and EKF height from a flight ULog, next to estimator flags.

Each series is also written above its own on-ground pre-arm value, so the
numbers are base_link climbs. The 0.24 m x500_base pose is not added here:
baro and the IMU are already on base_link, and the vision pose is the model
origin with that offset constant, so it cancels once the series is zeroed.
The fake RTAB-Map drift is +East in the odom frame. Its z term is 0.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np

FLAG_FIELDS = (
    "cs_ev_pos",
    "cs_ev_yaw",
    "cs_ev_hgt",
    "cs_ev_vel",
    "cs_baro_hgt",
    "cs_gnss_pos",
    "cs_gps_hgt",
)


def _nearest(timestamps_us: np.ndarray, t_us: int, max_gap_us: int) -> int | None:
    if timestamps_us.size == 0:
        return None
    index = int(np.searchsorted(timestamps_us, t_us))
    choices = []
    if index < timestamps_us.size:
        choices.append(index)
    if index > 0:
        choices.append(index - 1)
    best = min(choices, key=lambda item: abs(int(timestamps_us[item]) - int(t_us)))
    if abs(int(timestamps_us[best]) - int(t_us)) > int(max_gap_us):
        return None
    return best


def aligned_rows(
    clock_us: np.ndarray,
    columns: dict,
    *,
    step_us: int = 1_000_000,
    max_gap_us: int = 250_000,
) -> list[dict]:
    """One row per ``step_us`` of ``clock_us``.

    ``columns`` maps a name to ``(timestamps_us, values)``. ``values`` is a
    1-D array or a list of dicts (estimator flags).
    """

    clock = np.asarray(clock_us, dtype=np.int64)
    if clock.size == 0:
        return []
    origin = int(clock[0])
    rows = []
    t_us = origin
    last = int(clock[-1])
    while t_us <= last:
        row = {"t_s": round((t_us - origin) / 1e6, 3)}
        for name, (stamps, values) in columns.items():
            stamps_us = np.asarray(stamps, dtype=np.int64)
            index = _nearest(stamps_us, t_us, max_gap_us)
            if index is None:
                row[name] = None
            else:
                value = values[index]
                if isinstance(value, dict):
                    row[name] = value
                else:
                    number = float(value)
                    row[name] = None if not np.isfinite(number) else round(number, 4)
        rows.append(row)
        t_us += int(step_us)
    return rows


# Same figure as grading.BASE_LINK_ABOVE_MODEL_ORIGIN_M. Repeated here so the
# summary does not import the grader.
BASE_LINK_ABOVE_MODEL_ORIGIN_M = 0.24

SOURCE_POINTS = {
    "ekf": (
        "vehicle_local_position.z is NED down of the EKF position. The IMU is "
        "on base_link with no extra pose, so this is base_link in the EKF "
        "frame. EKF2_HGT_REF=0 puts the origin at the baro height when the "
        "estimator starts, not at the Gazebo model origin. A resting z near "
        "-0.24 m means that run's origin happened to sit near the model origin, "
        "0.24 m below base_link."
    ),
    "baro_alt": (
        "vehicle_air_data.baro_alt_meter is the air_pressure sensor on "
        "base_link, no extra pose, up-positive. The resting value is the "
        "noisy pressure reference (about 0.15 m of scatter), not the 0.24 m "
        "link offset."
    ),
    "baro_observation": (
        "estimator_aid_src_baro_hgt.observation is that same baro height in "
        "the EKF's NED frame. Zeroing it removes the reference, so the climb "
        "is base_link above its pre-arm baro reading."
    ),
    "vision": (
        "vehicle_visual_odometry.position[2] is NED down of the pose the "
        "bridge forwarded. The fake node copies /ground_truth/odom, which is "
        "the model origin, and publishes it as odom -> base_link zeroed at "
        "the first sample. The 0.24 m base_link offset is missing from the "
        "absolute pose and cancels in the climb."
    ),
}


def prearm_level(times_us, values) -> float:
    """Median up-height in the 2 s on the ground just before the climb."""

    times = np.asarray(times_us, dtype=np.int64)
    up = np.asarray(values, dtype=float)
    finite = np.isfinite(up)
    times = times[finite]
    up = up[finite]
    if up.size == 0:
        return 0.0
    t0 = int(times[0])
    early = up[times <= t0 + 5_000_000]
    if early.size == 0:
        early = up[: min(10, up.size)]
    seed = float(np.median(early))
    moved = np.flatnonzero(np.abs(up - seed) > 0.4)
    if moved.size == 0:
        return seed
    climb_t = int(times[int(moved[0])])
    held = up[(times >= climb_t - 2_000_000) & (times < climb_t)]
    if held.size < 5:
        held = up[(times >= climb_t - 5_000_000) & (times < climb_t)]
    if held.size == 0:
        return seed
    return float(np.median(held))


def _hover_and_landing(times_us, up_values, zero: float, landing_after_us: int | None = None) -> dict:
    times = np.asarray(times_us, dtype=np.int64)
    up = np.asarray(up_values, dtype=float)
    finite = np.isfinite(up)
    times = times[finite]
    up = up[finite]
    empty = {
        "prearm_up_m": round(zero, 4),
        "raw_hover_up_m": None,
        "hover_above_prearm_m": None,
        "landing_above_prearm_m": None,
    }
    if up.size == 0:
        return empty
    relative = up - zero
    peak = float(np.max(relative))
    hover = relative >= peak - 0.4
    end = int(times[-1]) if landing_after_us is None else int(landing_after_us)
    landed = times >= end - 2_000_000
    landing = None
    if np.any(landed):
        landing = round(float(np.median(relative[landed])), 4)
    return {
        "prearm_up_m": round(float(zero), 4),
        "raw_hover_up_m": round(float(np.median(up[hover])), 4),
        "hover_above_prearm_m": round(float(np.median(relative[hover])), 4),
        "landing_above_prearm_m": landing,
    }


def _dataset(ulg, name: str):
    matches = [item for item in ulg.data_list if item.name == name]
    if not matches:
        return None
    return matches[0].data


def summarize_ulog(path: Path, *, step_us: int = 1_000_000) -> dict:
    from pyulog import ULog

    ulg = ULog(str(path))
    local = _dataset(ulg, "vehicle_local_position")
    air = _dataset(ulg, "vehicle_air_data")
    baro = _dataset(ulg, "estimator_aid_src_baro_hgt")
    visual = _dataset(ulg, "vehicle_visual_odometry")
    flags = _dataset(ulg, "estimator_status_flags")
    if local is None:
        return {"rows": [], "note": "vehicle_local_position missing"}
    clock = np.asarray(local["timestamp"], dtype=np.int64)
    ekf_up = -np.asarray(local["z"], dtype=float)
    ekf_zero = prearm_level(clock, ekf_up)
    flight_end = int(clock[-1])
    columns = {
        "ekf_z_down_m": (clock, np.asarray(local["z"], dtype=float)),
        "ekf_height_m": (clock, ekf_up),
        "ekf_above_prearm_m": (clock, ekf_up - ekf_zero),
    }
    sources = {
        "ekf": {
            "point": SOURCE_POINTS["ekf"],
            **_hover_and_landing(clock, ekf_up, ekf_zero, flight_end),
        }
    }
    if air is not None and "baro_alt_meter" in air:
        stamps = np.asarray(air["timestamp"], dtype=np.int64)
        baro_alt = np.asarray(air["baro_alt_meter"], dtype=float)
        baro_zero = prearm_level(stamps, baro_alt)
        columns["baro_alt_m"] = (stamps, baro_alt)
        columns["baro_alt_above_prearm_m"] = (stamps, baro_alt - baro_zero)
        sources["baro_alt"] = {
            "point": SOURCE_POINTS["baro_alt"],
            **_hover_and_landing(stamps, baro_alt, baro_zero, flight_end),
        }
    if baro is not None and "observation" in baro:
        observation = np.asarray(baro["observation"], dtype=float)
        stamps = np.asarray(baro["timestamp"], dtype=np.int64)
        baro_up = -observation
        baro_zero = prearm_level(stamps, baro_up)
        columns["baro_obs_down_m"] = (stamps, observation)
        columns["baro_height_m"] = (stamps, baro_up)
        columns["baro_above_prearm_m"] = (stamps, baro_up - baro_zero)
        sources["baro_observation"] = {
            "point": SOURCE_POINTS["baro_observation"],
            **_hover_and_landing(stamps, baro_up, baro_zero, flight_end),
        }
    if visual is not None and "position[2]" in visual:
        vision_z = np.asarray(visual["position[2]"], dtype=float)
        stamps = np.asarray(visual["timestamp"], dtype=np.int64)
        vision_up = -vision_z
        vision_zero = prearm_level(stamps, vision_up)
        columns["vision_z_down_m"] = (stamps, vision_z)
        columns["vision_height_m"] = (stamps, vision_up)
        columns["vision_above_prearm_m"] = (stamps, vision_up - vision_zero)
        sources["vision"] = {
            "point": SOURCE_POINTS["vision"],
            **_hover_and_landing(stamps, vision_up, vision_zero, flight_end),
        }
    if flags is not None:
        count = len(flags["timestamp"])
        packed = []
        for index in range(count):
            packed.append({
                name: bool(int(flags[name][index]))
                for name in FLAG_FIELDS
                if name in flags
            })
        columns["flags"] = (np.asarray(flags["timestamp"], dtype=np.int64), packed)
    rows = aligned_rows(clock, columns, step_us=step_us)
    return {
        "frame": "base_link",
        "zero": "each source's own on-ground pre-arm value",
        "base_link_above_model_origin_m": BASE_LINK_ABOVE_MODEL_ORIGIN_M,
        "height_frames": {
            "baro_alt_m": "vehicle_air_data.baro_alt_meter, up, sensor on base_link",
            "baro_height_m": "minus estimator_aid_src_baro_hgt.observation",
            "baro_above_prearm_m": "baro observation height minus its pre-arm value",
            "vision_height_m": "minus vehicle_visual_odometry.position[2], model origin published as base_link",
            "vision_above_prearm_m": "vision height minus its pre-arm value",
            "ekf_height_m": "minus vehicle_local_position.z, base_link in the EKF frame",
            "ekf_above_prearm_m": "EKF height minus its pre-arm value",
        },
        "sources": sources,
        "rows": rows,
    }


def fake_z_drift() -> dict:
    """The fake node adds drift on odom +x only. z is identically zero."""

    raw = os.environ.get("FAKE_VISION_DRIFT_MPS", "0").strip() or "0"
    east = float(raw)
    return {
        "east_mps": east,
        "north_mps": 0.0,
        "z_mps": 0.0,
        "detail": (
            "FakeRtabmap drift is +East in the odom frame (drift_mps * dt, 0, 0). "
            "The z component is 0 whether or not FAKE_VISION_DRIFT_MPS is 0. "
            f"This run's east drift is {east} m/s."
        ),
    }


def write_flight_summary(flight_dir: Path) -> dict:
    flight_dir = Path(flight_dir)
    grade_path = flight_dir / "grade.json"
    summary_path = flight_dir / "summary.json"
    ulogs = sorted(flight_dir.glob("attempt-*/flight.ulg"))
    grade = {}
    if grade_path.is_file():
        grade = json.loads(grade_path.read_text(encoding="utf-8"))
    heights = {"rows": []}
    if ulogs:
        heights = summarize_ulog(ulogs[-1])
    drift = fake_z_drift()
    grade["heights"] = heights
    grade["fake_z_drift"] = drift
    if grade_path.is_file():
        grade_path.write_text(json.dumps(grade, indent=2) + "\n", encoding="utf-8")
    summary = {
        "status": grade.get("status"),
        "passed": grade.get("passed"),
        "yaw": grade.get("yaw"),
        "height_reference": grade.get("height_reference"),
        "checks": [
            {key: item.get(key) for key in ("name", "passed", "measured", "limit", "detail")}
            for item in grade.get("checks") or []
        ],
        "heights": heights,
        "fake_z_drift": drift,
        "ulg": str(ulogs[-1]) if ulogs else None,
    }
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Write the flight height summary")
    parser.add_argument("--flight-dir", required=True)
    args = parser.parse_args()
    write_flight_summary(Path(args.flight_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
