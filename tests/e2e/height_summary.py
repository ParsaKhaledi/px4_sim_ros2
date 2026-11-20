#!/usr/bin/env python3
"""Baro, vision, and EKF height from a flight ULog, next to estimator flags.

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
    columns = {
        "ekf_z_down_m": (clock, np.asarray(local["z"], dtype=float)),
        "ekf_height_m": (clock, -np.asarray(local["z"], dtype=float)),
    }
    if air is not None and "baro_alt_meter" in air:
        columns["baro_alt_m"] = (
            np.asarray(air["timestamp"], dtype=np.int64),
            np.asarray(air["baro_alt_meter"], dtype=float),
        )
    if baro is not None and "observation" in baro:
        observation = np.asarray(baro["observation"], dtype=float)
        stamps = np.asarray(baro["timestamp"], dtype=np.int64)
        columns["baro_obs_down_m"] = (stamps, observation)
        columns["baro_height_m"] = (stamps, -observation)
    if visual is not None and "position[2]" in visual:
        vision_z = np.asarray(visual["position[2]"], dtype=float)
        stamps = np.asarray(visual["timestamp"], dtype=np.int64)
        columns["vision_z_down_m"] = (stamps, vision_z)
        columns["vision_height_m"] = (stamps, -vision_z)
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
        "height_frames": {
            "baro_alt_m": "vehicle_air_data.baro_alt_meter, up",
            "baro_height_m": "minus estimator_aid_src_baro_hgt.observation",
            "vision_height_m": "minus vehicle_visual_odometry.position[2]",
            "ekf_height_m": "minus vehicle_local_position.z",
        },
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
