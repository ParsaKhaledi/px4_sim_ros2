"""Run loading, segmentation, metrics, and grading for one flight."""

from __future__ import annotations

import json
import sys
from collections.abc import Callable
from pathlib import Path
from typing import Any

from e2e_limits import E2ELimits
from flight_analysis.grade import error_dict
from flight_analysis.grade import grade
from flight_analysis.log import FlightLog
from flight_analysis.log import Track
from flight_analysis.log import actuator_series
from flight_analysis.log import command_track
from flight_analysis.log import estimate_track
from flight_analysis.log import flag_series
from flight_analysis.log import ground_truth_track
from flight_analysis.log import parameter
from flight_analysis.log import rate_series
from flight_analysis.log import tilt_series
from flight_analysis.log import vision_delay_seconds
from flight_analysis.metrics import measure_flight
from flight_analysis.plots import write_plots
from flight_analysis.segment import segment_command
from flight_analysis.spawn import ResolvedSpawn
from flight_analysis.spawn import resolve_spawn
from flight_analysis.tum import TimeAlignment
from flight_analysis.tum import align_tum_to_ulog
from flight_analysis.tum import alignment_from_px4_offset
from flight_analysis.tum import load_tum
from flight_analysis.tum import shift_track


def analyze_flight(
    log: FlightLog,
    limits: E2ELimits,
    tum_path: Path | None = None,
    log_path: Path | None = None,
    run_id: str = "flight",
    spawn_json: Path | None = None,
    spawn_xyz: tuple[float, float, float] | None = None,
    spawn_yaw: float | None = None,
    clock_offset_s: float | None = None,
    warn: Callable[[str], None] | None = None,
) -> dict[str, Any]:
    """Grade one flight and return the ``metrics.json`` document.

    A TUM file is preferred over ``*_groundtruth`` topics. The file is
    Gazebo world ENU, so a spawn pose is required: ``spawn.json`` beside
    the log, or ``spawn_xyz`` and ``spawn_yaw``. ``clock_offset_s`` is
    ``px4_offset_s`` (ROS sim time minus PX4 boot time) and wins over the
    file. Without either clock, the climb edge is estimated and ``warn``
    is called. Raises ``SpawnError`` when the pose is missing and
    ``TimeAlignmentError`` when the clocks do not overlap.
    """

    command = command_track(log)
    estimate = estimate_track(log)
    logged_truth = ground_truth_track(log)
    spawn = None
    if tum_path is not None:
        spawn = resolve_spawn(log_path, spawn_json, spawn_xyz, spawn_yaw, clock_offset_s)
    truth, truth_meta = _resolve_truth(
        estimate,
        logged_truth,
        tum_path,
        spawn,
        _warn if warn is None else warn,
    )
    legs = segment_command(command)
    tilt = tilt_series(log)
    rates = rate_series(log)
    measured = measure_flight(
        command,
        estimate,
        legs,
        truth,
        None if tilt is None else tilt[0],
        None if tilt is None else tilt[1],
        parameter(log, "MPC_TILTMAX_AIR"),
        None if rates is None else rates[0],
        None if rates is None else rates[1],
        actuator_series(log),
        flag_series(log),
        vision_delay_seconds(log),
        limits.settle_hold_s,
        limits.settle_timeout_s,
        limits.settle_tolerance_m,
        limits.yaw_settle_deg,
        limits.height_tolerance_m,
    )
    graded = grade(measured, limits)
    report = _document(run_id, log_path, truth_meta, graded, measured)
    report["_tracks"] = {
        "command": command,
        "estimate": estimate,
        "ground_truth": truth,
        "tilt_time_s": None if tilt is None else tilt[0],
        "tilt_deg": None if tilt is None else _degrees(tilt[1]),
        "tilt_limit_deg": parameter(log, "MPC_TILTMAX_AIR"),
    }
    return report


def write_report(report: dict[str, Any], output: Path) -> None:
    """Write ``metrics.json`` and the PNG plots into ``output``."""

    output.mkdir(parents=True, exist_ok=True)
    tracks = report["_tracks"]
    public = {key: value for key, value in report.items() if key != "_tracks"}
    (output / "metrics.json").write_text(
        json.dumps(_json_ready(public), indent=2) + "\n",
        encoding="utf-8",
    )
    write_plots(
        output,
        str(public["run_id"]),
        tracks["command"],
        tracks["estimate"],
        tracks["ground_truth"],
        tracks["tilt_time_s"],
        tracks["tilt_deg"],
        tracks["tilt_limit_deg"],
    )


def _warn(message: str) -> None:
    """Print a ground-truth warning on stderr."""

    print(f"flight_analysis: {message}", file=sys.stderr)


def _resolve_truth(
    estimate: Track,
    logged: Track | None,
    tum_path: Path | None,
    spawn: ResolvedSpawn | None,
    warn: Callable[[str], None],
) -> tuple[Track | None, dict[str, Any]]:
    """Pick TUM truth when a file is given, otherwise the logged topics."""

    if tum_path is not None:
        if spawn is None:
            raise RuntimeError("TUM ground truth requires a resolved spawn pose")
        tum = load_tum(tum_path, spawn.xyz_m, spawn.yaw_rad)
        if spawn.px4_offset_s is None:
            alignment = align_tum_to_ulog(estimate, tum)
            clock_source = "climb_edge_estimate"
            warn(
                "no px4_offset_s in spawn.json and no --clock-offset; "
                "using the climb-edge estimate "
                f"(px4_offset_s={-alignment.offset_s:.3f} s, "
                "ROS sim time minus PX4 boot time)"
            )
        else:
            alignment = alignment_from_px4_offset(estimate, tum, spawn.px4_offset_s)
            clock_source = spawn.clock_source or "cli"
        detail = (
            "TUM trajectory in Gazebo world ENU, moved into the spawn frame, "
            "then converted from ENU/FLU to NED/FRD"
        )
        if logged is not None:
            detail += "; vehicle_local_position_groundtruth was logged and was not used"
        meta = _truth_meta("tum", detail, alignment)
        meta["spawn"] = {
            "xyz_m": list(spawn.xyz_m),
            "yaw_rad": spawn.yaw_rad,
            "xyz_source": spawn.xyz_source,
            "yaw_source": spawn.yaw_source,
        }
        meta["clock"] = {
            "source": clock_source,
            "px4_offset_s": -alignment.offset_s,
            "applied_offset_s": alignment.offset_s,
        }
        return shift_track(tum, alignment.offset_s), meta
    if logged is not None:
        return logged, _truth_meta(
            "ulog",
            "vehicle_local_position_groundtruth",
            None,
        )
    return None, _truth_meta(
        "none",
        "no TUM file and vehicle_local_position_groundtruth was not logged",
        None,
    )


def _truth_meta(source: str, detail: str, alignment: TimeAlignment | None) -> dict[str, Any]:
    """JSON block describing which ground-truth source was used."""

    if alignment is None:
        return {
            "source": source,
            "detail": detail,
            "time_offset_s": 0.0 if source == "ulog" else None,
            "correlation": None,
            "alignment": "same_clock" if source == "ulog" else None,
            "overlap_s": None,
        }
    return {
        "source": source,
        "detail": detail,
        "time_offset_s": alignment.offset_s,
        "correlation": alignment.correlation,
        "alignment": alignment.method,
        "overlap_s": alignment.overlap_s,
    }


def _document(
    run_id: str,
    log_path: Path | None,
    truth_meta: dict[str, Any],
    graded: dict[str, Any],
    measured,
) -> dict[str, Any]:
    """Assemble the public metrics document, plus private plot tracks."""

    return {
        "run_id": run_id,
        "log": None if log_path is None else str(log_path),
        "time_base": "sim_seconds",
        "ground_truth": truth_meta,
        "passed": graded["passed"],
        "limits": graded["limits"],
        "checks": graded["checks"],
        "flight": {
            "tracking_error": error_dict(measured.tracking),
            "estimation_error": error_dict(measured.estimation),
            "tilt_peak_deg": measured.tilt_peak_deg,
            "tilt_limit_deg": measured.tilt_limit_deg,
            "actuator_saturation": measured.saturation,
            "body_rates": measured.rates,
            "height_divergence": measured.height_divergence,
            "vision_delay": measured.vision_delay,
            "vision_ground_truth_leak": measured.leak,
            "return_distance_m": measured.return_distance_m,
        },
        "legs": [_leg_dict(row) for row in measured.legs],
    }


def _leg_dict(row) -> dict[str, Any]:
    """JSON form of one leg, including its estimator flags and errors."""

    leg = row.leg
    return {
        "index": leg.index,
        "kind": leg.kind,
        "t_start_s": leg.t_start_s,
        "t_arrived_s": leg.t_arrived_s,
        "t_end_s": leg.t_end_s,
        "target_north_m": leg.target_north_m,
        "target_east_m": leg.target_east_m,
        "target_down_m": leg.target_down_m,
        "target_yaw_rad": leg.target_yaw_rad,
        "overshoot_m": row.overshoot_m,
        "yaw_overshoot_deg": row.yaw_overshoot_deg,
        "settling_time_s": row.settling_time_s,
        "yaw_settling_time_s": row.yaw_settling_time_s,
        "stopping_distance_m": row.stopping_distance_m,
        "tracking_error": error_dict(row.tracking),
        "estimation_error": error_dict(row.estimation),
        "estimator_flags": row.flags,
    }


def _degrees(tilt_rad):
    """Convert a tilt array to degrees for the plot."""

    import numpy as np

    return np.degrees(tilt_rad)


def _json_ready(value: Any) -> Any:
    """Replace numpy scalars and non-finite floats so ``json`` can write them."""

    import math

    import numpy as np

    if isinstance(value, dict):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    if isinstance(value, np.ndarray):
        return [_json_ready(item) for item in value.tolist()]
    if isinstance(value, np.floating):
        value = float(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            return None
        return round(value, 6)
    return value

