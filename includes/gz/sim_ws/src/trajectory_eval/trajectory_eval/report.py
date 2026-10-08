"""Turn resampled trajectories into JSON, CSV, and plots."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np

from trajectory_eval.metrics import (
    PoseSample,
    absolute_trajectory_error,
    final_position_error,
    gps_noise,
    height_error,
    pair_to_ground_truth,
    relative_pose_error,
    takeoff_relative_positions,
    yaw_error,
)
from trajectory_eval.tum import write_tum

ESTIMATE_NAMES = ("gps", "rtabmap", "ekf2")


def evaluate(streams: dict[str, list[PoseSample]], distances: tuple[float, ...] = (1.0, 5.0)) -> dict:
    """Compute the report document. ``streams['ground_truth']`` is required."""
    ground_truth = streams.get("ground_truth") or []
    document: dict = {
        "counts": {name: len(samples) for name, samples in streams.items()},
        "ground_truth_duration_s": (
            ground_truth[-1].t - ground_truth[0].t if len(ground_truth) >= 2 else 0.0
        ),
        "trajectories": {},
    }
    if len(ground_truth) < 2:
        document["error"] = "ground truth has fewer than 2 samples"
        return document
    for name in ESTIMATE_NAMES:
        raw = streams.get(name) or []
        estimate, reference = pair_to_ground_truth(raw, ground_truth)
        if len(estimate) < 2:
            document["trajectories"][name] = {"available": False, "raw_count": len(raw)}
            continue
        entry = {
            "available": True,
            "paired_count": len(estimate),
            "ate": absolute_trajectory_error(estimate, reference),
            "final_position_error": final_position_error(estimate, reference),
            "height_error_m": height_error(estimate, reference),
            "rpe": {},
        }
        if name == "gps":
            entry["yaw_error"] = None
            entry["yaw_error_note"] = "GPS fixes have no body heading, so yaw is not scored."
        else:
            entry["yaw_error"] = yaw_error(estimate, reference)
        for distance in distances:
            entry["rpe"][str(distance)] = relative_pose_error(estimate, reference, distance)
        if name == "gps":
            entry["gps_vs_ground_truth_noise"] = gps_noise(estimate, reference)
        document["trajectories"][name] = entry
    return document


def _flatten(prefix: str, value, rows: list[tuple[str, str, float]]) -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            _flatten(f"{prefix}.{key}" if prefix else str(key), item, rows)
    elif isinstance(value, bool):
        rows.append((prefix, "flag", 1.0 if value else 0.0))
    elif isinstance(value, (int, float)) and not isinstance(value, bool):
        rows.append((prefix, "value", float(value)))
    elif isinstance(value, list) and value and all(isinstance(item, (int, float)) for item in value):
        for index, item in enumerate(value):
            rows.append((f"{prefix}[{index}]", "value", float(item)))


def write_csv(path: Path, document: dict) -> None:
    rows: list[tuple[str, str, float]] = []
    _flatten("", document, rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["metric", "kind", "value"])
        writer.writerows(rows)


def write_plots(directory: Path, streams: dict[str, list[PoseSample]]) -> list[str]:
    """Write XY and error-vs-time plots. Returns filenames, or [] if matplotlib is missing."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    ground_truth = streams.get("ground_truth") or []
    if len(ground_truth) < 2:
        return []
    directory.mkdir(parents=True, exist_ok=True)
    written: list[str] = []
    figure, axis = plt.subplots(figsize=(6, 6))
    gt_xy = takeoff_relative_positions(ground_truth)
    axis.plot(gt_xy[:, 0], gt_xy[:, 1], label="ground_truth", linewidth=2)
    for name in ESTIMATE_NAMES:
        samples, _reference = pair_to_ground_truth(streams.get(name) or [], ground_truth)
        if len(samples) < 2:
            continue
        xy = takeoff_relative_positions(samples)
        axis.plot(xy[:, 0], xy[:, 1], label=name)
    axis.set_aspect("equal", adjustable="datalim")
    axis.set_xlabel("east from takeoff (m)")
    axis.set_ylabel("north from takeoff (m)")
    axis.set_title("Trajectories relative to takeoff")
    axis.legend()
    axis.grid(True, alpha=0.3)
    xy_path = directory / "trajectories_xy.png"
    figure.tight_layout()
    figure.savefig(xy_path)
    plt.close(figure)
    written.append(xy_path.name)

    figure, axis = plt.subplots(figsize=(7, 4))
    for name in ESTIMATE_NAMES:
        samples, reference = pair_to_ground_truth(streams.get(name) or [], ground_truth)
        if len(samples) < 2:
            continue
        est = takeoff_relative_positions(samples)
        ref = takeoff_relative_positions(reference)
        error = np.linalg.norm(est - ref, axis=1)
        times = np.array([sample.t for sample in reference])
        times = times - times[0]
        axis.plot(times, error, label=name)
    axis.set_xlabel("time from start (s)")
    axis.set_ylabel("position error (m)")
    axis.set_title("Unaligned ATE relative to takeoff")
    axis.legend()
    axis.grid(True, alpha=0.3)
    err_path = directory / "ate_unaligned.png"
    figure.tight_layout()
    figure.savefig(err_path)
    plt.close(figure)
    written.append(err_path.name)
    return written


def _json_ready(value):
    """Replace NaN with null so metrics.json is strict JSON."""
    if isinstance(value, dict):
        return {key: _json_ready(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_ready(item) for item in value]
    if isinstance(value, (np.floating, float)):
        number = float(value)
        if math.isnan(number) or math.isinf(number):
            return None
        return number
    if isinstance(value, (np.integer,)):
        return int(value)
    return value


def write_report(directory: Path, streams: dict[str, list[PoseSample]],
                 distances: tuple[float, ...] = (1.0, 5.0)) -> dict:
    """Write TUM files, metrics.json, metrics.csv, and plots."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    for name, samples in streams.items():
        if samples:
            write_tum(directory / f"{name}.tum", samples)
    document = evaluate(streams, distances)
    document["plots"] = write_plots(directory, streams)
    (directory / "metrics.json").write_text(
        json.dumps(_json_ready(document), indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_csv(directory / "metrics.csv", document)
    return document
