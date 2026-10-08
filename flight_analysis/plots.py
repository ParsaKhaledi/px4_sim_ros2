"""PNG plots of one graded flight. Matplotlib is imported only when drawing."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from flight_analysis.log import Track


def write_plots(
    output: Path,
    run_id: str,
    command: Track,
    estimate: Track,
    ground_truth: Track | None,
    tilt_time_s: np.ndarray | None,
    tilt_deg: np.ndarray | None,
    tilt_limit_deg: float | None,
) -> list[Path]:
    """Write the position, yaw, tilt, and error figures. Returns their paths."""

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    paths = [
        _position_plot(plt, output / "position.png", run_id, command, estimate, ground_truth),
        _yaw_plot(plt, output / "yaw.png", run_id, command, estimate, ground_truth),
        _tilt_plot(plt, output / "tilt.png", run_id, tilt_time_s, tilt_deg, tilt_limit_deg),
        _error_plot(plt, output / "errors.png", run_id, command, estimate, ground_truth),
    ]
    return paths


def _position_plot(plt, path: Path, run_id: str, command: Track, estimate: Track, truth: Track | None) -> Path:
    """North, east, and up for the setpoint, the estimate, and ground truth."""

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 8))
    series = (
        ("north", command.north_m, estimate.north_m, None if truth is None else truth.north_m),
        ("east", command.east_m, estimate.east_m, None if truth is None else truth.east_m),
        ("up", -command.down_m, -estimate.down_m, None if truth is None else -truth.down_m),
    )
    for axis, (label, cmd, est, gt) in zip(axes, series, strict=True):
        axis.plot(command.time_s, cmd, color="0.5", label="setpoint")
        axis.plot(estimate.time_s, est, label="estimate")
        if truth is not None and gt is not None:
            axis.plot(truth.time_s, gt, label="ground truth")
        axis.set_ylabel(f"{label} (m)")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    axes[-1].set_xlabel("sim time (s)")
    fig.suptitle(f"{run_id} position")
    return _save(fig, path)


def _yaw_plot(plt, path: Path, run_id: str, command: Track, estimate: Track, truth: Track | None) -> Path:
    """Yaw in degrees, unwrapped so a half-turn does not jump across the plot."""

    fig, axis = plt.subplots(figsize=(8, 4))
    axis.plot(command.time_s, np.degrees(np.unwrap(command.yaw_rad)), color="0.5", label="setpoint")
    axis.plot(estimate.time_s, np.degrees(np.unwrap(estimate.yaw_rad)), label="estimate")
    if truth is not None:
        axis.plot(truth.time_s, np.degrees(np.unwrap(truth.yaw_rad)), label="ground truth")
    axis.set_ylabel("yaw (deg)")
    axis.set_xlabel("sim time (s)")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    fig.suptitle(f"{run_id} yaw")
    return _save(fig, path)


def _tilt_plot(
    plt,
    path: Path,
    run_id: str,
    time_s: np.ndarray | None,
    tilt_deg: np.ndarray | None,
    limit_deg: float | None,
) -> Path:
    """Tilt off vertical, with the logged tilt limit when the parameter exists."""

    fig, axis = plt.subplots(figsize=(8, 4))
    if time_s is not None and tilt_deg is not None:
        axis.plot(time_s, tilt_deg, label="tilt")
    if limit_deg is not None:
        axis.axhline(limit_deg, color="0.3", linestyle="--", label="MPC_TILTMAX_AIR")
    axis.set_ylabel("tilt (deg)")
    axis.set_xlabel("sim time (s)")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    fig.suptitle(f"{run_id} tilt")
    return _save(fig, path)


def _error_plot(plt, path: Path, run_id: str, command: Track, estimate: Track, truth: Track | None) -> Path:
    """Horizontal and height error, tracking separate from estimation."""

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    horizontal, vertical = _against(estimate, command)
    axes[0].plot(estimate.time_s, horizontal, label="tracking")
    axes[1].plot(estimate.time_s, vertical, label="tracking")
    if truth is not None:
        est_h, est_v = _against(estimate, truth)
        axes[0].plot(estimate.time_s, est_h, label="estimation")
        axes[1].plot(estimate.time_s, est_v, label="estimation")
    axes[0].set_ylabel("horizontal error (m)")
    axes[1].set_ylabel("height error (m)")
    axes[1].set_xlabel("sim time (s)")
    for axis in axes:
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    title = f"{run_id} tracking vs estimation"
    if truth is None:
        title += " (no ground truth)"
    fig.suptitle(title)
    return _save(fig, path)


def _against(estimate: Track, reference: Track) -> tuple[np.ndarray, np.ndarray]:
    """Horizontal and height error of the estimate against a reference track."""

    time_s = estimate.time_s
    if reference.time_s.shape[0] < 2:
        zeros = np.zeros(time_s.shape[0])
        return zeros, zeros
    inside = (time_s >= reference.time_s[0]) & (time_s <= reference.time_s[-1])
    north = np.interp(time_s, reference.time_s, reference.north_m)
    east = np.interp(time_s, reference.time_s, reference.east_m)
    down = np.interp(time_s, reference.time_s, reference.down_m)
    horizontal = np.hypot(estimate.north_m - north, estimate.east_m - east)
    vertical = np.abs((-estimate.down_m) - (-down))
    horizontal = np.where(inside, horizontal, np.nan)
    vertical = np.where(inside, vertical, np.nan)
    return horizontal, vertical


def _save(fig, path: Path) -> Path:
    """Write one figure and close it."""

    fig.tight_layout()
    fig.savefig(path, dpi=120)
    fig.clf()
    import matplotlib.pyplot as plt

    plt.close(fig)
    return path
