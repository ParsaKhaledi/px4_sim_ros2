"""Pure checks used by /sim/preflight_check.

The service returns success only when every line starts with PASS. The message
is the full reason list, one check per line.
"""

from __future__ import annotations

from collections import deque

import numpy as np


class RateTracker:
    """Wall-clock message rate over a sliding window."""

    def __init__(self, window_s: float) -> None:
        self.window_s = window_s
        self.times: deque[float] = deque()

    def add(self, stamp: float) -> None:
        self.times.append(stamp)
        self._trim(stamp)

    def _trim(self, now: float) -> None:
        while self.times and now - self.times[0] > self.window_s:
            self.times.popleft()

    def hz(self, now: float) -> float:
        self._trim(now)
        if len(self.times) < 2:
            return 0.0
        span = self.times[-1] - self.times[0]
        if span <= 0.0:
            return 0.0
        return (len(self.times) - 1) / span


def line(ok: bool, text: str) -> str:
    return f"{'PASS' if ok else 'FAIL'} {text}"


def check_min(name: str, value: float, minimum: float, unit: str) -> tuple[bool, str]:
    ok = value >= minimum
    return ok, line(ok, f"{name}: {value:.3f} {unit} >= {minimum:.3f} {unit}")


def check_max(name: str, value: float, maximum: float, unit: str) -> tuple[bool, str]:
    ok = value <= maximum
    return ok, line(ok, f"{name}: {value:.3f} {unit} <= {maximum:.3f} {unit}")


def relative_position_error(current_est, origin_est, current_gt, origin_gt) -> float:
    """Distance between two motions measured from each stream's start pose."""
    est = np.asarray(current_est, dtype=float) - np.asarray(origin_est, dtype=float)
    gt = np.asarray(current_gt, dtype=float) - np.asarray(origin_gt, dtype=float)
    return float(np.linalg.norm(est - gt))


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Return (x, y, z, w) for R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    half_roll, half_pitch, half_yaw = roll * 0.5, pitch * 0.5, yaw * 0.5
    cr, sr = np.cos(half_roll), np.sin(half_roll)
    cp, sp = np.cos(half_pitch), np.sin(half_pitch)
    cy, sy = np.cos(half_yaw), np.sin(half_yaw)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return float(x), float(y), float(z), float(w)


def parse_spawn_pose(text: str) -> tuple[float, float, float, float, float, float]:
    """Parse PX4_GZ_MODEL_POSE ``x,y,z,roll,pitch,yaw``."""
    parts = [part.strip() for part in text.split(",")]
    if len(parts) != 6:
        raise ValueError(f"PX4_GZ_MODEL_POSE needs 6 comma-separated numbers, got {text!r}")
    x, y, z, roll, pitch, yaw = (float(part) for part in parts)
    return x, y, z, roll, pitch, yaw


def match_versioned_topic(names: list[str], suffix: str) -> str | None:
    """Pick ``/fmu/out/<suffix>`` or the highest ``<suffix>_vN`` topic."""
    exact = []
    versioned = []
    for name in names:
        tail = name.rstrip("/").split("/")[-1]
        if tail == suffix:
            exact.append(name)
        prefix = suffix + "_v"
        if tail.startswith(prefix) and tail[len(prefix):].isdigit():
            versioned.append((int(tail[len(prefix):]), name))
    if versioned:
        return sorted(versioned)[-1][1]
    if exact:
        return exact[0]
    return None


def summarize(results: list[tuple[bool, str]]) -> tuple[bool, str]:
    """Join check lines. Success is true only when every check passed."""
    success = all(ok for ok, _text in results) and len(results) > 0
    message = "\n".join(text for _ok, text in results)
    return success, message
