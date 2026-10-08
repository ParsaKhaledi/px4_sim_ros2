"""Pure checks used by /sim/preflight_check.

The service returns success only when every line starts with PASS. The message
is the full reason list, one check per line.
"""

from __future__ import annotations

import os
from collections import deque

import numpy as np


def _span_hz(stamps: deque[float], now: float, window_s: float) -> float:
    while stamps and now - stamps[0] > window_s:
        stamps.popleft()
    if len(stamps) < 2:
        return 0.0
    span = stamps[-1] - stamps[0]
    if span <= 0.0:
        return 0.0
    return (len(stamps) - 1) / span


class RateTracker:
    """Message rate over a sliding window, in sim time and in wall time."""

    def __init__(self, window_s: float) -> None:
        self.window_s = window_s
        self.times: deque[float] = deque()
        self.sim_times: deque[float] = deque()

    def add(self, stamp: float, sim_s: float | None = None) -> None:
        """Record a wall-clock arrival and, when known, the header stamp."""
        self.times.append(stamp)
        _span_hz(self.times, stamp, self.window_s)
        if sim_s is not None:
            self.sim_times.append(sim_s)
            _span_hz(self.sim_times, sim_s, self.window_s)

    def hz(self, now: float) -> float:
        """Wall-clock rate. Kept so older callers keep working."""
        return self.hz_wall(now)

    def hz_wall(self, now: float) -> float:
        return _span_hz(self.times, now, self.window_s)

    def hz_sim(self, now_sim: float) -> float:
        return _span_hz(self.sim_times, now_sim, self.window_s)


def header_stamp_s(msg) -> float | None:
    """Header stamp in seconds, or None when the message has no header."""
    header = getattr(msg, "header", None)
    if header is None or not hasattr(header, "stamp"):
        return None
    stamp = header.stamp
    if not hasattr(stamp, "sec"):
        return None
    return float(stamp.sec) + float(getattr(stamp, "nanosec", 0)) * 1e-9


def gated_rate_hz(sim_hz: float | None, wall_hz: float, rtf: float | None) -> float:
    """Sim-time rate, or wall rate divided by real-time factor when there is no header."""
    if sim_hz is not None:
        return sim_hz
    if rtf is not None and rtf > 0.0:
        return wall_hz / rtf
    return wall_hz


def _env_float(environ: dict[str, str], name: str) -> float | None:
    raw = environ.get(name, "")
    if raw == "":
        return None
    return float(raw)


def expected_sensor_hz(kind: str, environ: dict[str, str] | None = None) -> float:
    """Camera or IMU rate from CAM_RATE_HZ / IMU_RATE_HZ or VISION_PROFILE.

    ``full`` is 30 Hz cameras and 200 Hz IMU. ``cpu`` is 10 Hz cameras and
    100 Hz IMU. With no profile the full rates are used.
    """
    env = os.environ if environ is None else environ
    if kind == "camera":
        override = _env_float(env, "CAM_RATE_HZ")
        if override is not None:
            return override
        if env.get("VISION_PROFILE", "").strip().lower() == "cpu":
            return 10.0
        return 30.0
    if kind != "imu":
        raise ValueError(f"unknown sensor kind {kind!r}")
    override = _env_float(env, "IMU_RATE_HZ")
    if override is not None:
        return override
    if env.get("VISION_PROFILE", "").strip().lower() == "cpu":
        return 100.0
    return 200.0


def expected_imu_hz(environ: dict[str, str] | None = None) -> float:
    """Oak IMU rate from ``IMU_RATE_HZ`` or ``VISION_PROFILE``.

    ``px4`` does not use this. Its preflight minimum is 80 Hz in sim time.
    """
    env = os.environ if environ is None else environ
    return expected_sensor_hz("imu", env)


def minimum_rate_hz(kind: str, environ: dict[str, str] | None = None) -> float:
    """Preflight minimum.

    An explicit ``PREFLIGHT_MIN_*_HZ`` wins. Otherwise the camera minimum and
    the oak IMU minimum are half the expected rate. ``IMU_SOURCE=px4`` uses
    80 Hz in sim time. XRCE copies ``sensor_combined`` at most once per 10 ms,
    so 100 Hz is a ceiling and ordinary jitter would fail a 100 Hz gate.
    """
    env = os.environ if environ is None else environ
    name = "PREFLIGHT_MIN_CAMERA_HZ" if kind == "camera" else "PREFLIGHT_MIN_IMU_HZ"
    override = _env_float(env, name)
    if override is not None:
        return override
    if kind == "imu" and env.get("IMU_SOURCE", "").strip().lower() == "px4":
        return 80.0
    if kind == "imu":
        return 0.5 * expected_imu_hz(env)
    return 0.5 * expected_sensor_hz(kind, env)


def default_min_rtf(environ: dict[str, str] | None = None) -> float:
    """0.15 under software rendering, otherwise 0.8. PREFLIGHT_MIN_RTF overrides."""
    env = os.environ if environ is None else environ
    override = _env_float(env, "PREFLIGHT_MIN_RTF")
    if override is not None:
        return override
    software = env.get("HEADLESS_SOFTWARE", "0").strip().lower()
    if software in {"1", "true", "yes"}:
        return 0.15
    return 0.8


def line(ok: bool, text: str) -> str:
    return f"{'PASS' if ok else 'FAIL'} {text}"


def check_min(name: str, value: float, minimum: float, unit: str) -> tuple[bool, str]:
    ok = value >= minimum
    return ok, line(ok, f"{name}: {value:.3f} {unit} >= {minimum:.3f} {unit}")


def check_rate(topic: str, sim_hz: float, wall_hz: float, minimum: float) -> tuple[bool, str]:
    """Gate on the sim-time rate and report the wall rate beside it."""
    ok = sim_hz >= minimum
    relation = ">=" if ok else "<"
    text = (
        f"{topic}: {sim_hz:.1f} Hz sim ({wall_hz:.1f} Hz wall) "
        f"{relation} {minimum:.1f} Hz sim"
    )
    return ok, line(ok, text)


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
