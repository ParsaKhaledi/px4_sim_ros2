"""Pure checks used by /sim/preflight_check.

The service returns success when every check passed. The message is the full
reason list, one check per line, each starting with PASS, FAIL, or SKIP.
A SKIP line means that check did not run. It is not a PASS and it does not
fail the service.

Rates are sim time from message headers and ``/clock``. The wall rate is
reported beside them. Thresholds come from ``PREFLIGHT_MIN_*``,
``IMU_RATE_HZ``, ``IMU_SOURCE``, ``CAM_RATE_HZ``, ``VISION_PROFILE``,
``HEADLESS_SOFTWARE``, ``PX4_GZ_MODEL``, ``PX4_SIM_MODEL``, ``CameraType``,
and ``LIDAR_DOWN``.
"""

from __future__ import annotations

import os
import re
import sys
from collections import deque
from pathlib import Path

import numpy as np


def _span_hz(stamps: deque[float], now: float, window_s: float) -> float:
    """Rate inside a window whose right edge is ``now``, not the last stamp."""
    while stamps and now - stamps[0] > window_s:
        stamps.popleft()
    if len(stamps) < 2:
        return 0.0
    end = now if now > stamps[-1] else stamps[-1]
    span = end - stamps[0]
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
        """Arrival rate on the wall clock. The right edge is ``now``."""
        return _span_hz(self.times, now, self.window_s)

    def hz_sim(self, now_sim: float) -> float:
        """Header-stamp rate. ``now_sim`` is ``/clock``, so a gap lowers the rate."""
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


def stale_limit_s(minimum_hz: float) -> float:
    """Silence allowed before a topic is stale: three periods, and at least 0.5 s."""
    period = (1.0 / minimum_hz) if minimum_hz > 0.0 else 0.5
    return max(3.0 * period, 0.5)


def topic_rate_line(
    topic: str,
    tracker: RateTracker | None,
    now_wall: float,
    now_sim: float | None,
    minimum: float,
    rtf: float | None,
) -> tuple[bool, str]:
    """Sim-time rate over a window that ends at ``/clock``, or a stale failure.

    The window's right edge is the current sim time. A topic whose newest
    header is older than :func:`stale_limit_s` fails even when the samples
    it did publish were fast.
    """
    wall_hz = tracker.hz_wall(now_wall) if tracker is not None else 0.0
    sim_hz = None
    if tracker is not None and tracker.sim_times and now_sim is not None:
        age = now_sim - tracker.sim_times[-1]
        if age > stale_limit_s(minimum):
            return False, line(False, f"{topic}: stale: last message {age:.3f} s sim ago")
        if len(tracker.sim_times) >= 2:
            sim_hz = tracker.hz_sim(now_sim)
    elif tracker is not None and len(tracker.sim_times) >= 2:
        sim_hz = tracker.hz_sim(tracker.sim_times[-1])
    return check_rate(topic, gated_rate_hz(sim_hz, wall_hz, rtf), wall_hz, minimum)


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
    """Camera rate from ``CAM_RATE_HZ`` or ``VISION_PROFILE``, or the IMU rate.

    ``full`` is 30 Hz cameras. ``cpu`` is 10 Hz cameras. With no profile the
    camera rate is 30 Hz. The IMU rate is :func:`expected_imu_hz`.
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
    return expected_imu_hz(env)


def _imu_source(env: dict[str, str]) -> str:
    raw = env.get("IMU_SOURCE", "oak").strip().lower()
    return "oak" if raw in {"", "oak"} else raw


def _load_geometry():
    """Import ``geometry`` from the path, or from ``includes/gz/oakd_s2``.

    That file arrives with the vision-profile work. A missing module returns
    None so this branch still resolves the IMU rate from the model.
    """
    try:
        import geometry
        return geometry
    except ImportError:
        pass
    oakd = Path(__file__).resolve().parents[4] / "oakd_s2"
    if not (oakd / "geometry.py").is_file():
        return None
    entry = str(oakd)
    if entry not in sys.path:
        sys.path.insert(0, entry)
    try:
        import geometry
    except ImportError:
        return None
    return geometry


def _profile_imu_hz(env: dict[str, str]) -> float | None:
    """``geometry.profile_from_env().imu_hz`` when that module can be imported."""
    geometry = _load_geometry()
    if geometry is None:
        return None
    try:
        profile = geometry.profile_from_env(env)
    except (TypeError, ValueError):
        return None
    imu_hz = getattr(profile, "imu_hz", None)
    if imu_hz is None:
        return None
    return float(imu_hz)


_IMU_UPDATE_RATE = re.compile(
    r"<sensor\b[^>]*\btype=[\"']imu[\"'][^>]*>.*?<update_rate>\s*([0-9.]+)\s*</update_rate>",
    re.DOTALL | re.IGNORECASE,
)


def _oak_model_imu_hz(env: dict[str, str]) -> float | None:
    """IMU ``update_rate`` from the Oak-D model this stack bridges."""
    camera = env.get("CameraType", "rgbd").strip().lower()
    name = "OakD-Lite-stereo" if camera == "stereo" else "OakD-Lite-rgbd"
    path = Path(__file__).resolve().parents[4] / "models" / name / "model.sdf"
    if not path.is_file():
        return None
    match = _IMU_UPDATE_RATE.search(path.read_text(encoding="utf-8"))
    if match is None:
        return None
    return float(match.group(1))


def _resolve_imu_hz(env: dict[str, str]) -> tuple[float, str]:
    """Expected IMU rate and which rule supplied it.

    Precedence: ``IMU_RATE_HZ``, the vision profile ``imu_hz`` when
    ``geometry`` imports, the bridged model's SDF ``update_rate``, then
    50 Hz for oak or 80 Hz for px4.
    """
    override = _env_float(env, "IMU_RATE_HZ")
    if override is not None:
        return override, "rate"
    profile = _profile_imu_hz(env)
    if profile is not None:
        return profile, "profile"
    if _imu_source(env) != "px4":
        model = _oak_model_imu_hz(env)
        if model is not None:
            return model, "model"
        return 50.0, "fallback"
    return 80.0, "fallback"


def expected_imu_hz(environ: dict[str, str] | None = None) -> float:
    """Expected IMU rate in Hz. The preflight minimum is half of this.

    A vision-profile rate is the exception: that ``imu_hz`` is the minimum,
    so a full profile of 200 Hz rejects a 100 Hz sim-time stream.
    """
    env = os.environ if environ is None else environ
    return _resolve_imu_hz(env)[0]


def minimum_rate_hz(kind: str, environ: dict[str, str] | None = None) -> float:
    """Preflight minimum. ``PREFLIGHT_MIN_*_HZ`` wins, else half the expected rate.

    The vision profile's ``imu_hz`` is used whole. Half of 200 Hz is 100 Hz,
    and a 100 Hz sim-time IMU must fail that profile. Model and fallback
    rates are halved, so the oak minimum is never a fixed 50 Hz.
    """
    env = os.environ if environ is None else environ
    name = "PREFLIGHT_MIN_CAMERA_HZ" if kind == "camera" else "PREFLIGHT_MIN_IMU_HZ"
    override = _env_float(env, name)
    if override is not None:
        return override
    if kind == "imu":
        hz, source = _resolve_imu_hz(env)
        if source == "profile":
            return hz
        return 0.5 * hz
    return 0.5 * expected_sensor_hz(kind, env)


def default_min_rtf(environ: dict[str, str] | None = None) -> float:
    """0.15 under software rendering, otherwise 0.8. PREFLIGHT_MIN_RTF overrides."""
    env = os.environ if environ is None else environ
    override = _env_float(env, "PREFLIGHT_MIN_RTF")
    if override is not None:
        return override
    software = env.get("HEADLESS_SOFTWARE", "0").strip().lower()
    # Software GL on a CPU sim sits near 0.2; 0.8 is the floor with a GPU.
    if software in {"1", "true", "yes"}:
        return 0.15
    return 0.8


def line(ok: bool, text: str) -> str:
    """``PASS`` or ``FAIL`` followed by the reason text."""
    return f"{'PASS' if ok else 'FAIL'} {text}"


# PX4 spawns the airframe as ``<model>_<instance>``. The start script's model
# is x500_depth and the first instance is 0, so GZBridge listens on
# /model/x500_depth_0/odometry_with_covariance.
DEFAULT_PX4_VISION_MODEL = "x500_depth_0"
GROUND_TRUTH_LEAK_CHECK = "no ground-truth leak to PX4 vision"


def px4_vision_model_name(environ: dict[str, str] | None = None) -> str:
    """Spawned Gazebo model name on PX4's external-vision topic.

    ``PX4_GZ_MODEL`` is the airframe (this repo uses ``x500_depth``).
    ``PX4_SIM_MODEL`` is the same name with an optional ``gz_`` prefix.
    PX4 appends ``_<PX4_INSTANCE>`` (default 0). A value that already ends
    with that suffix is the spawned entity name.
    """
    env = os.environ if environ is None else environ
    raw = env.get("PX4_GZ_MODEL", "").strip() or env.get("PX4_SIM_MODEL", "").strip()
    if raw.startswith("gz_"):
        raw = raw[3:]
    instance = env.get("PX4_INSTANCE", "").strip() or env.get("px4_instance", "").strip() or "0"
    if not raw:
        raw = "x500_depth"
    suffix = f"_{instance}"
    if raw.endswith(suffix):
        return raw
    return f"{raw}{suffix}"


def px4_vision_covariance_topic(model_name: str | None = None, environ: dict[str, str] | None = None) -> str:
    """Topic GZBridge subscribes to and forwards to EKF2 as external vision."""
    name = model_name if model_name is not None else px4_vision_model_name(environ)
    return f"/model/{name}/odometry_with_covariance"


def gz_topic_publishers(info_text: str) -> list[str]:
    """Publisher lines from ``gz topic -i`` output.

    Subscriber addresses are ignored. ``No publishers`` is an empty list.
    """
    publishers: list[str] = []
    in_publishers = False
    for raw in info_text.splitlines():
        stripped = raw.strip()
        lower = stripped.lower()
        if lower.startswith("publishers"):
            in_publishers = True
            continue
        if lower.startswith("subscribers"):
            in_publishers = False
            continue
        if not in_publishers or not stripped:
            continue
        if lower == "none" or lower.startswith("no publisher"):
            continue
        publishers.append(stripped)
    return publishers


def ground_truth_leak_from_info(
    info_text: str,
    *,
    gz_cli: bool,
    model_name: str | None = None,
    returncode: int = 0,
) -> tuple[bool, str]:
    """PASS, FAIL, or SKIP for the ground-truth vision-topic leak.

    ``gz_cli`` false is SKIP: the check did not run. A publisher on
    ``/model/<model>/odometry_with_covariance`` is FAIL because that pose
    enters EKF2 as external vision.
    """
    model = model_name or DEFAULT_PX4_VISION_MODEL
    topic = px4_vision_covariance_topic(model)
    name = GROUND_TRUTH_LEAK_CHECK
    if not gz_cli:
        return True, f"SKIP {name}: gz CLI is absent"
    publishers = gz_topic_publishers(info_text)
    if publishers:
        shown = publishers[0]
        extra = f" ({len(publishers)} publishers)" if len(publishers) > 1 else ""
        return False, (
            f"FAIL {name}: {topic} has a publisher {shown}{extra}; "
            "ground-truth pose would enter EKF2 as external vision"
        )
    if returncode != 0 and not re.search(r"publishers", info_text, re.IGNORECASE):
        detail = " ".join(info_text.split())
        if len(detail) > 180:
            detail = detail[:180] + "..."
        suffix = f": {detail}" if detail else ""
        return False, f"FAIL {name}: gz topic -i -t {topic} failed{suffix}"
    return True, f"PASS {name}: {topic} has no publisher"


def check_min(name: str, value: float, minimum: float, unit: str) -> tuple[bool, str]:
    """Pass when ``value`` is at least ``minimum``."""
    ok = value >= minimum
    return ok, line(ok, f"{name}: {value:.3f} {unit} >= {minimum:.3f} {unit}")


RTF_CPU_HINT = (
    "on CPU-only machines set VISION_PROFILE=cpu "
    "(measured ~0.45 vs 0.125 for full on apt_world)"
)


def with_rtf_hint(ok: bool, text: str, environ: dict[str, str] | None = None) -> str:
    """Append the CPU-profile hint to a failed real-time-factor line.

    The hint is omitted when the check passes, and when ``VISION_PROFILE``
    is already ``cpu``. Unset and ``full`` both get it. The result stays one line.
    """
    env = os.environ if environ is None else environ
    if ok or env.get("VISION_PROFILE", "").strip().lower() == "cpu":
        return text
    return f"{text.rstrip()}; {RTF_CPU_HINT}"


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
    """Pass when ``value`` is at most ``maximum``."""
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


def lidar_down_enabled(environ: dict[str, str] | None = None) -> bool:
    """True unless ``LIDAR_DOWN`` is 0, false, or no. Unset means on."""
    env = os.environ if environ is None else environ
    raw = env.get("LIDAR_DOWN", "1").strip().lower()
    if raw == "":
        raw = "1"
    return raw in {"1", "true", "yes"}


def distance_sensor_topic(names: list[str]) -> str | None:
    """``/fmu/out/distance_sensor`` or the highest ``distance_sensor_vN`` on that prefix.

    ``/fmu/in/distance_sensor`` is PX4's subscription and is ignored.
    """
    published = [name for name in names if "/fmu/out/" in name]
    return match_versioned_topic(published, "distance_sensor")


def distance_sensor_result(
    *,
    enabled: bool,
    topic: str | None,
    distance_m: float | None,
) -> tuple[bool, str] | None:
    """At-rest downward range, or None when ``LIDAR_DOWN`` is off.

    A missing ``/fmu/out/distance_sensor`` is SKIP. PX4 v1.17's
    ``dds_topics.yaml`` does not publish that topic. A reading passes when
    it is within 0.1 m to 0.5 m.
    """
    if not enabled:
        return None
    if not topic:
        return True, (
            "SKIP distance_sensor: /fmu/out/distance_sensor is not in the ROS graph "
            "(PX4 v1.17 dds_topics.yaml does not publish it)"
        )
    if distance_m is None:
        return False, line(False, f"distance_sensor: {topic} has no message yet")
    ok = 0.1 <= distance_m <= 0.5
    relation = "within" if ok else "outside"
    return ok, line(ok, f"distance_sensor: {distance_m:.3f} m {relation} 0.1-0.5 m")


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
    """Join check lines. Success is true when every check passed.

    A line that starts with SKIP did not run. It does not fail the service.
    """
    success = all(ok or text.startswith("SKIP") for ok, text in results) and len(results) > 0
    message = "\n".join(text for _ok, text in results)
    return success, message
