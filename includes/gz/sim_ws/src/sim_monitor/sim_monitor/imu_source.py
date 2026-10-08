"""Which node publishes ROS ``/imu``.

``IMU_SOURCE=oak`` (the default) bridges the Oak-D Gazebo IMU.
``IMU_SOURCE=px4`` relays PX4 ``sensor_combined`` and ``vehicle_attitude``.
Only one of those publishers is configured.
"""

from __future__ import annotations

import os

from sim_monitor.checks import expected_imu_hz  # noqa: F401  re-exported for callers

OAK = "oak"
PX4 = "px4"
IMU_TOPIC = "/imu"
OAK_FRAME = "imu_link"
PX4_FRAME = "base_link"


def normalize_imu_source(value: str | None) -> str:
    """Return ``oak`` or ``px4``. Empty means oak."""
    raw = "oak" if value is None or str(value).strip() == "" else str(value).strip().lower()
    if raw not in {OAK, PX4}:
        raise ValueError(f"IMU_SOURCE must be oak or px4, got {value!r}")
    return raw


def imu_source(environ: dict[str, str] | None = None) -> str:
    env = os.environ if environ is None else environ
    return normalize_imu_source(env.get("IMU_SOURCE", ""))


def configured_imu_publishers(source: str | None = None) -> tuple[str, ...]:
    """The single ``/imu`` publisher for this mode."""
    selected = normalize_imu_source(source)
    if selected == OAK:
        return ("ros_gz_bridge",)
    return ("px4_imu_relay",)


def bridge_gazebo_imu(source: str | None = None) -> bool:
    """True only in oak mode. The px4 relay owns ``/imu`` otherwise."""
    return normalize_imu_source(source) == OAK


def default_tf_pairs(environ: dict[str, str] | None = None) -> str:
    """Preflight TF pairs. The camera frame follows ``CameraType``."""
    env = os.environ if environ is None else environ
    chosen = env.get("PREFLIGHT_TF_PAIRS", "").strip()
    if chosen:
        return chosen
    camera = env.get("CameraType", "rgbd").strip().lower()
    camera_frame = "stereo_left_camera_frame" if camera == "stereo" else "camera_rgb_frame"
    return f"world:spawn,world:base_link_gt,base_link:{camera_frame},base_link:imu_link"


def optical_frame(environ: dict[str, str] | None = None) -> str:
    """Camera optical frame the IMU must be able to reach through TF."""
    env = os.environ if environ is None else environ
    chosen = env.get("PREFLIGHT_OPTICAL_FRAME", "").strip()
    if chosen:
        return chosen
    camera = env.get("CameraType", "rgbd").strip().lower()
    if camera == "stereo":
        return "stereo_left_camera_optical_frame"
    return "camera_rgb_optical_frame"


def diagonal_covariance(stddev: float) -> list[float]:
    """9-element row-major covariance with ``stddev**2`` on the diagonal."""
    variance = float(stddev) * float(stddev)
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


def ros_stamp_ns(
    px4_timestamp_us: int,
    offset_us: int | None,
    receive_ns: int,
    max_skew_ns: int = 2_000_000_000,
) -> int:
    """ROS stamp, in nanoseconds, on the same clock as ``receive_ns``.

    ``/fmu/out/timesync_status.estimated_offset`` is the smoothed
    companion-minus-PX4 offset in microseconds (PX4 v1.17 publishes that
    topic by default). Adding it to the sample timestamp gives the companion
    clock. In SITL the agent clock is the host wall clock, not Gazebo, so a
    converted stamp that sits more than ``max_skew_ns`` from the receive time
    is discarded. The receive time is the node clock: sim time when
    ``use_sim_time`` is set, which is the clock RTAB-Map already uses.
    Until the first timesync sample, the stamp is the receive time.
    """
    if offset_us is None:
        return int(receive_ns)
    converted = (int(px4_timestamp_us) + int(offset_us)) * 1000
    if abs(converted - int(receive_ns)) > int(max_skew_ns):
        return int(receive_ns)
    return converted
