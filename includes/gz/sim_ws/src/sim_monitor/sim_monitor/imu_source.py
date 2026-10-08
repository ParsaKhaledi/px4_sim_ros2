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


RECEIVE = "receive"
PX4_OFFSET = "px4_offset"
# RTAB-Map's IMU path wants 100 Hz or more. The relay warns under this.
PX4_RATE_WARN_HZ = 100.0
DEFAULT_YAW_VARIANCE = 1.0e3


def normalize_stamp_mode(value: str | None) -> str:
    """``IMU_STAMP_MODE``. Empty means receive time."""
    raw = RECEIVE if value is None or str(value).strip() == "" else str(value).strip().lower()
    if raw not in {RECEIVE, PX4_OFFSET}:
        raise ValueError(f"IMU_STAMP_MODE must be receive or px4_offset, got {value!r}")
    return raw


def stamp_mode(environ: dict[str, str] | None = None) -> str:
    env = os.environ if environ is None else environ
    return normalize_stamp_mode(env.get("IMU_STAMP_MODE", ""))


def px4_sample_us(timestamp_us: int, timestamp_sample_us: int | None = None) -> int:
    """Microseconds of the IMU sample.

    ``sensor_combined`` in px4_msgs v1.17 has ``timestamp`` only, and that
    field is the gyro sample time. ``timestamp_sample`` is used when the
    message has it (``vehicle_attitude`` does).
    """
    if timestamp_sample_us is not None:
        return int(timestamp_sample_us)
    return int(timestamp_us)


def measure_px4_offset_ns(sample_us: int, clock_ns: int) -> int:
    """ROS clock minus the PX4 sample time, both in nanoseconds. Measured once."""
    return int(clock_ns) - int(sample_us) * 1000


def imu_stamp_ns(
    mode: str,
    receive_ns: int,
    sample_us: int | None = None,
    offset_ns: int | None = None,
) -> int:
    """Stamp on the ROS clock.

    The default, ``receive``, is the node clock at arrival. With
    ``use_sim_time`` that clock is Gazebo's ``/clock``.

    PX4's ``timestamp`` is microseconds since PX4 boot. Publishing that raw
    value does not match image stamps on ``/clock``. RTAB-Map's
    ``wait_imu_to_init`` then drops the IMU samples. ``px4_offset`` does not
    publish the boot timestamp either. It adds one fixed offset, measured at
    the first sample as ``/clock`` minus the sample time. SITL lockstep keeps
    PX4's clock on Gazebo time, so that constant stays valid.
    """
    selected = normalize_stamp_mode(mode)
    if selected == RECEIVE or offset_ns is None or sample_us is None:
        return int(receive_ns)
    return int(sample_us) * 1000 + int(offset_ns)


def diagonal_covariance(stddev: float) -> list[float]:
    """9-element row-major covariance with ``stddev**2`` on the diagonal."""
    variance = float(stddev) * float(stddev)
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


def orientation_covariance(roll_pitch_stddev: float, yaw_variance: float = DEFAULT_YAW_VARIANCE) -> list[float]:
    """Roll and pitch variance on the diagonal, and a large yaw variance.

    The matrix is row-major about x, y, z. Z is yaw in the ENU orientation.
    The default yaw variance is 1e3 rad² so a consumer can use the quaternion
    for gravity and ignore heading.
    """
    variance = float(roll_pitch_stddev) * float(roll_pitch_stddev)
    yaw = float(yaw_variance)
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, yaw]


def input_rate_hz(sim_stamps_ns: list[int]) -> float | None:
    """Rate from ROS-clock stamps. ``None`` until two stamps span a positive time."""
    if len(sim_stamps_ns) < 2:
        return None
    span_s = (int(sim_stamps_ns[-1]) - int(sim_stamps_ns[0])) * 1e-9
    if span_s <= 0.0:
        return None
    return (len(sim_stamps_ns) - 1) / span_s


def rate_log(rate_hz: float | None, warn_below: float = PX4_RATE_WARN_HZ) -> tuple[str, str] | None:
    """``(info|warn, text)`` for the measured sensor_combined rate."""
    if rate_hz is None:
        return None
    text = f"sensor_combined {rate_hz:.1f} Hz sim"
    if rate_hz < warn_below:
        return ("warn", f"{text}, below {warn_below:.0f} Hz")
    return ("info", text)
