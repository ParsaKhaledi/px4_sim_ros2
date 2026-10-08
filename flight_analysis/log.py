"""Load a PX4 ULog into plain arrays. Tests build the same objects themselves."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np

from flight_analysis.frames import tilt_rad_from_quaternion
from flight_analysis.frames import yaw_from_ned_quaternion


class LogError(RuntimeError):
    """The log is missing, unreadable, or has no trajectory to grade."""


@dataclass
class Series:
    """One logged topic. ``time_s`` is seconds of sim time."""

    time_s: np.ndarray
    fields: dict[str, np.ndarray]


@dataclass
class FlightLog:
    """Topics and initial parameters from one flight."""

    series: dict[str, Series]
    parameters: dict[str, float]


@dataclass
class Track:
    """A trajectory in NED, with yaw in radians from north toward east."""

    time_s: np.ndarray
    north_m: np.ndarray
    east_m: np.ndarray
    down_m: np.ndarray
    yaw_rad: np.ndarray
    vn_m_s: np.ndarray | None = None
    ve_m_s: np.ndarray | None = None
    vd_m_s: np.ndarray | None = None


class LogLoader(Protocol):
    """Anything that can turn a log path into a ``FlightLog``."""

    def load(self, path: Path) -> FlightLog:
        """Load one flight log from disk."""


class ULogLoader:
    """Read a ``.ulg`` file through pyulog."""

    def load(self, path: Path) -> FlightLog:
        """Load topics and the initial parameter set from a ULog."""

        path = Path(path)
        if not path.is_file():
            raise LogError(f"ULog not found: {path}")
        try:
            from pyulog import ULog
        except ImportError as exc:
            raise LogError(
                "pyulog is not installed; see flight_analysis/requirements.txt"
            ) from exc
        try:
            ulog = ULog(str(path))
        except Exception as exc:
            raise LogError(f"could not read ULog {path}: {exc}") from exc
        topics: dict[str, dict[str, np.ndarray]] = {}
        for data in ulog.data_list:
            if getattr(data, "multi_id", 0) not in (0, None):
                continue
            if data.name in topics:
                continue
            topics[data.name] = {key: np.asarray(value) for key, value in data.data.items()}
        parameters = getattr(ulog, "initial_parameters", {}) or {}
        return flight_log_from_arrays(dict(parameters), topics)


def flight_log_from_arrays(
    parameters: dict[str, object],
    topics: dict[str, dict[str, np.ndarray]],
) -> FlightLog:
    """Build a ``FlightLog`` from decoded arrays.

    Each topic needs ``timestamp`` in sim-time microseconds, which is the
    clock PX4 writes into a ULog. Tests call this instead of pyulog.
    """

    series: dict[str, Series] = {}
    for name, fields in topics.items():
        if "timestamp" not in fields:
            continue
        time_s = np.asarray(fields["timestamp"], dtype=np.float64) * 1e-6
        order = np.argsort(time_s, kind="stable")
        cleaned: dict[str, np.ndarray] = {}
        for key, value in fields.items():
            if key == "timestamp":
                continue
            array = np.asarray(value)
            if array.shape[0] != time_s.shape[0]:
                continue
            cleaned[key] = array[order]
        series[name] = Series(time_s=time_s[order], fields=cleaned)
    parsed: dict[str, float] = {}
    for key, value in parameters.items():
        try:
            parsed[str(key)] = float(value)
        except (TypeError, ValueError):
            continue
    return FlightLog(series=series, parameters=parsed)


def command_track(log: FlightLog) -> Track:
    """Return the commanded setpoint, preferring ``trajectory_setpoint``."""

    series = log.series.get("trajectory_setpoint")
    if series is None:
        series = log.series.get("vehicle_local_position_setpoint")
    if series is None:
        raise LogError(
            "log has no trajectory_setpoint or vehicle_local_position_setpoint"
        )
    north, east, down = _position(series)
    yaw = _first(series, "yaw", "heading")
    return _track(series.time_s, north, east, down, yaw, None, None, None)


def estimate_track(log: FlightLog) -> Track:
    """Return the EKF2 local position (``vehicle_local_position``)."""

    series = log.series.get("vehicle_local_position")
    if series is None:
        raise LogError("log has no vehicle_local_position")
    north, east, down = _position(series)
    yaw = _first(series, "heading", "yaw")
    if yaw is None:
        yaw = _yaw_from_attitude(log, "vehicle_attitude")
    return _track(
        series.time_s,
        north,
        east,
        down,
        yaw,
        _first(series, "vx"),
        _first(series, "vy"),
        _first(series, "vz"),
    )


def ground_truth_track(log: FlightLog) -> Track | None:
    """Return logged ground truth, or ``None`` when those topics are absent."""

    series = log.series.get("vehicle_local_position_groundtruth")
    if series is None:
        return None
    north, east, down = _position(series)
    yaw = _first(series, "heading", "yaw")
    if yaw is None:
        yaw = _yaw_from_attitude(log, "vehicle_attitude_groundtruth")
    return _track(
        series.time_s,
        north,
        east,
        down,
        yaw,
        _first(series, "vx"),
        _first(series, "vy"),
        _first(series, "vz"),
    )


def tilt_series(log: FlightLog) -> tuple[np.ndarray, np.ndarray] | None:
    """Return ``(time_s, tilt_rad)`` from ``vehicle_attitude``, if logged."""

    series = log.series.get("vehicle_attitude")
    if series is None:
        return None
    quat = _quaternion(series)
    if quat is None:
        return None
    qw, qx, qy, qz = quat
    tilt = np.array(
        [tilt_rad_from_quaternion(w, x, y, z) for w, x, y, z in zip(qw, qx, qy, qz, strict=True)],
        dtype=np.float64,
    )
    return series.time_s, tilt


def rate_series(log: FlightLog) -> tuple[np.ndarray, np.ndarray] | None:
    """Return ``(time_s, Nx3 body rates in rad/s)`` when gyro rates are logged."""

    series = log.series.get("vehicle_angular_velocity")
    if series is None:
        return None
    axes = _xyz(series, "xyz")
    if axes is None:
        return None
    return series.time_s, axes


def actuator_series(log: FlightLog) -> tuple[str, np.ndarray, np.ndarray] | None:
    """Return motor commands as ``(kind, time_s, NxM)``.

    ``kind`` is ``normalized`` for ``actuator_motors`` (0 idle, 1 full) or
    ``pwm`` for ``actuator_outputs`` in microseconds.
    """

    motors = log.series.get("actuator_motors")
    if motors is not None:
        control = _indexed(motors, "control")
        if control is not None:
            return "normalized", motors.time_s, control
    outputs = log.series.get("actuator_outputs")
    if outputs is None:
        return None
    values = _indexed(outputs, "output")
    if values is None:
        return None
    return "pwm", outputs.time_s, values


def flag_series(log: FlightLog) -> Series | None:
    """Return ``estimator_status_flags`` when the log has it."""

    return log.series.get("estimator_status_flags")


def vision_delay_seconds(log: FlightLog) -> np.ndarray | None:
    """Return ``timestamp - timestamp_sample`` for visual odometry, in seconds.

    Both stamps are sim-time microseconds. The difference is how long the
    vision sample waited before PX4 integrated it.
    """

    series = log.series.get("vehicle_visual_odometry")
    if series is None:
        return None
    sample = series.fields.get("timestamp_sample")
    if sample is None:
        return None
    # ``time_s`` was already converted; the sample stamp is still microseconds.
    return series.time_s - np.asarray(sample, dtype=np.float64) * 1e-6


def parameter(log: FlightLog, name: str) -> float | None:
    """Return one initial parameter, or ``None`` when it was not logged."""

    if name not in log.parameters:
        return None
    return float(log.parameters[name])


def _track(
    time_s: np.ndarray,
    north: np.ndarray,
    east: np.ndarray,
    down: np.ndarray,
    yaw: np.ndarray | None,
    vn: np.ndarray | None,
    ve: np.ndarray | None,
    vd: np.ndarray | None,
) -> Track:
    """Forward-fill NaN commands and drop samples that still have no position."""

    north = _fill_hold(north)
    east = _fill_hold(east)
    down = _fill_hold(down)
    if yaw is None:
        yaw = np.zeros(time_s.shape[0], dtype=np.float64)
    else:
        yaw = _fill_hold(np.asarray(yaw, dtype=np.float64))
    valid = np.isfinite(north) & np.isfinite(east) & np.isfinite(down)
    if not np.any(valid):
        raise LogError("setpoint or position samples are all NaN")
    yaw = np.where(np.isfinite(yaw), yaw, 0.0)

    def _cut(values: np.ndarray | None) -> np.ndarray | None:
        """Keep the finite-position rows of one optional velocity column."""

        if values is None:
            return None
        return np.asarray(values, dtype=np.float64)[valid]

    return Track(
        time_s=time_s[valid],
        north_m=_cut(north),
        east_m=_cut(east),
        down_m=_cut(down),
        yaw_rad=_cut(yaw),
        vn_m_s=_cut(vn) if vn is not None else None,
        ve_m_s=_cut(ve) if ve is not None else None,
        vd_m_s=_cut(vd) if vd is not None else None,
    )


def _position(series: Series) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Read a NED position as north, east, down."""

    indexed = _indexed(series, "position")
    if indexed is not None and indexed.shape[1] >= 3:
        return indexed[:, 0], indexed[:, 1], indexed[:, 2]
    north = _first(series, "x", "north")
    east = _first(series, "y", "east")
    down = _first(series, "z", "down")
    if north is None or east is None or down is None:
        raise LogError("position topic has no x/y/z or position[0..2]")
    return north, east, down


def _first(series: Series, *names: str) -> np.ndarray | None:
    """Return the first present field as a float array."""

    for name in names:
        if name in series.fields:
            return np.asarray(series.fields[name], dtype=np.float64)
    return None


def _indexed(series: Series, prefix: str) -> np.ndarray | None:
    """Stack ``prefix[0]``, ``prefix[1]``, ... into an ``NxM`` array."""

    columns: list[np.ndarray] = []
    index = 0
    while True:
        for key in (f"{prefix}[{index}]", f"{prefix}_{index}"):
            if key in series.fields:
                columns.append(np.asarray(series.fields[key], dtype=np.float64))
                break
        else:
            break
        index += 1
    if not columns:
        return None
    return np.column_stack(columns)


def _xyz(series: Series, prefix: str) -> np.ndarray | None:
    """Return a 3-column body-rate array, or ``None`` when it is absent."""

    stacked = _indexed(series, prefix)
    if stacked is None or stacked.shape[1] < 3:
        return None
    return stacked[:, :3]


def _quaternion(series: Series) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None:
    """Return ``(w, x, y, z)`` from ``q[0..3]``. PX4 stores ``w`` first."""

    stacked = _indexed(series, "q")
    if stacked is None or stacked.shape[1] < 4:
        return None
    return stacked[:, 0], stacked[:, 1], stacked[:, 2], stacked[:, 3]


def _yaw_from_attitude(log: FlightLog, topic: str) -> np.ndarray | None:
    """Yaw from an attitude topic, or ``None`` when that topic is absent."""

    series = log.series.get(topic)
    if series is None:
        return None
    quat = _quaternion(series)
    if quat is None:
        return None
    qw, qx, qy, qz = quat
    return np.array(
        [yaw_from_ned_quaternion(w, x, y, z) for w, x, y, z in zip(qw, qx, qy, qz, strict=True)],
        dtype=np.float64,
    )


def _fill_hold(values: np.ndarray) -> np.ndarray:
    """Replace NaN with the previous finite sample.

    PX4 writes NaN in a setpoint component to mean "leave this axis alone",
    so the last real command is the one the vehicle is still flying.
    """

    out = np.asarray(values, dtype=np.float64).copy()
    last = np.nan
    for index, value in enumerate(out):
        if np.isfinite(value):
            last = value
        else:
            out[index] = last
    return out
