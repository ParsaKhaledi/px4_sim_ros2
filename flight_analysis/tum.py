"""TUM ground truth: spawn frame, ENU to NED, then the ROS clock onto the ULog."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from flight_analysis.frames import enu_flu_to_ned_frd
from flight_analysis.frames import world_enu_to_spawn_enu
from flight_analysis.log import Track


# A matching climb of a couple of seconds is not enough: the hover and the
# move have to sit inside both traces before the offset is trusted.
MIN_OVERLAP_S = 5.0
# Below this the climb shapes disagree and the offset is a coincidence.
MIN_CORRELATION = 0.5
# Already on the same clock when the zero-lag match is this good.
ALIGNED_CORRELATION = 0.8


class TimeAlignmentError(RuntimeError):
    """The TUM file and the ULog do not share enough of the same flight."""


@dataclass(frozen=True)
class TimeAlignment:
    """How a TUM clock was placed onto the ULog clock.

    ``offset_s`` is added to each TUM timestamp. Both clocks are sim time.
    When the offset comes from ``px4_offset_s`` (ROS sim time minus PX4
    boot time), ``offset_s = -px4_offset_s``, because
    ``t_px4 = t_ros - px4_offset_s``.
    """

    offset_s: float
    correlation: float
    method: str
    overlap_s: float


def load_tum(
    path: Path,
    spawn_xyz: tuple[float, float, float],
    spawn_yaw_rad: float,
) -> Track:
    """Read a TUM file, move it into the spawn frame, and convert to NED/FRD.

    Each data line is ``timestamp tx ty tz qx qy qz qw`` with time in
    seconds of ROS sim time. Position is Gazebo world ENU. ``spawn_xyz``
    and ``spawn_yaw_rad`` are the takeoff pose in that same world; they
    are not optional. Passing the origin would silently treat world ENU
    as the PX4 local frame.
    """

    times: list[float] = []
    north: list[float] = []
    east: list[float] = []
    down: list[float] = []
    yaw: list[float] = []
    for line_number, raw in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), start=1):
        text = raw.strip()
        if not text or text.startswith("#"):
            continue
        parts = text.split()
        if len(parts) < 8:
            raise TimeAlignmentError(f"{path}:{line_number} is not a TUM pose")
        try:
            stamp = float(parts[0])
            values = [float(part) for part in parts[1:8]]
            local = world_enu_to_spawn_enu(*values, spawn_xyz, spawn_yaw_rad)
            pose = enu_flu_to_ned_frd(*local)
        except ValueError as exc:
            raise TimeAlignmentError(f"{path}:{line_number} is not a TUM pose") from exc
        times.append(stamp)
        north.append(pose.north_m)
        east.append(pose.east_m)
        down.append(pose.down_m)
        yaw.append(pose.yaw_rad())
    if len(times) < 2:
        raise TimeAlignmentError(f"{path} has fewer than two poses")
    order = np.argsort(np.asarray(times))
    return Track(
        time_s=np.asarray(times, dtype=np.float64)[order],
        north_m=np.asarray(north, dtype=np.float64)[order],
        east_m=np.asarray(east, dtype=np.float64)[order],
        down_m=np.asarray(down, dtype=np.float64)[order],
        yaw_rad=np.asarray(yaw, dtype=np.float64)[order],
    )


def align_tum_to_ulog(ulog: Track, tum: Track) -> TimeAlignment:
    """Estimate the seconds to add to TUM time so it matches the ULog.

    Up-velocity during the climb is the shared feature. A constant height
    bias does not move the peak, which a position correlation would.
    """

    t_ref, v_ref = _up_velocity(ulog)
    t_other, v_other = _up_velocity(tum)
    corr0, overlap0 = _correlation(t_ref, v_ref, t_other, v_other, 0.0)
    if overlap0 >= MIN_OVERLAP_S and corr0 >= ALIGNED_CORRELATION:
        return TimeAlignment(0.0, corr0, "already_aligned", overlap0)

    offset = _fft_offset(t_ref, v_ref, t_other, v_other)
    correlation, overlap = _correlation(t_ref, v_ref, t_other, v_other, offset)
    if overlap < MIN_OVERLAP_S:
        raise TimeAlignmentError(
            f"TUM ground truth overlaps the ULog for {overlap:.2f} s after alignment "
            f"(offset {offset:.3f} s); need at least {MIN_OVERLAP_S:.0f} s"
        )
    if correlation < MIN_CORRELATION:
        raise TimeAlignmentError(
            f"vertical-velocity correlation peaked at {correlation:.2f} "
            f"(offset {offset:.3f} s), below {MIN_CORRELATION:.2f}; "
            "the TUM file does not match this flight's climb"
        )
    if abs(offset) <= 0.1 and overlap0 >= MIN_OVERLAP_S and corr0 >= correlation - 0.02:
        return TimeAlignment(0.0, corr0, "already_aligned", overlap0)
    return TimeAlignment(float(offset), float(correlation), "vertical_velocity", float(overlap))


def alignment_from_px4_offset(ulog: Track, tum: Track, px4_offset_s: float) -> TimeAlignment:
    """Place TUM time on the ULog clock with a known ``px4_offset_s``.

    The convention is ROS sim time minus PX4 boot time:

        px4_offset_s = t_ros - t_px4
        t_px4 = t_ros - px4_offset_s

    ``TimeAlignment.offset_s`` is what gets added to each TUM timestamp, so
    it is ``-px4_offset_s``. A positive ``px4_offset_s`` means the ROS
    clock is ahead of PX4 boot time and TUM stamps move backward.
    """

    applied = -float(px4_offset_s)
    t_ref, v_ref = _up_velocity(ulog)
    t_other, v_other = _up_velocity(tum)
    correlation, overlap = _correlation(t_ref, v_ref, t_other, v_other, applied)
    if overlap < MIN_OVERLAP_S:
        raise TimeAlignmentError(
            f"TUM ground truth overlaps the ULog for {overlap:.2f} s "
            f"with px4_offset_s={px4_offset_s:.3f} "
            f"(TUM timestamps shifted by {applied:.3f} s); "
            f"need at least {MIN_OVERLAP_S:.0f} s"
        )
    return TimeAlignment(applied, float(correlation), "explicit", float(overlap))


def shift_track(track: Track, offset_s: float) -> Track:
    """Return a copy of ``track`` with ``offset_s`` added to every timestamp."""

    return Track(
        time_s=track.time_s + offset_s,
        north_m=track.north_m.copy(),
        east_m=track.east_m.copy(),
        down_m=track.down_m.copy(),
        yaw_rad=track.yaw_rad.copy(),
        vn_m_s=None if track.vn_m_s is None else track.vn_m_s.copy(),
        ve_m_s=None if track.ve_m_s is None else track.ve_m_s.copy(),
        vd_m_s=None if track.vd_m_s is None else track.vd_m_s.copy(),
    )


def _up_velocity(track: Track) -> tuple[np.ndarray, np.ndarray]:
    """Up-positive vertical speed. Logged ``vz`` is down-positive in NED."""

    if track.vd_m_s is not None:
        return track.time_s, -np.asarray(track.vd_m_s, dtype=np.float64)
    height_up = -track.down_m
    if track.time_s.shape[0] < 2:
        raise TimeAlignmentError("need at least two samples to align ground truth")
    return track.time_s, np.gradient(height_up, track.time_s)


def _correlation(
    t_ref: np.ndarray,
    y_ref: np.ndarray,
    t_other: np.ndarray,
    y_other: np.ndarray,
    offset_s: float,
) -> tuple[float, float]:
    """Normalized correlation after adding ``offset_s`` to the other clock."""

    start = max(float(t_ref[0]), float(t_other[0] + offset_s))
    end = min(float(t_ref[-1]), float(t_other[-1] + offset_s))
    overlap = end - start
    if overlap <= 0.0:
        return 0.0, 0.0
    count = max(8, int(overlap / 0.02))
    grid = np.linspace(start, end, count)
    left = np.interp(grid, t_ref, y_ref)
    right = np.interp(grid, t_other + offset_s, y_other)
    left = left - left.mean()
    right = right - right.mean()
    denom = float(np.linalg.norm(left) * np.linalg.norm(right))
    if denom < 1e-9:
        return 0.0, float(overlap)
    return float(np.dot(left, right) / denom), float(overlap)


def _fft_offset(
    t_ref: np.ndarray,
    y_ref: np.ndarray,
    t_other: np.ndarray,
    y_other: np.ndarray,
    dt: float = 0.02,
) -> float:
    """Lag, in seconds, that slides the other signal onto the reference.

    ``offset`` is added to the other timestamps. A feature that happens
    later in the TUM file needs a negative offset.
    """

    start = min(float(t_ref[0]), float(t_other[0]))
    end = max(float(t_ref[-1]), float(t_other[-1]))
    grid = np.arange(start, end + dt, dt)
    if grid.shape[0] < 4:
        raise TimeAlignmentError("not enough samples to align ground truth")
    left = _on_grid(grid, t_ref, y_ref)
    right = _on_grid(grid, t_other, y_other)
    size = int(2 ** np.ceil(np.log2(grid.shape[0] * 2)))
    corr = np.fft.irfft(np.fft.rfft(left, n=size) * np.conj(np.fft.rfft(right, n=size)), n=size)
    lag = int(np.argmax(corr))
    if lag > size // 2:
        lag -= size
    return float(lag * dt)


def _on_grid(grid: np.ndarray, time_s: np.ndarray, values: np.ndarray) -> np.ndarray:
    """Resample onto ``grid``, with zeros outside the signal's own span."""

    sampled = np.interp(grid, time_s, values, left=0.0, right=0.0)
    inside = (grid >= time_s[0]) & (grid <= time_s[-1])
    sampled = np.where(inside, sampled, 0.0)
    if np.any(inside):
        sampled[inside] = sampled[inside] - sampled[inside].mean()
    return sampled
