"""Independent downward-lidar check against EKF2 height above ground.

The range is along the body down axis. Tilt compensation multiplies it by
the NED-down component of that axis, ``R[2, 2]`` of the body-to-NED
rotation, which is ``cos(roll) cos(pitch)``.

The comparison is against EKF2 local height above the ground sample, not
``dist_bottom``. With range aid on, ``dist_bottom`` follows this same
sensor, so it is not an independent height.

A single sample past the tolerance does not abort. The gap has to stay
above the tolerance for more than ``duration_s``. With no ``distance_sensor``
the guard stays idle and reports that once.
"""

from __future__ import annotations

import math

NO_DISTANCE_SENSOR_LOG = 'no distance_sensor; lidar height guard disabled'
DISTANCE_SENSOR_ACTIVE_LOG = 'distance_sensor received; lidar height guard active'
# Wait long enough that a slow rangefinder beats the idle announcement.
ABSENCE_LOG_AFTER_S = 1.0


def tilt_compensated_height(range_m: float, body_down_cosine: float) -> float | None:
    """Vertical metres above the surface. ``None`` when the beam points up."""
    distance = float(range_m)
    cosine = float(body_down_cosine)
    if not math.isfinite(distance) or not math.isfinite(cosine):
        return None
    if distance < 0.0 or cosine <= 1e-3:
        return None
    return distance * cosine


class LidarHeightGuard:
    """Abort after a sustained lidar-versus-EKF2 height gap."""

    def __init__(self, tolerance_m: float = 0.3, duration_s: float = 0.5) -> None:
        tolerance = float(tolerance_m)
        duration = float(duration_s)
        if tolerance <= 0.0 or tolerance != tolerance:
            raise ValueError('lidar_height_tolerance_m must be a positive finite number')
        if duration <= 0.0 or duration != duration:
            raise ValueError('lidar_height_duration_s must be a positive finite number')
        self.tolerance_m = tolerance
        self.duration_s = duration
        self._breach_since: float | None = None
        self._started_s: float | None = None
        self._absence_logged = False
        self.fired = False
        self.reason: str | None = None

    def observe(self, time_s: float, lidar_height_m: float, ekf_height_m: float) -> str | None:
        """Return the abort reason once the gap has lasted more than ``duration_s``."""
        if self.fired:
            return None
        error = abs(float(lidar_height_m) - float(ekf_height_m))
        if error <= self.tolerance_m:
            self._breach_since = None
            return None
        now = float(time_s)
        if self._breach_since is None:
            self._breach_since = now
            return None
        elapsed = now - self._breach_since
        if elapsed <= self.duration_s:
            return None
        self.fired = True
        self.reason = (
            f'lidar height {float(lidar_height_m):.3f} m differs from '
            f'EKF2 height above ground {float(ekf_height_m):.3f} m by {error:.3f} m '
            f'for {elapsed:.3f} s '
            f'(limit {self.tolerance_m:.3f} m for more than {self.duration_s:.3f} s)'
        )
        return self.reason

    def absence(self, time_s: float, have_sample: bool) -> str | None:
        """One line when no downward range has arrived. Later samples still count."""
        if have_sample or self._absence_logged:
            return None
        now = float(time_s)
        if self._started_s is None:
            self._started_s = now
        if now - self._started_s < ABSENCE_LOG_AFTER_S:
            return None
        self._absence_logged = True
        return NO_DISTANCE_SENSOR_LOG


def lidar_height_log(reason: str) -> str:
    """One line for the flight log. The reason names the lidar-versus-EKF2 gap."""
    return f'lidar height abort: {reason}'
