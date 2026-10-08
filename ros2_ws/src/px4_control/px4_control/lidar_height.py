"""Independent downward-lidar check against EKF2 height above the floor.

The range is along the body down axis. Tilt compensation multiplies it by
the NED-down component of that axis, ``R[2, 2]`` of the body-to-NED
rotation, which is ``cos(roll) cos(pitch)``. The sensor sits
``lidar_mount_offset_m`` above the floor at rest (base_link 0.24 m above
the model origin, the sensor 5 cm below base_link, so 0.19 m). That
offset is subtracted after tilt compensation so both sides are heights
of the same reference above the floor.

EKF2 height is local ``z`` relative to the last landed sample, not
``dist_bottom``. With range aid on, ``dist_bottom`` follows this sensor.

The check stays disarmed until a reading is above 0.5 m and inside the
sensor's min/max with ``signal_quality`` not 0. Samples outside that
range are ignored. A single armed sample past the tolerance does not
abort. The gap has to stay above the tolerance for more than
``duration_s``. With no ``distance_sensor`` the guard stays idle and
reports that once.
"""

from __future__ import annotations

import math

NO_DISTANCE_SENSOR_LOG = 'no distance_sensor; lidar height guard disabled'
DISTANCE_SENSOR_ACTIVE_LOG = 'distance_sensor received; lidar height guard active'
# A slow rangefinder still beats this idle announcement.
ABSENCE_LOG_AFTER_S = 1.0
# Stay disarmed until the lidar itself reads above this, so a resting
# airframe (range about the mount offset, often below min range) cannot trip.
LIDAR_ARM_ABOVE_M = 0.5


def tilt_compensated_height(range_m: float, body_down_cosine: float) -> float | None:
    """Vertical metres from the sensor to the floor. ``None`` when the beam points up."""
    distance = float(range_m)
    cosine = float(body_down_cosine)
    if not math.isfinite(distance) or not math.isfinite(cosine):
        return None
    if distance < 0.0 or cosine <= 1e-3:
        return None
    return distance * cosine


def height_above_floor(range_m: float, body_down_cosine: float, mount_offset_m: float) -> float | None:
    """Vehicle height above the floor: tilt-compensated range minus the mount offset."""
    vertical = tilt_compensated_height(range_m, body_down_cosine)
    if vertical is None:
        return None
    return vertical - float(mount_offset_m)


def reading_in_range(
    range_m: float,
    min_distance: float,
    max_distance: float,
    signal_quality: int,
) -> bool:
    """True when the sample is inside min/max and the sensor reports a real return.

    ``signal_quality`` 0 is no return. ``-1`` is unknown and is kept.
    """
    distance = float(range_m)
    minimum = float(min_distance)
    maximum = float(max_distance)
    if not math.isfinite(distance) or not math.isfinite(minimum) or not math.isfinite(maximum):
        return False
    if int(signal_quality) == 0:
        return False
    return minimum <= distance <= maximum


class LidarHeightGuard:
    """Abort after a sustained lidar-versus-EKF2 height gap above the floor."""

    def __init__(
        self,
        tolerance_m: float = 0.3,
        duration_s: float = 0.5,
        mount_offset_m: float = 0.19,
        arm_above_m: float = LIDAR_ARM_ABOVE_M,
    ) -> None:
        tolerance = float(tolerance_m)
        duration = float(duration_s)
        offset = float(mount_offset_m)
        arm_above = float(arm_above_m)
        if tolerance <= 0.0 or tolerance != tolerance:
            raise ValueError('lidar_height_tolerance_m must be a positive finite number')
        if duration <= 0.0 or duration != duration:
            raise ValueError('lidar_height_duration_s must be a positive finite number')
        if offset < 0.0 or offset != offset:
            raise ValueError('lidar_mount_offset_m must be a non-negative finite number')
        if arm_above <= 0.0 or arm_above != arm_above:
            raise ValueError('lidar arm height must be a positive finite number')
        self.tolerance_m = tolerance
        self.duration_s = duration
        self.mount_offset_m = offset
        self.arm_above_m = arm_above
        self.armed = False
        self._breach_since: float | None = None
        self._started_s: float | None = None
        self._absence_logged = False
        self.fired = False
        self.reason: str | None = None

    def observe(
        self,
        time_s: float,
        range_m: float,
        body_down_cosine: float,
        ekf_height_m: float,
        min_distance: float,
        max_distance: float,
        signal_quality: int,
    ) -> str | None:
        """Return the abort reason once an armed gap has lasted more than ``duration_s``.

        Out-of-range samples, quality 0, and an upward beam are ignored.
        They do not arm the check and they do not keep a breach open.
        """
        if self.fired:
            return None
        if not reading_in_range(range_m, min_distance, max_distance, signal_quality):
            self._breach_since = None
            return None
        lidar_height = height_above_floor(range_m, body_down_cosine, self.mount_offset_m)
        if lidar_height is None:
            self._breach_since = None
            return None
        if not self.armed:
            if float(range_m) <= self.arm_above_m:
                return None
            self.armed = True
        error = abs(lidar_height - float(ekf_height_m))
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
            f'lidar height above floor {lidar_height:.3f} m differs from '
            f'EKF2 height above floor {float(ekf_height_m):.3f} m by {error:.3f} m '
            f'for {elapsed:.3f} s '
            f'(mount offset {self.mount_offset_m:.3f} m, '
            f'limit {self.tolerance_m:.3f} m for more than {self.duration_s:.3f} s)'
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
