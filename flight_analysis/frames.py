"""ENU/FLU poses from ROS, converted into the NED/FRD frame PX4 logs."""

from __future__ import annotations

import math
from dataclasses import dataclass


# 180° about the ENU axis (1, 1, 0): east swaps with north and up becomes down.
_Q_ENU_TO_NED = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)
# 180° of roll: body left becomes right, and body up becomes down.
_Q_FLU_TO_FRD = (0.0, 1.0, 0.0, 0.0)


@dataclass(frozen=True)
class NedPose:
    """One pose in the PX4 world: NED position and a body-FRD-to-NED quaternion."""

    north_m: float
    east_m: float
    down_m: float
    qw: float
    qx: float
    qy: float
    qz: float

    def yaw_rad(self) -> float:
        """Return yaw in radians, zero at north and positive toward east."""

        return yaw_from_ned_quaternion(self.qw, self.qx, self.qy, self.qz)


def wrap_pi(angle: float) -> float:
    """Wrap an angle in radians into (-pi, pi]."""

    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    if wrapped == -math.pi or math.isclose(wrapped, -math.pi):
        return math.pi
    return wrapped


def yaw_from_ned_quaternion(qw: float, qx: float, qy: float, qz: float) -> float:
    """Return the NED yaw of a body-to-world quaternion, in radians."""

    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(sin_yaw, cos_yaw)


def tilt_rad_from_quaternion(qw: float, qx: float, qy: float, qz: float) -> float:
    """Return the tilt off vertical, in radians.

    Thrust is along body up, so this is the angle between body down and
    world down. ``R_zz = 1 - 2(x² + y²)`` for a unit quaternion.
    """

    cos_tilt = 1.0 - 2.0 * (qx * qx + qy * qy)
    cos_tilt = max(-1.0, min(1.0, cos_tilt))
    return math.acos(cos_tilt)


def enu_flu_to_ned_frd(
    x: float,
    y: float,
    z: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> NedPose:
    """Convert one ROS ENU/FLU pose into PX4 NED/FRD.

    ``x, y, z`` are east, north, and up, in metres. The quaternion is the
    FLU body in that ENU world, TUM order ``qx qy qz qw``.

    Position is ``(north, east, down) = (y, x, -z)``. Attitude is the
    Hamilton product ``q_enu_to_ned ⊗ q_enu ⊗ q_flu_to_frd``, so
    ``yaw_ned = π/2 - yaw_enu``. A right-hand ROS pitch about the left
    axis points the nose down, and that sign is kept in the NED pitch.
    """

    north_m = y
    east_m = x
    down_m = -z
    rotated = _quat_mul(_quat_mul(_Q_ENU_TO_NED, (qw, qx, qy, qz)), _Q_FLU_TO_FRD)
    norm = math.sqrt(sum(part * part for part in rotated))
    if norm == 0.0:
        rotated = (1.0, 0.0, 0.0, 0.0)
    else:
        rotated = tuple(part / norm for part in rotated)
    if rotated[0] < 0.0:
        rotated = tuple(-part for part in rotated)
    qw_n, qx_n, qy_n, qz_n = rotated
    return NedPose(north_m, east_m, down_m, qw_n, qx_n, qy_n, qz_n)


def _quat_mul(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Hamilton product of two ``(w, x, y, z)`` quaternions."""

    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )
