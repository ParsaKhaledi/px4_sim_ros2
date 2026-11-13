"""RTAB-Map odometry to a PX4 ``VehicleOdometry`` sample.

A frame is lost tracking only when RTAB-Map says so: a null pose, a pose
covariance of 9999 (``publish_null_when_lost``), or ``OdomInfo.lost``. Those
samples are never forwarded. The age check compares the caller clock, which
is sim time when the node uses ``use_sim_time``, against the last accepted
header stamp. It only notices a stalled publisher. It does not mark the
stream lost and it does not change ``reset_counter``.

``reset_counter`` increments only when odometry info reports a new map or a
reset. A slow but valid frame keeps the previous counter.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from px4_control.frames import (
    R_FRD_FROM_FLU,
    R_NED_FROM_ENU,
    attitude_from_enu_quat,
    covariance_block,
    diagonal_variances,
    enu_to_ned,
    flu_to_frd,
    rotate_covariance,
)

# RTAB-Map's lost-estimate convention: a diagonal of 9999 means the pose is
# not a measurement. The same value on a non-lost info message is how a reset
# asks the mapper to start a new map.
LOST_VARIANCE = 9999.0


@dataclass(frozen=True)
class OdomSample:
    """Plain odometry input so the bridge can be tested without ROS messages."""

    stamp_sec: float
    position_enu: np.ndarray
    quat_xyzw: np.ndarray
    linear_flu: np.ndarray
    angular_flu: np.ndarray
    pose_covariance: np.ndarray
    twist_covariance: np.ndarray
    tracking_lost: bool = False


@dataclass(frozen=True)
class VisualOdom:
    """Fields copied onto ``px4_msgs/VehicleOdometry``."""

    stamp_sec: float
    position_ned: np.ndarray
    quaternion_wxyz: np.ndarray
    velocity_frd: np.ndarray
    angular_velocity_frd: np.ndarray
    position_variance: np.ndarray
    orientation_variance: np.ndarray
    velocity_variance: np.ndarray
    reset_counter: int
    yaw_ned: float


def _finite_vector(value: np.ndarray, size: int) -> bool:
    array = np.asarray(value, dtype=float).reshape(-1)
    return array.size == size and bool(np.all(np.isfinite(array)))


def _invalid_diagonal(covariance: np.ndarray, max_variance: float) -> bool:
    diag = np.diag(np.asarray(covariance, dtype=float).reshape(3, 3))
    if not np.all(np.isfinite(diag)):
        return True
    if np.any(diag < 0.0):
        return True
    return bool(np.any(diag > max_variance))


def lost_covariance(covariance: np.ndarray) -> bool:
    """True when a 6x6 covariance uses RTAB-Map's 9999 lost/reset value."""
    array = np.asarray(covariance, dtype=float).reshape(-1)
    if array.size < 36:
        return False
    diag = array.reshape(6, 6).diagonal()
    return bool(np.any(np.isfinite(diag) & (diag >= LOST_VARIANCE)))


def _is_null_pose(position: np.ndarray, quat: np.ndarray) -> bool:
    """RTAB-Map's published null transform, not a real pose at the origin.

    A pose at the origin with an identity quaternion is a real measurement.
    The null pose is a zero translation together with a zero quaternion.
    """
    pos = np.asarray(position, dtype=float).reshape(-1)
    rotation = np.asarray(quat, dtype=float).reshape(-1)
    if pos.size != 3 or rotation.size != 4:
        return False
    if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(rotation)):
        return False
    return float(np.linalg.norm(rotation)) < 1e-6 and float(np.linalg.norm(pos)) < 1e-6


class VisionOdometryBridge:
    """Convert ENU/FLU odometry and remember SLAM map resets."""

    def __init__(
        self,
        timeout_s: float = 0.3,
        reset_jump_m: float = 0.75,
        max_variance: float = 25.0,
    ) -> None:
        if timeout_s <= 0.0:
            raise ValueError('timeout_s must be positive')
        self.timeout_s = float(timeout_s)
        # Pose jumps are not a reset. The argument remains so existing callers
        # and the ROS parameter still load.
        self.reset_jump_m = float(reset_jump_m)
        self.max_variance = float(max_variance)
        self.reset_counter = 0
        self._last: VisualOdom | None = None
        self._last_rx: float | None = None
        self._lost = False

    @property
    def tracking(self) -> bool:
        return self._last is not None and not self._lost

    def mark_lost(self) -> None:
        """Stop publishing. The last pose is not restamped later."""
        self._lost = True
        self._last = None

    def note_odom_info(self, *, lost: bool = False, new_map: bool = False) -> None:
        """Apply an ``OdomInfo`` signal.

        ``lost`` stops publishing. ``new_map`` is the reset that starts a new
        map; it is the only event that increments ``reset_counter``.
        """
        if new_map:
            self.reset_counter = (self.reset_counter + 1) % 256
        if lost:
            self.mark_lost()

    def push(self, sample: OdomSample) -> VisualOdom | None:
        """Ingest one odometry message. Returns a sample only while tracking."""
        if sample.tracking_lost or _is_null_pose(sample.position_enu, sample.quat_xyzw):
            self.mark_lost()
            return None
        if lost_covariance(sample.pose_covariance):
            self.mark_lost()
            return None
        if sample.stamp_sec <= 0.0:
            return None
        if not _finite_vector(sample.position_enu, 3) or not _finite_vector(sample.quat_xyzw, 4):
            return None
        attitude = attitude_from_enu_quat(sample.quat_xyzw)
        if attitude is None:
            return None
        pose_cov = np.asarray(sample.pose_covariance, dtype=float).reshape(6, 6)
        twist_cov = np.asarray(sample.twist_covariance, dtype=float).reshape(6, 6)
        position_cov_enu = covariance_block(pose_cov, 0)
        orientation_cov_enu = covariance_block(pose_cov, 3)
        velocity_cov_flu = covariance_block(twist_cov, 0)
        if (
            _invalid_diagonal(position_cov_enu, self.max_variance)
            or _invalid_diagonal(orientation_cov_enu, self.max_variance)
            or _invalid_diagonal(velocity_cov_flu, self.max_variance)
        ):
            return None

        position_ned = enu_to_ned(sample.position_enu)
        velocity_frd = flu_to_frd(sample.linear_flu)
        angular_frd = flu_to_frd(sample.angular_flu)
        position_cov_ned = rotate_covariance(position_cov_enu, R_NED_FROM_ENU)
        # Pose orientation covariance is in the parent ENU frame. PX4 wants the
        # body-FRD diagonal: R_frd_enu = R_ned_frd.T @ R_ned_enu.
        orientation_cov_frd = rotate_covariance(
            orientation_cov_enu,
            attitude.rotation.T @ R_NED_FROM_ENU,
        )
        velocity_cov_frd = rotate_covariance(velocity_cov_flu, R_FRD_FROM_FLU)
        visual = VisualOdom(
            stamp_sec=float(sample.stamp_sec),
            position_ned=position_ned,
            quaternion_wxyz=attitude.quaternion_wxyz,
            velocity_frd=velocity_frd,
            angular_velocity_frd=angular_frd,
            position_variance=diagonal_variances(position_cov_ned),
            orientation_variance=diagonal_variances(orientation_cov_frd),
            velocity_variance=diagonal_variances(velocity_cov_frd),
            reset_counter=self.reset_counter,
            yaw_ned=attitude.yaw,
        )
        self._lost = False
        self._last_rx = float(sample.stamp_sec)
        self._last = visual
        return visual

    def current(self, now_sec: float) -> VisualOdom | None:
        """Latest valid sample, or ``None`` when lost or the stream has stalled.

        ``now_sec`` is the caller clock (sim time under ``use_sim_time``). It
        is compared with the message stamp and is never written into the
        returned stamp. A stall does not mark tracking lost and does not
        increment ``reset_counter``.
        """
        if self._lost or self._last is None or self._last_rx is None:
            return None
        if now_sec - self._last_rx > self.timeout_s:
            return None
        return self._last
