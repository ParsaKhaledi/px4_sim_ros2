"""RTAB-Map odometry to a PX4 ``VehicleOdometry`` sample.

The bridge keeps the SLAM header stamp. On tracking loss, or when the last
sample is older than the timeout, it returns nothing. It does not publish the
previous pose under a new timestamp.

``reset_counter`` increments when a valid pose jumps farther than the motion
predicted from the previous sample, which is how an RTAB-Map relocalization
shows up on ``nav_msgs/Odometry``.
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


class VisionOdometryBridge:
    """Convert ENU/FLU odometry and remember SLAM resets."""

    def __init__(
        self,
        timeout_s: float = 0.3,
        reset_jump_m: float = 0.75,
        max_variance: float = 25.0,
    ) -> None:
        if timeout_s <= 0.0:
            raise ValueError('timeout_s must be positive')
        self.timeout_s = float(timeout_s)
        self.reset_jump_m = float(reset_jump_m)
        self.max_variance = float(max_variance)
        self.reset_counter = 0
        self._last: VisualOdom | None = None
        self._last_rx: float | None = None
        self._lost = False
        self._tracked_once = False
        self._prev_position: np.ndarray | None = None
        self._prev_velocity_ned: np.ndarray | None = None
        self._prev_stamp: float | None = None

    @property
    def tracking(self) -> bool:
        return self._last is not None and not self._lost

    def mark_lost(self) -> None:
        """Stop publishing. The last pose is not restamped later."""
        self._lost = True
        self._last = None

    def push(self, sample: OdomSample) -> VisualOdom | None:
        """Ingest one odometry message. Returns a sample only while tracking."""
        self._last_rx = sample.stamp_sec
        recovering = self._lost and self._tracked_once
        if sample.tracking_lost or sample.stamp_sec <= 0.0:
            self.mark_lost()
            return None
        if not _finite_vector(sample.position_enu, 3) or not _finite_vector(sample.quat_xyzw, 4):
            self.mark_lost()
            return None
        attitude = attitude_from_enu_quat(sample.quat_xyzw)
        if attitude is None:
            self.mark_lost()
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
            self.mark_lost()
            return None

        position_ned = enu_to_ned(sample.position_enu)
        velocity_frd = flu_to_frd(sample.linear_flu)
        angular_frd = flu_to_frd(sample.angular_flu)
        velocity_ned = attitude.rotation @ velocity_frd
        # One bump re-anchors EKF2. A gap and a pose jump on the same sample
        # are the same event, so they do not increment twice.
        if recovering:
            self.reset_counter = (self.reset_counter + 1) % 256
        else:
            self._bump_reset_if_jumped(position_ned, velocity_ned, sample.stamp_sec)

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
        self._prev_position = position_ned
        self._prev_velocity_ned = velocity_ned
        self._prev_stamp = float(sample.stamp_sec)
        self._lost = False
        self._tracked_once = True
        self._last = visual
        return visual

    def current(self, now_sec: float) -> VisualOdom | None:
        """Latest valid sample, or ``None`` when tracking is stale.

        ``now_sec`` is only used for the timeout. It is never written into
        the returned stamp.
        """
        if self._lost or self._last is None or self._last_rx is None:
            return None
        if now_sec - self._last_rx > self.timeout_s:
            self.mark_lost()
            return None
        return self._last

    def _bump_reset_if_jumped(self, position_ned: np.ndarray, velocity_ned: np.ndarray, stamp: float) -> None:
        if self._prev_position is None or self._prev_stamp is None or self._prev_velocity_ned is None:
            return
        dt = max(1e-3, stamp - self._prev_stamp)
        predicted = self._prev_position + self._prev_velocity_ned * dt
        if float(np.linalg.norm(position_ned - predicted)) > self.reset_jump_m:
            self.reset_counter = (self.reset_counter + 1) % 256
