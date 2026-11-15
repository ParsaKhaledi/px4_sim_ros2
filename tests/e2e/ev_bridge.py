"""Bridge /rtabmap/odom into /fmu/in/vehicle_visual_odometry for e2e.

Camera CI flies with ``px4_offboard`` (no ``px4_control`` node). This module
reuses the same ``VisionOdometryBridge`` conversion that the control package
uses, loaded from the bind-mounted source tree.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path


def _ensure_px4_control_path() -> None:
    candidates = (
        Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "px4_control",
        Path("/home/px4/volume/ros2_ws/src/px4_control"),
    )
    for root in candidates:
        if (root / "px4_control" / "vision_bridge.py").is_file():
            text = str(root)
            if text not in sys.path:
                sys.path.insert(0, text)
            return
    raise ImportError("px4_control vision_bridge.py not found on the volume")


def load_bridge_types():
    """Return ``(VisionOdometryBridge, OdomSample, lost_covariance)``."""
    _ensure_px4_control_path()
    from px4_control.vision_bridge import (  # noqa: WPS433
        OdomSample,
        VisionOdometryBridge,
        lost_covariance,
    )

    return VisionOdometryBridge, OdomSample, lost_covariance


class EvBridgeStats:
    """Rate / age counters for the RTAB-Map → EV path."""

    def __init__(self) -> None:
        self.accepted = 0
        self.stamps: list[float] = []
        self.wall_rx: list[float] = []
        self.last_enu: tuple[float, float, float] | None = None
        self.last_lost = False
        self.explicit_lost = False

    def note_accepted(self, stamp_sec: float, position_enu) -> None:
        self.accepted += 1
        self.stamps.append(float(stamp_sec))
        self.wall_rx.append(time.monotonic())
        if len(self.stamps) > 400:
            self.stamps = self.stamps[-400:]
            self.wall_rx = self.wall_rx[-400:]
        east, north, up = (float(position_enu[0]), float(position_enu[1]), float(position_enu[2]))
        self.last_enu = (east, north, up)
        self.last_lost = False

    def note_lost(self) -> None:
        self.last_lost = True
        self.explicit_lost = True

    def rate_hz(self, window_s: float = 5.0) -> float | None:
        if len(self.wall_rx) < 2:
            return None
        newest = self.wall_rx[-1]
        recent = [t for t in self.wall_rx if newest - t <= window_s]
        if len(recent) < 2:
            return None
        span = recent[-1] - recent[0]
        if span <= 1e-6:
            return None
        return (len(recent) - 1) / span


def fill_vehicle_odometry(msg, visual) -> None:
    """Copy a ``VisualOdom`` sample onto a ``VehicleOdometry`` message."""
    msg.timestamp = max(0, int(round(float(visual.stamp_sec) * 1e6)))
    msg.timestamp_sample = msg.timestamp
    msg.pose_frame = type(msg).POSE_FRAME_NED
    msg.velocity_frame = type(msg).VELOCITY_FRAME_BODY_FRD
    for index in range(3):
        msg.position[index] = float(visual.position_ned[index])
        msg.velocity[index] = float(visual.velocity_frd[index])
        msg.angular_velocity[index] = float(visual.angular_velocity_frd[index])
        msg.position_variance[index] = float(visual.position_variance[index])
        msg.orientation_variance[index] = float(visual.orientation_variance[index])
        msg.velocity_variance[index] = float(visual.velocity_variance[index])
    for index in range(4):
        msg.q[index] = float(visual.quaternion_wxyz[index])
    msg.reset_counter = int(visual.reset_counter)
    msg.quality = 100


def vision_gt_error_m(vision_enu, gz_enu) -> float | None:
    """Horizontal+vertical distance between RTAB-Map ENU and Gazebo ENU."""
    if vision_enu is None or gz_enu is None:
        return None
    return math.dist(vision_enu[:3], gz_enu[:3])
