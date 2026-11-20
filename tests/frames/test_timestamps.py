"""Vision stamps use the uXRCE-DDS offset. The raw header copy does not."""

from __future__ import annotations

import numpy as np

from px4_control.vision_bridge import (
    MAX_VO_DELAY_US,
    TimeSyncOffset,
    visual_timestamps,
    vo_stamp_faults,
)
from ev_bridge import fill_vehicle_odometry
from px4_control.frames import attitude_from_enu_quat
from px4_control.vision_bridge import OdomSample, VisionOdometryBridge


class _Msg:
    POSE_FRAME_NED = 1
    VELOCITY_FRAME_BODY_FRD = 2

    def __init__(self) -> None:
        self.timestamp = 0
        self.timestamp_sample = 0
        self.pose_frame = 0
        self.velocity_frame = 0
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.position_variance = [0.0, 0.0, 0.0]
        self.orientation_variance = [0.0, 0.0, 0.0]
        self.velocity_variance = [0.0, 0.0, 0.0]
        self.q = [0.0, 0.0, 0.0, 0.0]
        self.reset_counter = 0
        self.quality = 0


def test_offset_shifts_both_fields_and_the_gap_is_the_delay():
    offset = 1_500_000
    capture = 10.0
    publish = 10.045
    sent, sample = visual_timestamps(capture, publish, offset)
    assert sample == 10_000_000 + offset
    assert sent == 10_045_000 + offset
    assert sent - sample == 45_000
    assert vo_stamp_faults(sent, sample, expected_delay_us=45_000) == []


def test_copying_the_header_fails_the_delay_check():
    # The bridge used to set both fields from the header. The gap was 0.
    header_us = 10_000_000
    faults = vo_stamp_faults(header_us, header_us, expected_delay_us=45_000)
    assert faults
    assert any("expected 45000" in item for item in faults)


def test_gap_bounds():
    assert any("negative" in item for item in vo_stamp_faults(1_000, 2_000))
    assert any("above 100 ms" in item for item in vo_stamp_faults(MAX_VO_DELAY_US + 5, 0))
    assert vo_stamp_faults(80_000, 0) == []


def test_fill_uses_the_same_conversion():
    bridge = VisionOdometryBridge()
    visual = bridge.push(OdomSample(
        stamp_sec=10.0,
        position_enu=np.zeros(3),
        quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        linear_flu=np.zeros(3),
        angular_flu=np.zeros(3),
        pose_covariance=np.zeros(36),
        twist_covariance=np.zeros(36),
    ))
    assert visual is not None
    assert attitude_from_enu_quat(np.array([0.0, 0.0, 0.0, 1.0])) is not None
    msg = _Msg()
    fill_vehicle_odometry(msg, visual, publish_sec=10.045, offset_us=250_000)
    assert msg.timestamp - msg.timestamp_sample == 45_000
    assert msg.timestamp_sample == 10_000_000 + 250_000
    assert vo_stamp_faults(msg.timestamp, msg.timestamp_sample, expected_delay_us=45_000) == []


def test_timesync_beats_the_px4_stamp_fallback():
    clock = TimeSyncOffset()
    clock.note_px4_stamp(5_000_000, 1.0)
    assert clock.offset_us == 4_000_000
    assert clock.source == "px4_stamp"
    clock.note_timesync(1234)
    clock.note_px4_stamp(9_000_000, 2.0)
    assert clock.offset_us == 1234
    assert clock.source == "timesync"
