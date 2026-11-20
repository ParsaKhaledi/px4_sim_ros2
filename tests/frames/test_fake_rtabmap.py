"""Fake RTAB-Map pose math. The ROS node is not started."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from px4_control.frames import (
    attitude_from_enu_quat,
    quat_xyzw_to_rot,
    rot_to_quat_xyzw,
    rpy_to_rot,
)
from px4_control.testing.fake_rtabmap import FakeRtabmap, GroundTruth
from px4_control.vision_bridge import LOST_VARIANCE, OdomSample, VisionOdometryBridge, vo_stamp_faults


def _quat(yaw: float) -> np.ndarray:
    return rot_to_quat_xyzw(rpy_to_rot(0.0, 0.0, yaw))


def _gt(stamp: float, position=(0.0, 0.0, 0.2), yaw: float = 0.0) -> GroundTruth:
    return GroundTruth(
        stamp_sec=stamp,
        position_enu=np.array(position, dtype=float),
        quat_xyzw=_quat(yaw),
        linear_flu=np.zeros(3),
        angular_flu=np.zeros(3),
    )


def _drain(source: FakeRtabmap, sample: GroundTruth):
    source.push(sample)
    return source.poll(sample.stamp_sec + source.delay_s)


def test_publisher_matches_rtabmap_sensor_qos():
    # qos:=2 is best effort. The flight subscriber is best effort too.
    text = Path(__file__).resolve().parents[2].joinpath(
        "ros2_ws/src/px4_control/px4_control/testing/fake_rtabmap.py"
    ).read_text(encoding="utf-8")
    assert "ReliabilityPolicy.BEST_EFFORT" in text
    assert 'create_publisher(Odometry, "/rtabmap/odom", sensor_qos)' in text


def test_origin_keeps_world_yaw_and_header_is_the_capture():
    yaw = math.radians(37.0)
    spawn = np.array([-1.7, 2.4, 0.15])
    source = FakeRtabmap(world_axes=True)
    first = _drain(source, _gt(5.0, spawn, yaw))
    assert len(first) == 1
    assert first[0].position_enu == pytest.approx([0.0, 0.0, 0.0], abs=1e-9)
    assert quat_xyzw_to_rot(first[0].quat_xyzw) == pytest.approx(quat_xyzw_to_rot(_quat(yaw)), abs=1e-8)
    assert first[0].capture_sec == 5.0
    assert first[0].publish_sec == pytest.approx(5.045)
    assert first[0].frame_id == "odom"
    assert first[0].child_frame_id == "base_link"
    # Nothing is due before the delay.
    source.push(_gt(5.08, spawn + np.array([0.3, 0.0, 0.0]), yaw))
    assert source.poll(5.08) == []
    moved = source.poll(5.08 + source.delay_s)
    assert moved[0].position_enu == pytest.approx([0.3, 0.0, 0.0], abs=1e-8)
    assert moved[0].capture_sec == 5.08


def test_start_at_origin_identity_drops_the_spawn_yaw():
    yaw = math.radians(37.0)
    source = FakeRtabmap(world_axes=False)
    first = _drain(source, _gt(1.0, (1.0, 2.0, 0.3), yaw))
    assert first[0].position_enu == pytest.approx([0.0, 0.0, 0.0], abs=1e-8)
    assert quat_xyzw_to_rot(first[0].quat_xyzw) == pytest.approx(np.eye(3), abs=1e-8)
    # The bridge reads that identity as facing East, which is not the spawn yaw.
    attitude = attitude_from_enu_quat(first[0].quat_xyzw)
    assert attitude is not None
    assert attitude.yaw == pytest.approx(math.pi / 2.0, abs=1e-6)
    assert attitude.yaw != pytest.approx(math.pi / 2.0 - yaw, abs=1e-3)


def test_rate_delay_noise_and_drift():
    source = FakeRtabmap(rate_hz=12.5, noise_m=0.0, drift_mps=0.0)
    published = []
    for index in range(51):
        stamp = index * 0.02
        source.push(_gt(stamp, (0.0, 0.0, 0.2)))
        published.extend(source.poll(stamp + source.delay_s))
    assert len(published) == 13
    assert all(item.publish_sec - item.capture_sec == pytest.approx(0.045) for item in published)

    drifted = FakeRtabmap(drift_mps=0.1)
    last = None
    for index in range(13):
        stamp = index * 0.08
        last = _drain(drifted, _gt(stamp, (0.0, 0.0, 0.2)))
    assert last is not None
    assert last[0].position_enu[0] == pytest.approx(0.096, abs=1e-6)
    # Drift is +East only. z is unchanged even when drift_mps is not zero.
    assert last[0].position_enu[2] == pytest.approx(0.0, abs=1e-9)

    noisy = FakeRtabmap(noise_m=0.05, seed=3)
    clean = FakeRtabmap(noise_m=0.0, seed=3)
    _drain(noisy, _gt(1.0, (0.0, 0.0, 1.0)))
    _drain(clean, _gt(1.0, (0.0, 0.0, 1.0)))
    second_noisy = _drain(noisy, _gt(1.08, (0.0, 0.0, 1.0)))[0].position_enu
    second_clean = _drain(clean, _gt(1.08, (0.0, 0.0, 1.0)))[0].position_enu
    assert not np.allclose(second_noisy, second_clean, atol=1e-4)


def test_faults_dropout_late_jump_and_lost():
    dropout = FakeRtabmap(fault="dropout", dropout_yaw_deg=70.0)
    assert len(_drain(dropout, _gt(1.0, yaw=0.0))) == 1
    dropout.push(_gt(1.08, yaw=math.radians(80.0)))
    assert dropout.poll(1.08 + dropout.delay_s) == []

    late = FakeRtabmap(fault="late", late_extra_s=0.08)
    message = _drain(late, _gt(3.0))[0]
    assert message.capture_sec == pytest.approx(2.92)
    gap_us = int(round((message.publish_sec - message.capture_sec) * 1e6))
    assert vo_stamp_faults(gap_us, 0, expected_delay_us=45_000)

    jump = FakeRtabmap(fault="jump", jump_m=2.0)
    _drain(jump, _gt(1.0, (0.0, 0.0, 0.2)))
    jumped = _drain(jump, _gt(1.08, (0.0, 0.0, 0.2)))[0]
    assert jumped.position_enu[0] == pytest.approx(2.0, abs=1e-6)
    bridge = VisionOdometryBridge()
    first = bridge.push(OdomSample(
        stamp_sec=1.0,
        position_enu=np.zeros(3),
        quat_xyzw=_quat(0.0),
        linear_flu=np.zeros(3),
        angular_flu=np.zeros(3),
        pose_covariance=np.zeros(36),
        twist_covariance=np.zeros(36),
    ))
    second = bridge.push(OdomSample(
        stamp_sec=1.08,
        position_enu=jumped.position_enu,
        quat_xyzw=jumped.quat_xyzw,
        linear_flu=jumped.linear_flu,
        angular_flu=jumped.angular_flu,
        pose_covariance=jumped.pose_covariance,
        twist_covariance=jumped.twist_covariance,
    ))
    assert first is not None and second is not None
    assert bridge.reset_counter == 0

    lost = FakeRtabmap(fault="lost")
    lost_msg = _drain(lost, _gt(4.0, (1.0, 1.0, 1.0), 0.3))[0]
    assert lost_msg.lost
    assert lost_msg.position_enu == pytest.approx(np.zeros(3))
    assert lost_msg.quat_xyzw == pytest.approx(np.zeros(4))
    assert lost_msg.pose_covariance[0] == LOST_VARIANCE
    assert bridge.push(OdomSample(
        stamp_sec=lost_msg.capture_sec,
        position_enu=lost_msg.position_enu,
        quat_xyzw=lost_msg.quat_xyzw,
        linear_flu=lost_msg.linear_flu,
        angular_flu=lost_msg.angular_flu,
        pose_covariance=lost_msg.pose_covariance,
        twist_covariance=lost_msg.twist_covariance,
    )) is None
