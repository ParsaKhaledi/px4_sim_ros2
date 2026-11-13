"""End-to-end height abort and the vision-hover suspect grade."""

import math

import numpy as np

from px4_control.e2e_height import HeightGuard, height_abort_log
from px4_control.grading import grade_hover
from px4_control.motion import Limits, MotionExecutive, Phase, Snapshot


def _limits() -> Limits:
    return Limits(
        v_xy=1.0,
        v_z=0.8,
        a_max=1.0,
        a_brake=1.0,
        yaw_rate=math.radians(30.0),
        yaw_accel=math.radians(90.0),
        settle_pos=0.05,
        settle_yaw=math.radians(2.0),
        settle_time=0.2,
        cmd_timeout=0.5,
    )


def _snap(position, landed=False) -> Snapshot:
    return Snapshot(np.asarray(position, dtype=float), np.zeros(3), 0.0, 0.0, landed)


def test_height_guard_aborts_on_estimate_or_setpoint_and_logs_the_gap():
    guard = HeightGuard(0.5)
    assert guard.observe(1.0, 2.0, 2.0, 2.1) is None
    reason = guard.observe(1.2, 2.0, 2.0, 2.7)
    assert reason is not None
    assert '|estimate z - ground truth z|=0.700 m' in reason
    assert '|setpoint z - ground truth z|=0.700 m' in reason
    assert guard.fault_to_abort_s == 0.0
    assert guard.fault_time_s == 1.2
    line = height_abort_log(reason, guard.fault_to_abort_s)
    assert line.startswith('height abort:')
    assert 'fault_to_abort_s=0.000' in line
    assert guard.observe(1.4, 9.0, 9.0, 0.0) is None
    assert guard.fault_time_s == 1.2


def test_setpoint_height_alone_is_enough_to_abort():
    guard = HeightGuard(0.3)
    reason = guard.observe(4.0, 2.0, 6.5, 2.05)
    assert reason is not None
    assert 'setpoint z' in reason
    assert 'estimate z' not in reason


def test_takeoff_lands_when_the_height_fault_is_applied():
    guard = HeightGuard(0.4)
    executive = MotionExecutive(_limits())
    ground = _snap(np.zeros(3), landed=True)
    executive.update(0.0, ground)
    air = _snap(np.array([0.0, 0.0, -1.0]), landed=False)
    takeoff_id = executive.request_takeoff(0.05, air, 2.0)
    setpoint = executive.update(0.1, air)
    assert executive.phase == Phase.TAKEOFF
    estimate_up = -float(air.position_ned[2])
    setpoint_up = -float(setpoint.position[2])
    ground_truth_up = 6.5
    reason = guard.observe(0.1, estimate_up, setpoint_up, ground_truth_up)
    assert reason is not None
    executive.abort_and_land(0.1, air, reason)
    logged = height_abort_log(reason, guard.fault_to_abort_s)
    assert 'fault_to_abort_s=0.000' in logged
    assert executive.poll(takeoff_id).message == reason
    landing = executive.update(0.15, air)
    assert executive.phase == Phase.LAND
    assert float(landing.velocity[2]) >= 0.40


def test_vision_hover_within_one_millimetre_is_suspect():
    grade = grade_hover('vision', [0.0002, -0.0004, 0.001])
    assert grade.grade == 'suspect'
    assert grade.failed is True
    wider = grade_hover('vision', [0.0002, 0.0011])
    assert wider.grade == 'ok'
    assert wider.failed is False
    gps = grade_hover('gps', [0.0, 0.0])
    assert gps.grade == 'ok'
    assert gps.failed is False
    empty = grade_hover('vision', [])
    assert empty.grade == 'unscored'
    assert empty.failed is False