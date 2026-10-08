"""Setpoint generation without Gazebo."""

import math

import numpy as np
import pytest

from px4_control.motion import Limits, MotionExecutive, Snapshot
from px4_control.trajectory import BrakeThenHold


def _limits(**kwargs) -> Limits:
    base = dict(
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
    base.update(kwargs)
    return Limits(**base)


def _snap(position, velocity=None, yaw=0.0, landed=False) -> Snapshot:
    vel = np.zeros(3) if velocity is None else np.asarray(velocity, dtype=float)
    return Snapshot(np.asarray(position, dtype=float), vel, yaw, 0.0, landed)


def _fly(executive, snap, seconds, dt=0.05, follow=True):
    samples = []
    steps = int(seconds / dt)
    for index in range(steps + 1):
        time_s = index * dt
        setpoint = executive.update(time_s, snap)
        samples.append(setpoint)
        if follow and setpoint.use_position:
            snap = Snapshot(
                setpoint.position.copy(),
                setpoint.velocity.copy(),
                setpoint.yaw,
                setpoint.yaw_rate,
                False,
            )
    return samples


def test_brake_does_not_snap_the_position_setpoint():
    start = np.array([0.0, 0.0, -1.0])
    velocity = np.array([1.0, 0.0, 0.0])
    brake = BrakeThenHold(start, velocity, yaw=0.2, yaw_rate=0.0, a_brake=1.0, yaw_accel=1.0)
    first = brake.sample(0.0)
    assert first.position == pytest.approx(start)
    assert first.velocity == pytest.approx(velocity)
    assert first.stopped is False
    stopped = brake.sample(brake.t_stop)
    assert stopped.stopped is True
    # Stopping distance is v^2 / (2 a) = 0.5 m, not a latch at the start point.
    assert stopped.position[0] == pytest.approx(0.5)
    assert float(np.linalg.norm(stopped.velocity)) == pytest.approx(0.0, abs=1e-6)


def test_goto_does_not_step_the_position_setpoint():
    executive = MotionExecutive(_limits())
    start = np.array([0.0, 0.0, -1.0])
    snap = _snap(start, landed=True)
    executive.update(0.0, snap)
    goal = np.array([3.0, 0.0, -1.0])
    goal_id = executive.request_goto(0.0, snap, goal, None)
    first = executive.update(0.05, snap)
    assert first.use_position is True
    assert np.all(np.isfinite(first.position))
    assert float(np.linalg.norm(first.position - goal)) > 2.0
    assert float(np.linalg.norm(first.position - start)) < 0.05
    samples = [first]
    state = snap
    for index in range(1, 160):
        time_s = index * 0.05
        setpoint = executive.update(time_s, state)
        samples.append(setpoint)
        state = Snapshot(setpoint.position.copy(), setpoint.velocity.copy(), setpoint.yaw, setpoint.yaw_rate, False)
        if executive.poll(goal_id).done:
            break
    speeds = [float(np.linalg.norm(sample.velocity[:2])) for sample in samples]
    assert max(speeds) <= 1.05
    assert executive.poll(goal_id).success
    assert samples[-1].position == pytest.approx(goal, abs=0.05)


def test_cmd_vel_position_is_finite_and_then_brakes():
    executive = MotionExecutive(_limits())
    snap = _snap(np.array([0.0, 0.0, -1.0]), landed=True)
    executive.update(0.0, snap)
    executive.note_cmd_vel(0.2, 0.4, 0.0, 0.0)
    moved = executive.update(0.25, snap)
    assert np.all(np.isfinite(moved.position[:2]))
    assert moved.position[0] != pytest.approx(0.0)
    # No further command: after the timeout the executive brakes instead of
    # latching the position while the velocity is still nonzero.
    coast = executive.update(0.8, Snapshot(moved.position, moved.velocity, moved.yaw, 0.0, False))
    assert float(np.linalg.norm(coast.velocity)) < float(np.linalg.norm(moved.velocity)) + 1e-6
    assert coast.position[0] == pytest.approx(moved.position[0], abs=0.05)


def test_turn_ramps_at_the_yaw_rate_limit_with_xy_locked():
    executive = MotionExecutive(_limits())
    snap = _snap(np.array([1.0, 2.0, -1.5]), landed=True)
    executive.update(0.0, snap)
    goal_id = executive.request_turn(0.0, snap, math.pi)
    locked = None
    peak = 0.0
    state = snap
    done = False
    for index in range(1, 400):
        time_s = index * 0.05
        setpoint = executive.update(time_s, state)
        if locked is None:
            locked = setpoint.position.copy()
        assert setpoint.position[:2] == pytest.approx(locked[:2], abs=1e-6)
        peak = max(peak, abs(setpoint.yaw_rate))
        state = Snapshot(setpoint.position.copy(), setpoint.velocity.copy(), setpoint.yaw, setpoint.yaw_rate, False)
        if executive.poll(goal_id).done:
            done = True
            break
    assert done
    assert executive.poll(goal_id).success
    assert peak <= math.radians(30.0) + 1e-3
    # 180 deg at 30 deg/s cannot finish much before 6 seconds.
    assert index * 0.05 >= 5.0
    assert abs(state.yaw_ned - math.pi) < 0.1 or abs(state.yaw_ned + math.pi) < 0.1


def test_takeoff_climbs_smoothly_and_hold_waits():
    executive = MotionExecutive(_limits(settle_time=0.1))
    snap = _snap(np.zeros(3), landed=True)
    executive.update(0.0, snap)
    takeoff_id = executive.request_takeoff(0.0, snap, 2.0)
    state = snap
    for index in range(1, 400):
        setpoint = executive.update(index * 0.05, state)
        assert setpoint.position[2] <= 0.05
        state = Snapshot(setpoint.position.copy(), setpoint.velocity.copy(), setpoint.yaw, 0.0, False)
        if executive.poll(takeoff_id).done:
            break
    assert executive.poll(takeoff_id).success
    assert state.position_ned[2] == pytest.approx(-2.0, abs=0.1)
    hold_id = executive.request_hold(index * 0.05, state, 0.5)
    start = index
    for index in range(start + 1, start + 80):
        setpoint = executive.update(index * 0.05, state)
        state = Snapshot(setpoint.position.copy(), setpoint.velocity.copy(), setpoint.yaw, 0.0, False)
        if executive.poll(hold_id).done:
            break
    assert executive.poll(hold_id).success
    assert (index - start) * 0.05 >= 0.5


def test_land_keeps_descending_until_the_detector_says_landed():
    executive = MotionExecutive(_limits(settle_time=0.1))
    ground = _snap(np.zeros(3), landed=True)
    executive.update(0.0, ground)
    air = _snap(np.array([0.0, 0.0, -2.0]), landed=False)
    executive.update(0.05, air)
    land_id = executive.request_land(0.05, air)
    state = air
    setpoint = None
    for index in range(1, 250):
        setpoint = executive.update(0.05 + index * 0.05, state)
        assert not executive.poll(land_id).done
        state = Snapshot(setpoint.position.copy(), np.zeros(3), setpoint.yaw, 0.0, False)
    assert setpoint is not None
    assert float(setpoint.position[2]) > 0.2
    landed = Snapshot(np.array([state.position_ned[0], state.position_ned[1], 0.02]), np.zeros(3), state.yaw_ned, 0.0, True)
    executive.update(20.0, landed)
    assert executive.poll(land_id).success
    assert executive.poll(land_id).message == 'landed'


def test_vision_loss_holds_and_does_not_keep_yawing():
    executive = MotionExecutive(_limits())
    snap = _snap(np.array([0.0, 0.0, -2.0]), yaw=0.0)
    executive.update(0.0, snap)
    executive.note_cmd_vel(0.0, 0.0, 0.0, 0.2)
    spinning = executive.update(0.05, snap)
    assert abs(spinning.yaw_rate) > 0.0
    executive.note_vision_lost(0.05)
    state = snap
    last = spinning
    for index in range(1, 80):
        last = executive.update(0.05 + index * 0.05, state)
        state = Snapshot(last.position.copy(), last.velocity.copy(), last.yaw, last.yaw_rate, False)
    assert last.use_position is True
    assert last.yaw_rate == pytest.approx(0.0, abs=1e-3)
    assert abs(last.yaw) < math.radians(15.0)
    executive.note_cmd_vel(6.0, 0.0, 0.0, 0.2)
    held = executive.update(6.05, state)
    assert held.yaw_rate == pytest.approx(0.0, abs=1e-3)
