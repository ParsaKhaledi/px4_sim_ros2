"""Setpoint executive shared by takeoff, land, goto, yaw, hold, and cmd_vel.

The executive never replaces the position setpoint with the goal in one step.
Each tick advances it under a velocity and acceleration limit, then publishes
position, velocity, and acceleration together.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from px4_control.frames import command_age, wrap_pi
from px4_control.trajectory import BrakeThenHold, step_scalar, step_vector2, step_yaw


class Phase(str, Enum):
    HOLD = 'hold'
    BRAKE = 'brake'
    TAKEOFF = 'takeoff'
    LAND = 'land'
    GOTO = 'goto'
    YAW = 'yaw'
    CMD_VEL = 'cmd_vel'


@dataclass
class Limits:
    v_xy: float = 1.0
    v_z: float = 0.6
    a_max: float = 1.0
    a_brake: float = 1.0
    yaw_rate: float = np.deg2rad(30.0)
    yaw_accel: float = np.deg2rad(60.0)
    settle_pos: float = 0.20
    settle_yaw: float = np.deg2rad(8.0)
    settle_time: float = 0.4
    cmd_timeout: float = 0.5
    speed_eps: float = 0.05


@dataclass
class Snapshot:
    position_ned: np.ndarray
    velocity_ned: np.ndarray
    yaw_ned: float
    yaw_rate: float = 0.0
    landed: bool = False


@dataclass
class Setpoint:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    yaw: float
    yaw_rate: float
    use_position: bool = True

    @staticmethod
    def velocity_hold() -> 'Setpoint':
        nan = float('nan')
        return Setpoint(
            position=np.array([nan, nan, nan], dtype=float),
            velocity=np.zeros(3),
            acceleration=np.zeros(3),
            yaw=nan,
            yaw_rate=0.0,
            use_position=False,
        )


@dataclass
class GoalStatus:
    goal_id: int
    done: bool = False
    success: bool = False
    message: str = ''


@dataclass
class _Settle:
    since: float | None = None

    def reset(self) -> None:
        self.since = None

    def update(self, time_s: float, ok: bool, hold_s: float) -> bool:
        if not ok:
            self.since = None
            return False
        if self.since is None:
            self.since = time_s
        return (time_s - self.since) >= hold_s


def _zeros() -> np.ndarray:
    return np.zeros(3, dtype=float)


class MotionExecutive:
    """Own the setpoint state machine. The ROS node only does I/O around it."""

    def __init__(self, limits: Limits | None = None) -> None:
        self.limits = limits or Limits()
        self.phase = Phase.HOLD
        self.speed_cap = None  # callable(pos_ned, direction_ne) -> max speed
        self._goal_seq = 0
        self._active: int | None = None
        self._status: dict[int, GoalStatus] = {}
        self._p: np.ndarray | None = None
        self._v = _zeros()
        self._a = _zeros()
        self._yaw = 0.0
        self._yaw_rate = 0.0
        self._last_t: float | None = None
        self._ground_d: float | None = None
        self._brake: BrakeThenHold | None = None
        self._brake_t0 = 0.0
        self._after_brake = Phase.HOLD
        self._target = _zeros()
        self._yaw_goal: float | None = None
        self._keep_yaw = True
        self._settle = _Settle()
        self._hold_duration = 0.0
        self._hold_since: float | None = None
        self._holding = False
        self._cmd_n = 0.0
        self._cmd_e = 0.0
        self._cmd_yaw_rate = 0.0
        self._cmd_time: float | None = None
        self._hold_z: float | None = None
        self._locked_xy: np.ndarray | None = None
        self.needs_disarm = False

    def poll(self, goal_id: int) -> GoalStatus:
        return self._status[goal_id]

    def accepts_cmd_vel(self) -> bool:
        return self.phase not in (Phase.TAKEOFF, Phase.LAND)

    def note_landed(self, landed: bool, down: float) -> None:
        if landed:
            self._ground_d = float(down)

    def update(self, time_s: float, snap: Snapshot | None) -> Setpoint:
        if snap is not None and snap.landed:
            self._ground_d = float(snap.position_ned[2])
        if self._p is None:
            if snap is None:
                return Setpoint.velocity_hold()
            self._seed(snap, time_s)
        assert self._p is not None
        dt = 0.0 if self._last_t is None else max(0.0, time_s - self._last_t)
        self._last_t = time_s
        if self.phase == Phase.CMD_VEL and self._cmd_time is not None:
            if command_age(self._cmd_time, time_s) >= self.limits.cmd_timeout:
                self._begin_brake(time_s, Phase.HOLD)
        if self.phase == Phase.BRAKE:
            self._tick_brake(time_s)
        elif self.phase == Phase.TAKEOFF:
            self._tick_path(time_s, dt, snap, vertical=True)
        elif self.phase == Phase.LAND:
            self._tick_land(time_s, dt, snap)
        elif self.phase == Phase.GOTO:
            self._tick_path(time_s, dt, snap, vertical=False)
        elif self.phase == Phase.YAW:
            self._tick_yaw(time_s, dt, snap)
        elif self.phase == Phase.CMD_VEL:
            self._tick_cmd(dt)
        else:
            self._v = _zeros()
            self._a = _zeros()
            self._yaw_rate = 0.0
            self._tick_hold_timer(time_s)
        return self._emit()

    def request_takeoff(self, time_s: float, snap: Snapshot, height: float) -> int:
        if height <= 0.0:
            return self._reject('height must be positive')
        if self.phase == Phase.LAND:
            return self._reject('rejected: land in progress')
        self._preempt_active()
        self._seed_if_needed(snap, time_s)
        assert self._p is not None
        ground = self._ground_d if self._ground_d is not None else float(snap.position_ned[2])
        self._ground_d = ground
        self._target = np.array([self._p[0], self._p[1], ground - height], dtype=float)
        self._yaw_goal = self._yaw
        self._keep_yaw = True
        self._locked_xy = self._p[:2].copy()
        return self._begin(Phase.TAKEOFF)

    def request_land(self, time_s: float, snap: Snapshot) -> int:
        if self.phase == Phase.TAKEOFF:
            return self._reject('rejected: takeoff in progress')
        self._preempt_active()
        self._seed_if_needed(snap, time_s)
        assert self._p is not None
        ground = self._ground_d if self._ground_d is not None else float(self._p[2])
        self._target = np.array([self._p[0], self._p[1], ground], dtype=float)
        self._yaw_goal = self._yaw
        self._keep_yaw = True
        self.needs_disarm = False
        return self._begin(Phase.LAND)

    def request_goto(
        self,
        time_s: float,
        snap: Snapshot,
        target_ned: np.ndarray,
        yaw: float | None,
    ) -> int:
        if self.phase in (Phase.TAKEOFF, Phase.LAND):
            return self._reject('rejected: takeoff or land in progress')
        self._seed_if_needed(snap, time_s)
        self._preempt_active()
        self._target = np.asarray(target_ned, dtype=float).reshape(3).copy()
        self._keep_yaw = yaw is None
        self._yaw_goal = self._yaw if yaw is None else float(yaw)
        return self._begin(Phase.GOTO)

    def request_turn(self, time_s: float, snap: Snapshot, delta_rad: float) -> int:
        """Turn relative to the current yaw. Positive delta is counter-clockwise
        when viewed from above (ROS), which is a negative NED yaw change.
        """
        if self.phase in (Phase.TAKEOFF, Phase.LAND):
            return self._reject('rejected: takeoff or land in progress')
        self._seed_if_needed(snap, time_s)
        self._preempt_active()
        self._yaw_goal = wrap_pi(self._yaw - delta_rad)
        moving = float(np.linalg.norm(self._v[:2])) > self.limits.speed_eps
        goal = self._begin(Phase.YAW if not moving else Phase.BRAKE)
        if moving:
            self._begin_brake(time_s, Phase.YAW)
            self._active = goal
        return goal

    def request_hold(self, time_s: float, snap: Snapshot, duration_s: float) -> int:
        if self.phase in (Phase.TAKEOFF, Phase.LAND):
            return self._reject('rejected: takeoff or land in progress')
        if duration_s < 0.0:
            return self._reject('hold duration must be >= 0')
        self._seed_if_needed(snap, time_s)
        self._preempt_active()
        goal = self._begin(Phase.BRAKE)
        self._hold_duration = float(duration_s)
        self._hold_since = None
        self._holding = True
        self._begin_brake(time_s, Phase.HOLD)
        self._active = goal
        return goal

    def note_cmd_vel(self, time_s: float, v_north: float, v_east: float, yaw_rate: float) -> None:
        if not self.accepts_cmd_vel() or self._p is None:
            return
        if self.phase != Phase.CMD_VEL:
            self._preempt_active()
            self._hold_z = float(self._p[2])
            self.phase = Phase.CMD_VEL
            self._active = None
        self._cmd_n = float(v_north)
        self._cmd_e = float(v_east)
        self._cmd_yaw_rate = float(yaw_rate)
        self._cmd_time = time_s

    def abort(self, message: str) -> None:
        if self._active is not None and not self._status[self._active].done:
            self._finish(False, message)
        self.phase = Phase.HOLD
        self._v = _zeros()
        self._a = _zeros()
        self._yaw_rate = 0.0

    def _seed_if_needed(self, snap: Snapshot, time_s: float) -> None:
        if self._p is None:
            self._seed(snap, time_s)

    def _seed(self, snap: Snapshot, time_s: float) -> None:
        self._p = np.asarray(snap.position_ned, dtype=float).reshape(3).copy()
        self._v = np.asarray(snap.velocity_ned, dtype=float).reshape(3).copy()
        self._yaw = float(snap.yaw_ned)
        self._yaw_rate = float(snap.yaw_rate)
        self._last_t = time_s
        if snap.landed:
            self._ground_d = float(self._p[2])
        if float(np.linalg.norm(self._v)) > self.limits.speed_eps or abs(self._yaw_rate) > self.limits.speed_eps:
            self._begin_brake(time_s, Phase.HOLD)

    def _begin(self, phase: Phase) -> int:
        self._goal_seq += 1
        goal_id = self._goal_seq
        self._status[goal_id] = GoalStatus(goal_id)
        self._active = goal_id
        self.phase = phase
        self._settle.reset()
        self._holding = False
        self._hold_since = None
        self.needs_disarm = False
        return goal_id

    def _reject(self, message: str) -> int:
        self._goal_seq += 1
        goal_id = self._goal_seq
        self._status[goal_id] = GoalStatus(goal_id, True, False, message)
        return goal_id

    def _preempt_active(self) -> None:
        if self._active is None:
            return
        status = self._status[self._active]
        if not status.done:
            status.done = True
            status.success = False
            status.message = 'preempted'

    def _finish(self, success: bool, message: str) -> None:
        if self._active is None:
            self.phase = Phase.HOLD
            return
        status = self._status[self._active]
        if not status.done:
            status.done = True
            status.success = success
            status.message = message
        self.phase = Phase.HOLD
        self._v = _zeros()
        self._yaw_rate = 0.0
        self._a = _zeros()

    def _begin_brake(self, time_s: float, then: Phase) -> None:
        assert self._p is not None
        self._brake = BrakeThenHold(
            self._p,
            self._v,
            self._yaw,
            self._yaw_rate,
            self.limits.a_brake,
            self.limits.yaw_accel,
        )
        self._brake_t0 = time_s
        self._after_brake = then
        self.phase = Phase.BRAKE
        self._settle.reset()

    def _tick_brake(self, time_s: float) -> None:
        assert self._brake is not None and self._p is not None
        sample = self._brake.sample(time_s - self._brake_t0)
        self._p = sample.position
        self._v = sample.velocity
        self._a = sample.acceleration
        self._yaw = sample.yaw
        self._yaw_rate = sample.yaw_rate
        if not sample.stopped:
            return
        self._v = _zeros()
        self._a = _zeros()
        self._yaw_rate = 0.0
        nxt = self._after_brake
        self._after_brake = Phase.HOLD
        self.phase = nxt
        self._settle.reset()
        if nxt == Phase.HOLD:
            self._tick_hold_timer(time_s)

    def _tick_hold_timer(self, time_s: float) -> None:
        """Count a hold only after the setpoint has been still for settle_time."""
        if not self._holding or self._active is None or self._status[self._active].done:
            return
        settled = self._settle.update(time_s, True, self.limits.settle_time)
        if not settled:
            return
        if self._hold_since is None:
            self._hold_since = time_s
        if (time_s - self._hold_since) >= self._hold_duration:
            self._holding = False
            self._finish(True, 'hold complete')

    def _cap(self, direction: np.ndarray) -> float:
        cap = self.limits.v_xy
        if self.speed_cap is not None and self._p is not None:
            extra = float(self.speed_cap(self._p, direction))
            if np.isfinite(extra):
                cap = min(cap, max(0.0, extra))
        return cap

    def _tick_path(self, time_s: float, dt: float, snap: Snapshot | None, vertical: bool) -> None:
        assert self._p is not None
        error_xy = self._target[:2] - self._p[:2]
        direction = error_xy
        v_xy = self._cap(direction) if not vertical else min(self.limits.v_xy, 0.4)
        pos_xy, vel_xy, acc_xy = step_vector2(self._p[:2], self._v[:2], self._target[:2], v_xy, self.limits.a_max, dt)
        z = step_scalar(float(self._p[2]), float(self._v[2]), float(self._target[2]), self.limits.v_z, self.limits.a_max, dt)
        self._p = np.array([pos_xy[0], pos_xy[1], z.position], dtype=float)
        self._v = np.array([vel_xy[0], vel_xy[1], z.velocity], dtype=float)
        self._a = np.array([acc_xy[0], acc_xy[1], z.acceleration], dtype=float)
        self._step_yaw_toward(dt, self._yaw if self._yaw_goal is None else self._yaw_goal)
        if snap is None:
            return
        pos_err = float(np.linalg.norm(snap.position_ned - self._target))
        yaw_err = 0.0 if self._yaw_goal is None else abs(wrap_pi(self._yaw_goal - snap.yaw_ned))
        slow = float(np.linalg.norm(snap.velocity_ned)) < self.limits.speed_eps
        if self._settle.update(time_s, pos_err <= self.limits.settle_pos and yaw_err <= self.limits.settle_yaw and slow, self.limits.settle_time):
            self._p = self._target.copy()
            self._finish(True, 'settled')

    def _tick_land(self, time_s: float, dt: float, snap: Snapshot | None) -> None:
        if snap is not None and snap.landed:
            self._p = np.asarray(snap.position_ned, dtype=float).reshape(3).copy()
            self._v = _zeros()
            self.needs_disarm = True
            self._finish(True, 'landed')
            return
        self._tick_path(time_s, dt, snap, vertical=True)
        if self.phase == Phase.HOLD:
            self.needs_disarm = True

    def _tick_yaw(self, time_s: float, dt: float, snap: Snapshot | None) -> None:
        assert self._p is not None and self._yaw_goal is not None
        self._v = _zeros()
        self._a = _zeros()
        self._step_yaw_toward(dt, self._yaw_goal)
        if snap is None:
            return
        xy_err = float(np.linalg.norm(snap.position_ned[:2] - self._p[:2]))
        z_err = abs(float(snap.position_ned[2] - self._p[2]))
        yaw_err = abs(wrap_pi(self._yaw_goal - snap.yaw_ned))
        slow = abs(snap.yaw_rate) < self.limits.speed_eps and float(np.linalg.norm(snap.velocity_ned)) < self.limits.speed_eps
        ok = xy_err <= self.limits.settle_pos and z_err <= self.limits.settle_pos and yaw_err <= self.limits.settle_yaw and slow
        if self._settle.update(time_s, ok, self.limits.settle_time):
            self._yaw = self._yaw_goal
            self._yaw_rate = 0.0
            self._finish(True, 'yaw settled')

    def _tick_cmd(self, dt: float) -> None:
        assert self._p is not None
        if dt <= 0.0:
            return
        desired = np.array([self._cmd_n, self._cmd_e], dtype=float)
        speed = float(np.linalg.norm(desired))
        if speed > self.limits.v_xy > 0.0:
            desired *= self.limits.v_xy / speed
        current = self._v[:2].copy()
        delta = desired - current
        limit = self.limits.a_max * dt
        norm = float(np.linalg.norm(delta))
        if norm > limit > 0.0:
            delta *= limit / norm
        self._v[:2] = current + delta
        self._a[:2] = (self._v[:2] - current) / dt
        self._p[:2] = self._p[:2] + self._v[:2] * dt
        z_goal = float(self._hold_z if self._hold_z is not None else self._p[2])
        z = step_scalar(float(self._p[2]), float(self._v[2]), z_goal, self.limits.v_z, self.limits.a_max, dt)
        self._p[2] = z.position
        self._v[2] = z.velocity
        self._a[2] = z.acceleration
        rate_delta = self._cmd_yaw_rate - self._yaw_rate
        rate_step = max(-self.limits.yaw_accel * dt, min(self.limits.yaw_accel * dt, rate_delta))
        self._yaw_rate += rate_step
        self._yaw = wrap_pi(self._yaw + self._yaw_rate * dt)

    def _step_yaw_toward(self, dt: float, goal: float) -> None:
        yaw, rate, accel = step_yaw(self._yaw, self._yaw_rate, goal, self.limits.yaw_rate, self.limits.yaw_accel, dt)
        self._yaw = yaw
        self._yaw_rate = rate
        # Yaw acceleration is not part of the linear acceleration vector.

    def _emit(self) -> Setpoint:
        assert self._p is not None
        return Setpoint(
            position=self._p.copy(),
            velocity=self._v.copy(),
            acceleration=self._a.copy(),
            yaw=float(self._yaw),
            yaw_rate=float(self._yaw_rate),
            use_position=True,
        )
