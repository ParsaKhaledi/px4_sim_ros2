"""Acceleration-limited setpoint steps.

These samplers advance a position setpoint by one control period. They do not
replace it with the goal, so PX4 receives a continuous position plus a
velocity and acceleration feedforward.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from px4_control.frames import wrap_pi


def _clamp(value: float, limit: float) -> float:
    if limit < 0.0:
        raise ValueError('limit must be non-negative')
    return max(-limit, min(limit, value))


@dataclass
class ScalarStep:
    position: float
    velocity: float
    acceleration: float


def _stopping_speed(distance: float, a_max: float, dt: float) -> float:
    """Speed that can stop in ``distance`` without exceeding ``a_max``.

    The extra ``(a dt / 2)`` term is the discrete step. ``sqrt(2 a d)`` alone
    arrives still moving, and the next tick then snaps velocity to zero.
    """
    if distance <= 0.0:
        return 0.0
    half_step = a_max * dt * 0.5
    return max(0.0, math.sqrt(2.0 * a_max * distance + half_step * half_step) - half_step)


def step_scalar(
    position: float,
    velocity: float,
    goal: float,
    v_max: float,
    a_max: float,
    dt: float,
) -> ScalarStep:
    """Advance one axis toward ``goal`` with a trapezoidal speed limit.

    Desired speed is the stopping speed at the remaining distance, capped by
    ``v_max``. Acceleration feedforward stays inside ``a_max``.
    """
    if v_max < 0.0 or a_max <= 0.0:
        raise ValueError('v_max must be >= 0 and a_max must be > 0')
    if dt <= 0.0:
        return ScalarStep(position, velocity, 0.0)
    error = goal - position
    if abs(error) < 1e-4 and abs(velocity) < 1e-3:
        return ScalarStep(goal, 0.0, 0.0)
    v_des_mag = min(v_max, _stopping_speed(abs(error), a_max, dt))
    v_des = math.copysign(v_des_mag, error) if abs(error) > 1e-9 else 0.0
    accel = _clamp((v_des - velocity) / dt, a_max)
    velocity_new = velocity + accel * dt
    position_new = position + velocity_new * dt
    if error * (goal - position_new) < 0.0:
        velocity_land = error / dt
        accel_land = (velocity_land - velocity) / dt
        if abs(accel_land) <= a_max:
            return ScalarStep(goal, velocity_land, accel_land)
        accel = _clamp(accel_land, a_max)
        velocity_new = velocity + accel * dt
        position_new = position + velocity_new * dt
        if error * (goal - position_new) < 0.0:
            position_new = goal
    return ScalarStep(position_new, velocity_new, accel)


def step_vector2(
    position: np.ndarray,
    velocity: np.ndarray,
    goal: np.ndarray,
    v_max: float,
    a_max: float,
    dt: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Straight-line planar step. Lateral velocity is braked, not ignored."""
    pos = np.asarray(position, dtype=float).reshape(2).copy()
    vel = np.asarray(velocity, dtype=float).reshape(2).copy()
    target = np.asarray(goal, dtype=float).reshape(2)
    if dt <= 0.0:
        return pos, vel, np.zeros(2)
    error = target - pos
    distance = float(np.linalg.norm(error))
    if distance < 1e-4 and float(np.linalg.norm(vel)) < 1e-3:
        return target.copy(), np.zeros(2), np.zeros(2)
    direction = error / distance if distance > 1e-9 else np.zeros(2)
    speed_along = float(np.dot(vel, direction))
    lateral = vel - direction * speed_along
    v_des = min(v_max, _stopping_speed(distance, a_max, dt))
    along_accel = _clamp((v_des - speed_along) / dt, a_max)
    lateral_norm = float(np.linalg.norm(lateral))
    if lateral_norm > 1e-9:
        lateral_accel = -lateral * (min(a_max, lateral_norm / dt) / lateral_norm)
    else:
        lateral_accel = np.zeros(2)
    acceleration = direction * along_accel + lateral_accel
    accel_norm = float(np.linalg.norm(acceleration))
    if accel_norm > a_max:
        acceleration = acceleration * (a_max / accel_norm)
    velocity_new = vel + acceleration * dt
    position_new = pos + velocity_new * dt
    if distance > 1e-6 and float(np.dot(target - position_new, direction)) < 0.0:
        # Land on the goal along track when one step can do it inside a_max.
        # Otherwise keep braking; do not snap the velocity to zero.
        along_error = distance
        speed_land = along_error / dt
        accel_land = _clamp((speed_land - speed_along) / dt, a_max)
        acceleration = direction * accel_land + lateral_accel
        accel_norm = float(np.linalg.norm(acceleration))
        if accel_norm > a_max:
            acceleration = acceleration * (a_max / accel_norm)
        velocity_new = vel + acceleration * dt
        position_new = pos + velocity_new * dt
        if float(np.dot(target - position_new, direction)) < 0.0:
            position_new = target.copy()
    return position_new, velocity_new, acceleration


def step_yaw_remaining(
    yaw: float,
    yaw_rate: float,
    remaining: float,
    rate_max: float,
    accel_max: float,
    dt: float,
) -> tuple[float, float, float, float]:
    """Advance yaw by a signed remaining angle.

    ``remaining`` is not wrapped. A turn of +pi and a turn of -pi stay on
    opposite paths. The returned remaining is what is left after this step.
    """
    stepped = step_scalar(0.0, yaw_rate, remaining, rate_max, accel_max, dt)
    if dt <= 0.0:
        return yaw, yaw_rate, 0.0, remaining
    consumed = stepped.position
    return wrap_pi(yaw + consumed), stepped.velocity, stepped.acceleration, remaining - consumed


def step_yaw(
    yaw: float,
    yaw_rate: float,
    goal: float,
    rate_max: float,
    accel_max: float,
    dt: float,
) -> tuple[float, float, float]:
    """Ramp yaw along the short path. ``rate_max`` and ``accel_max`` are rad/s."""
    stepped = step_scalar(0.0, yaw_rate, wrap_pi(goal - yaw), rate_max, accel_max, dt)
    # step_scalar integrates a local error coordinate that starts at 0.
    yaw_new = wrap_pi(yaw + stepped.position) if dt > 0.0 else yaw
    if dt <= 0.0:
        return yaw, yaw_rate, 0.0
    # Recompute from the wrapped delta so a goal crossing lands on the goal.
    if abs(wrap_pi(goal - yaw_new)) < 1e-4 and abs(stepped.velocity) < 1e-3:
        return wrap_pi(goal), 0.0, stepped.acceleration
    return yaw_new, stepped.velocity, stepped.acceleration


@dataclass
class BrakeSample:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    yaw: float
    yaw_rate: float
    stopped: bool


class BrakeThenHold:
    """Decelerate to a stop, then latch the stop position.

    The position setpoint at t=0 is the incoming setpoint, not a new hold
    point captured while the vehicle is still moving.
    """

    def __init__(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        yaw: float,
        yaw_rate: float,
        a_brake: float,
        yaw_accel: float,
    ) -> None:
        if a_brake <= 0.0 or yaw_accel <= 0.0:
            raise ValueError('brake accelerations must be positive')
        self.p0 = np.asarray(position, dtype=float).reshape(3).copy()
        self.v0 = np.asarray(velocity, dtype=float).reshape(3).copy()
        self.yaw0 = float(yaw)
        self.yaw_rate0 = float(yaw_rate)
        self.a_brake = float(a_brake)
        self.yaw_accel = float(yaw_accel)
        speed = float(np.linalg.norm(self.v0))
        if speed < 1e-4:
            self.direction = np.zeros(3)
            self.t_stop = 0.0
            self.p_latch = self.p0.copy()
        else:
            self.direction = self.v0 / speed
            self.t_stop = speed / self.a_brake
            self.p_latch = self.p0 + self.v0 * (self.t_stop * 0.5)
        self.t_yaw = abs(self.yaw_rate0) / self.yaw_accel

    def sample(self, elapsed: float) -> BrakeSample:
        t = max(0.0, elapsed)
        if t >= self.t_stop:
            position = self.p_latch.copy()
            velocity = np.zeros(3)
            acceleration = np.zeros(3)
            linear_stopped = True
        else:
            speed = float(np.linalg.norm(self.v0)) - self.a_brake * t
            velocity = self.direction * speed
            position = self.p0 + self.v0 * t - 0.5 * self.a_brake * self.direction * t * t
            acceleration = -self.a_brake * self.direction
            linear_stopped = False
        if t >= self.t_yaw:
            yaw = wrap_pi(self.yaw0 + math.copysign(0.5 * self.yaw_rate0 * self.t_yaw, self.yaw_rate0))
            if self.t_yaw == 0.0:
                yaw = self.yaw0
            yaw_rate = 0.0
            yaw_stopped = True
        else:
            sign = math.copysign(1.0, self.yaw_rate0) if self.yaw_rate0 != 0.0 else 0.0
            yaw_rate = self.yaw_rate0 - sign * self.yaw_accel * t
            yaw = wrap_pi(self.yaw0 + self.yaw_rate0 * t - 0.5 * sign * self.yaw_accel * t * t)
            yaw_stopped = False
        return BrakeSample(
            position=position,
            velocity=velocity,
            acceleration=acceleration,
            yaw=yaw,
            yaw_rate=yaw_rate,
            stopped=linear_stopped and yaw_stopped,
        )
