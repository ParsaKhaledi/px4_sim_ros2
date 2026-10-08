"""PX4 custom-mode codes and the offboard proof-of-life gate.

PX4 accepts an offboard switch only after ``OffboardControlMode`` has been
streaming at 2 Hz or more for about a second. A same-timestamp burst does
not satisfy that window.
"""

from __future__ import annotations

from dataclasses import dataclass

# MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
CUSTOM_MODE_ENABLED = 1.0

# PX4_CUSTOM_MAIN_MODE_*
MAIN_MANUAL = 1.0
MAIN_ALTCTL = 2.0
MAIN_POSCTL = 3.0
MAIN_AUTO = 4.0
MAIN_OFFBOARD = 6.0
MAIN_STABILIZED = 7.0

# PX4_CUSTOM_SUB_MODE_AUTO_*
SUB_AUTO_TAKEOFF = 2.0
SUB_AUTO_LOITER = 3.0
SUB_AUTO_LAND = 6.0

# VehicleStatus.NAVIGATION_STATE_*
NAV_BY_MODE = {
    'MANUAL': 0,
    'ALTCTL': 1,
    'POSCTL': 2,
    'AUTO_LOITER': 4,
    'OFFBOARD': 14,
    'STABILIZED': 15,
    'AUTO_TAKEOFF': 17,
    'AUTO_LAND': 18,
}

NAV_NAMES = {value: name for name, value in NAV_BY_MODE.items()}


@dataclass(frozen=True)
class ModeCommand:
    name: str
    param1: float
    param2: float
    param3: float
    nav_state: int


_MODES: dict[str, ModeCommand] = {
    'MANUAL': ModeCommand('MANUAL', CUSTOM_MODE_ENABLED, MAIN_MANUAL, 0.0, NAV_BY_MODE['MANUAL']),
    'ALTCTL': ModeCommand('ALTCTL', CUSTOM_MODE_ENABLED, MAIN_ALTCTL, 0.0, NAV_BY_MODE['ALTCTL']),
    'POSCTL': ModeCommand('POSCTL', CUSTOM_MODE_ENABLED, MAIN_POSCTL, 0.0, NAV_BY_MODE['POSCTL']),
    'STABILIZED': ModeCommand('STABILIZED', CUSTOM_MODE_ENABLED, MAIN_STABILIZED, 0.0, NAV_BY_MODE['STABILIZED']),
    'OFFBOARD': ModeCommand('OFFBOARD', CUSTOM_MODE_ENABLED, MAIN_OFFBOARD, 0.0, NAV_BY_MODE['OFFBOARD']),
    'AUTO_LOITER': ModeCommand('AUTO_LOITER', CUSTOM_MODE_ENABLED, MAIN_AUTO, SUB_AUTO_LOITER, NAV_BY_MODE['AUTO_LOITER']),
    'AUTO_TAKEOFF': ModeCommand('AUTO_TAKEOFF', CUSTOM_MODE_ENABLED, MAIN_AUTO, SUB_AUTO_TAKEOFF, NAV_BY_MODE['AUTO_TAKEOFF']),
    'AUTO_LAND': ModeCommand('AUTO_LAND', CUSTOM_MODE_ENABLED, MAIN_AUTO, SUB_AUTO_LAND, NAV_BY_MODE['AUTO_LAND']),
}


def parse_mode(name: str) -> ModeCommand:
    key = name.strip().upper().replace('-', '_').replace(' ', '_')
    aliases = {
        'POSITION': 'POSCTL',
        'POS': 'POSCTL',
        'ALTITUDE': 'ALTCTL',
        'HOLD': 'AUTO_LOITER',
        'LOITER': 'AUTO_LOITER',
        'LAND': 'AUTO_LAND',
        'TAKEOFF': 'AUTO_TAKEOFF',
    }
    key = aliases.get(key, key)
    if key not in _MODES:
        known = ', '.join(sorted(_MODES))
        raise ValueError(f'unknown mode {name!r}; expected one of {known}')
    return _MODES[key]


class OffboardStreamGate:
    """Track whether the setpoint stream has been alive long enough."""

    def __init__(self, min_hz: float = 2.0, warmup_s: float = 1.0, stale_s: float = 0.5) -> None:
        if min_hz <= 0.0 or warmup_s < 0.0:
            raise ValueError('min_hz must be positive and warmup_s non-negative')
        self.min_hz = float(min_hz)
        self.warmup_s = float(warmup_s)
        self.stale_s = float(stale_s)
        self._t0: float | None = None
        self._last: float | None = None
        self._count = 0

    def reset(self) -> None:
        self._t0 = None
        self._last = None
        self._count = 0

    def tick(self, time_s: float) -> None:
        """Record one published heartbeat. Duplicate timestamps do not count."""
        if self._t0 is None or self._last is None:
            self._t0 = time_s
            self._last = time_s
            self._count = 1
            return
        if time_s < self._last - 1e-6:
            self._t0 = time_s
            self._last = time_s
            self._count = 1
            return
        if time_s > self._last + 1e-6:
            self._count += 1
            self._last = time_s

    def ready(self, time_s: float) -> bool:
        if self._t0 is None or self._last is None or self._count < 2:
            return False
        if time_s - self._last > self.stale_s:
            return False
        elapsed = time_s - self._t0
        if elapsed < self.warmup_s:
            return False
        rate = (self._count - 1) / elapsed
        return rate >= self.min_hz

    def reason(self, time_s: float) -> str | None:
        if self.ready(time_s):
            return None
        return (
            'offboard stream is not warm: PX4 needs OffboardControlMode at '
            f'>={self.min_hz:.0f} Hz for {self.warmup_s:.1f} s before a mode switch'
        )
