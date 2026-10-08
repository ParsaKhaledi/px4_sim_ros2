"""Build FlightLog objects from plain arrays. No ULog and no pyulog."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from e2e_limits import E2ELimits
from e2e_limits import load_limits
from e2e_limits import parse_env_file
from flight_analysis.log import FlightLog
from flight_analysis.log import flight_log_from_arrays


FIXTURE = Path(__file__).resolve().parent / "fixtures" / "e2e.env"


def load_fixture_limits() -> E2ELimits:
    """Load limits from the test fixture, ignoring the developer environment."""

    return load_limits(environ={}, env_path=FIXTURE, example_path=FIXTURE)


def fixture_values() -> dict[str, str]:
    """Return the fixture file as a string mapping."""

    return parse_env_file(FIXTURE)


def times(start: float, end: float, dt: float = 0.05) -> np.ndarray:
    """Inclusive time grid from ``start`` to ``end``."""

    count = int(round((end - start) / dt)) + 1
    return start + np.arange(count) * dt


def flight_log(topics: dict[str, dict[str, np.ndarray]], parameters: dict[str, float] | None = None) -> FlightLog:
    """Wrap topic dictionaries. Timestamps are still microseconds."""

    return flight_log_from_arrays(parameters or {}, topics)


def microseconds(time_s: np.ndarray) -> np.ndarray:
    """Convert sim seconds to the integer microseconds a ULog would store."""

    return np.rint(time_s * 1e6).astype(np.int64)


def level_quaternion(count: int, roll_rad: float = 0.0) -> dict[str, np.ndarray]:
    """Body-to-NED quaternion samples for a pure roll (zero is level)."""

    half = roll_rad / 2.0
    return {
        "q[0]": np.full(count, math.cos(half)),
        "q[1]": np.full(count, math.sin(half)),
        "q[2]": np.zeros(count),
        "q[3]": np.zeros(count),
    }
