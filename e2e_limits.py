"""Shared E2E pass/fail limits.

The offline log tool and the in-flight grader (``tests/e2e/grading.py``, once
that file is on the branch) both import this module so each limit is named
in one place. This file stores names only. Numbers come from the process
environment, then ``.env`` at the repo root. ``.env.example`` is not a
fallback: it is checked in separately so its ``E2E_*`` entries stay equal
to ``.env``.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from dataclasses import dataclass
from dataclasses import fields
from pathlib import Path


# (environment name, dataclass field, "float" or "int")
# The names match the flight-test env file. Do not put numbers next to them.
LIMIT_FIELDS: tuple[tuple[str, str, str], ...] = (
    ("E2E_TAKEOFF_HEIGHT_M", "takeoff_height_m", "float"),
    ("E2E_HOVER_S", "hover_s", "float"),
    ("E2E_LEG_LENGTH_M", "leg_length_m", "float"),
    ("E2E_HOVER_DRIFT_M", "hover_drift_m", "float"),
    ("E2E_LEG_TOLERANCE_M", "leg_tolerance_m", "float"),
    ("E2E_OVERSHOOT_M", "overshoot_m", "float"),
    ("E2E_SETTLE_TOLERANCE_M", "settle_tolerance_m", "float"),
    ("E2E_SETTLE_HOLD_S", "settle_hold_s", "float"),
    ("E2E_SETTLE_TIMEOUT_S", "settle_timeout_s", "float"),
    ("E2E_YAW_TOLERANCE_DEG", "yaw_tolerance_deg", "float"),
    ("E2E_YAW_SETTLE_DEG", "yaw_settle_deg", "float"),
    ("E2E_HOVER_HEIGHT_BAND_M", "hover_height_band_m", "float"),
    ("E2E_HOVER_HEIGHT_HOLD_S", "hover_height_hold_s", "float"),
    ("E2E_RETURN_TOLERANCE_M", "return_tolerance_m", "float"),
    ("E2E_HEIGHT_TOLERANCE_M", "height_tolerance_m", "float"),
    ("E2E_MAX_RETRIES", "max_retries", "int"),
    ("E2E_CRASH_TILT_DEG", "crash_tilt_deg", "float"),
    ("E2E_CRASH_MIN_HEIGHT_M", "crash_min_height_m", "float"),
    ("E2E_CRASH_IMPACT_SPEED_MPS", "crash_impact_speed_mps", "float"),
    ("E2E_CRASH_DIVERGENCE_M", "crash_divergence_m", "float"),
    ("E2E_ODOM_TIMEOUT_S", "odom_timeout_s", "float"),
)

LIMIT_NAMES: tuple[str, ...] = tuple(name for name, _field, _kind in LIMIT_FIELDS)


class E2ELimitError(RuntimeError):
    """A required limit is missing or is not a usable number."""


@dataclass(frozen=True)
class E2ELimits:
    """One loaded value per E2E limit. Fields have no defaults."""

    takeoff_height_m: float
    hover_s: float
    leg_length_m: float
    hover_drift_m: float
    leg_tolerance_m: float
    overshoot_m: float
    settle_tolerance_m: float
    settle_hold_s: float
    settle_timeout_s: float
    yaw_tolerance_deg: float
    yaw_settle_deg: float
    hover_height_band_m: float
    hover_height_hold_s: float
    return_tolerance_m: float
    height_tolerance_m: float
    max_retries: int
    crash_tilt_deg: float
    crash_min_height_m: float
    crash_impact_speed_mps: float
    crash_divergence_m: float
    odom_timeout_s: float


def repo_root() -> Path:
    """Return the repository root (the directory that contains this file)."""

    return Path(__file__).resolve().parent


def parse_env_file(path: Path) -> dict[str, str]:
    """Read ``KEY=VALUE`` lines. A missing file yields an empty mapping."""

    if not path.is_file():
        return {}
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.startswith("export "):
            stripped = stripped[len("export ") :].strip()
        if "=" not in stripped:
            continue
        key, raw = stripped.split("=", 1)
        key = key.strip()
        value = raw.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in {'"', "'"}:
            value = value[1:-1]
        else:
            value = value.split("#", 1)[0].strip()
        if key:
            values[key] = value
    return values


def load_limits(
    environ: Mapping[str, str] | None = None,
    env_path: Path | None = None,
    root: Path | None = None,
) -> E2ELimits:
    """Load every limit. Process environment wins, then ``.env``.

    A name that is absent from both stops the load. The error lists every
    missing ``E2E_*`` key. ``.env.example`` is never read here.
    """

    base = root if root is not None else repo_root()
    dotenv = env_path if env_path is not None else base / ".env"
    process = os.environ if environ is None else environ

    # ``.env`` first, so the process environment overwrites it.
    resolved: dict[str, tuple[str, str]] = {}
    _fill(parse_env_file(dotenv), str(dotenv), resolved)
    _fill(process, "process environment", resolved)

    missing = [name for name, _field, _kind in LIMIT_FIELDS if name not in resolved]
    if missing:
        names = ", ".join(missing)
        raise E2ELimitError(
            f"E2E limits missing from the process environment and {dotenv}: {names}"
        )

    kwargs: dict[str, float | int] = {}
    for name, field, kind in LIMIT_FIELDS:
        raw, source = resolved[name]
        kwargs[field] = _parse_value(name, raw, source, kind)
    return E2ELimits(**kwargs)


def limit_field_names() -> tuple[str, ...]:
    """Return the dataclass field names in the same order as ``LIMIT_FIELDS``."""

    return tuple(field.name for field in fields(E2ELimits))


def _fill(values: Mapping[str, str], source: str, into: dict[str, tuple[str, str]]) -> None:
    """Copy non-empty limit strings into ``into``."""

    for name, _field, _kind in LIMIT_FIELDS:
        raw = values.get(name)
        if raw is None:
            continue
        text = str(raw).strip()
        if text == "":
            continue
        into[name] = (text, source)


def _parse_value(name: str, raw: str, source: str, kind: str) -> float | int:
    """Parse one limit and reject blank, non-numeric, or negative values."""

    try:
        number = float(raw)
    except ValueError as exc:
        raise E2ELimitError(f"{name} from {source} must be a number, got {raw!r}") from exc
    if number != number or number in {float("inf"), float("-inf")}:
        raise E2ELimitError(f"{name} from {source} must be finite, got {raw!r}")
    if number < 0:
        raise E2ELimitError(f"{name} from {source} must be >= 0, got {raw!r}")
    if kind == "int":
        if not number.is_integer():
            raise E2ELimitError(f"{name} from {source} must be a whole number, got {raw!r}")
        return int(number)
    return number
