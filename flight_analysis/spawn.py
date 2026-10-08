"""Spawn pose and clock offset for a TUM ground-truth file.

The default file is ``spawn.json`` in the same directory as the ``.ulg``
(``logs/<run_id>/spawn.json``). ``--spawn-json`` points at another path.
``--spawn-xyz``, ``--spawn-yaw``, and ``--clock-offset`` override one field
each when they are present.

``px4_offset_s`` is ROS sim time minus PX4 boot time:

    t_px4 = t_ros - px4_offset_s

``trajectory_eval record`` may also write ``px4_offset_spread_s`` (median
gap minus minimum gap, seconds), ``px4_offset_samples``, and, when the
offset is JSON null, ``px4_offset_reason``. A null offset is not a clock.
Unknown keys are ignored.

A missing pose is an error. A missing clock falls back to the climb-edge
estimate in ``tum.py``. The identity transform is never assumed.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path


# The recorder's offset is a spread of gaps. Wider than this, or fewer
# samples than this, and the value is still used but the run warns.
OFFSET_SPREAD_WARN_S = 0.02
OFFSET_SAMPLES_WARN = 50


class SpawnError(RuntimeError):
    """Ground truth was given without a usable spawn pose or clock file."""


@dataclass(frozen=True)
class ResolvedSpawn:
    """The pose and optional clock chosen for one TUM file.

    ``xyz_source`` and ``yaw_source`` are ``spawn_json`` or ``cli``.
    ``clock_source`` is ``spawn_json`` or ``cli`` when ``px4_offset_s`` is
    set, and ``None`` when the caller should estimate the climb edge.
    ``offset_is_null`` means the file contained ``px4_offset_s: null``.
    Spread and sample count are copied from the file when those keys exist.
    """

    xyz_m: tuple[float, float, float]
    yaw_rad: float
    xyz_source: str
    yaw_source: str
    px4_offset_s: float | None
    clock_source: str | None
    px4_offset_reason: str | None = None
    px4_offset_spread_s: float | None = None
    px4_offset_samples: int | None = None
    offset_is_null: bool = False


def resolve_spawn(
    log_path: Path | None,
    spawn_json: Path | None,
    spawn_xyz: tuple[float, float, float] | None,
    spawn_yaw: float | None,
    clock_offset_s: float | None,
) -> ResolvedSpawn:
    """Choose the spawn pose and, when one was given, the clock offset.

    The file beside the log is used unless ``spawn_json`` names another
    path. A CLI value replaces the matching file field. Pose fields that
    are still missing raise ``SpawnError`` instead of using the origin.
    """

    file_path = _select_file(log_path, spawn_json)
    file_values = _read_file(file_path) if file_path is not None else {}
    file_xyz = _xyz_from_file(file_path, file_values)
    file_yaw = _yaw_from_file(file_path, file_values)
    file_clock = _clock_from_file(file_path, file_values)

    xyz, xyz_source = _prefer(spawn_xyz, "cli", file_xyz, "spawn_json")
    yaw, yaw_source = _prefer(spawn_yaw, "cli", file_yaw, "spawn_json")
    missing = []
    if xyz is None:
        missing.append("spawn_xyz (or --spawn-xyz X Y Z)")
    if yaw is None:
        missing.append("spawn_yaw (or --spawn-yaw RAD)")
    if missing:
        if file_path is None:
            looked = _missing_file_note(log_path, spawn_json)
        else:
            looked = f"{file_path} does not supply them"
        names = ", ".join(missing)
        raise SpawnError(
            "ground truth needs the Gazebo spawn pose before the TUM file "
            f"can be moved into the takeoff frame; {looked}; missing {names}. "
            "Refusing to assume the identity transform."
        )

    clock, clock_source = _prefer(clock_offset_s, "cli", file_clock.px4_offset_s, "spawn_json")
    return ResolvedSpawn(
        xyz_m=(float(xyz[0]), float(xyz[1]), float(xyz[2])),
        yaw_rad=float(yaw),
        xyz_source=str(xyz_source),
        yaw_source=str(yaw_source),
        px4_offset_s=None if clock is None else float(clock),
        clock_source=clock_source,
        px4_offset_reason=file_clock.reason,
        px4_offset_spread_s=file_clock.spread_s,
        px4_offset_samples=file_clock.samples,
        offset_is_null=file_clock.explicit_null,
    )


def _select_file(log_path: Path | None, spawn_json: Path | None) -> Path | None:
    """Return the spawn file to read, or ``None`` when there is no file."""

    if spawn_json is not None:
        path = Path(spawn_json)
        if not path.is_file():
            raise SpawnError(f"--spawn-json not found: {path}")
        return path
    if log_path is None:
        return None
    candidate = Path(log_path).parent / "spawn.json"
    if candidate.is_file():
        return candidate
    return None


def _missing_file_note(log_path: Path | None, spawn_json: Path | None) -> str:
    """Describe where a spawn file was expected."""

    if spawn_json is not None:
        return f"--spawn-json {spawn_json} was not used"
    if log_path is None:
        return "no log path was given, so spawn.json could not be found beside it"
    return f"no spawn.json next to {log_path}"


def _read_file(path: Path) -> dict[str, object]:
    """Parse ``spawn.json``. The top level must be an object."""

    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SpawnError(f"{path} is not JSON: {exc}") from exc
    except OSError as exc:
        raise SpawnError(f"could not read {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise SpawnError(f"{path} must be a JSON object")
    return data


def _xyz_from_file(
    path: Path | None,
    data: dict[str, object],
) -> tuple[float, float, float] | None:
    """Return ``spawn_xyz`` when the file has it."""

    if "spawn_xyz" not in data:
        return None
    raw = data["spawn_xyz"]
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        raise SpawnError(f"{path} spawn_xyz must be [x, y, z] in metres")
    try:
        return (float(raw[0]), float(raw[1]), float(raw[2]))
    except (TypeError, ValueError) as exc:
        raise SpawnError(f"{path} spawn_xyz must be three numbers") from exc


def _yaw_from_file(path: Path | None, data: dict[str, object]) -> float | None:
    """Return ``spawn_yaw`` in radians when the file has it."""

    return _optional_number(path, data, "spawn_yaw", "a yaw in radians")


@dataclass(frozen=True)
class FileClock:
    """Clock fields copied from ``spawn.json``. Missing keys stay ``None``.

    ``explicit_null`` is true when ``px4_offset_s`` is present and JSON null.
    That is not a clock: the climb-edge estimate is used, and ``reason`` is
    the recorder's explanation.
    """

    px4_offset_s: float | None
    explicit_null: bool
    reason: str | None
    spread_s: float | None
    samples: int | None


def _clock_from_file(path: Path | None, data: dict[str, object]) -> FileClock:
    """Read the recorder's clock fields. Other keys in the file are ignored.

    ``px4_offset_s`` is ROS sim time minus PX4 boot time. JSON null means
    PX4 was not publishing. ``px4_offset_spread_s`` is the median gap minus
    the minimum gap, in seconds. ``px4_offset_samples`` is the sample count.
    ``px4_offset_reason`` is the string that explains a null offset.
    """

    offset, explicit_null = _offset_value(path, data)
    return FileClock(
        px4_offset_s=offset,
        explicit_null=explicit_null,
        reason=_reason(path, data),
        spread_s=_spread(path, data),
        samples=_samples(path, data),
    )


def _offset_value(path: Path | None, data: dict[str, object]) -> tuple[float | None, bool]:
    """Return ``(seconds, was_null)`` for ``px4_offset_s``."""

    if "px4_offset_s" not in data:
        return None, False
    raw = data["px4_offset_s"]
    if raw is None:
        return None, True
    return (
        _finite_number(path, "px4_offset_s", raw, "seconds of ROS sim time minus PX4 boot time"),
        False,
    )


def _reason(path: Path | None, data: dict[str, object]) -> str | None:
    """Return ``px4_offset_reason`` when it is a non-empty string."""

    if "px4_offset_reason" not in data or data["px4_offset_reason"] is None:
        return None
    raw = data["px4_offset_reason"]
    if not isinstance(raw, str):
        raise SpawnError(f"{path} px4_offset_reason must be a string")
    text = raw.strip()
    return text or None


def _spread(path: Path | None, data: dict[str, object]) -> float | None:
    """Return ``px4_offset_spread_s`` in seconds, or ``None`` when absent."""

    if "px4_offset_spread_s" not in data or data["px4_offset_spread_s"] is None:
        return None
    return _finite_number(
        path,
        "px4_offset_spread_s",
        data["px4_offset_spread_s"],
        "seconds, the median gap minus the minimum gap",
    )


def _samples(path: Path | None, data: dict[str, object]) -> int | None:
    """Return ``px4_offset_samples`` as an integer, or ``None`` when absent."""

    if "px4_offset_samples" not in data or data["px4_offset_samples"] is None:
        return None
    raw = data["px4_offset_samples"]
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise SpawnError(f"{path} px4_offset_samples must be an integer")
    number = float(raw)
    if not math.isfinite(number) or not number.is_integer():
        raise SpawnError(f"{path} px4_offset_samples must be an integer")
    return int(number)


def _finite_number(path: Path | None, key: str, raw: object, detail: str) -> float:
    """Parse one finite JSON number."""

    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise SpawnError(f"{path} {key} must be {detail}")
    number = float(raw)
    if not math.isfinite(number):
        raise SpawnError(f"{path} {key} must be finite")
    return number


def clock_quality_warnings(spread_s: float | None, samples: int | None) -> list[str]:
    """Warn when the recorder's offset estimate is wide or thinly sampled.

    The spread is the median gap minus the minimum gap. A spread over
    0.02 s, or fewer than 50 samples, is reported. The offset is still used.
    """

    messages: list[str] = []
    if spread_s is not None and spread_s > OFFSET_SPREAD_WARN_S:
        messages.append(
            f"px4_offset_spread_s is {spread_s:.4f} s, over {OFFSET_SPREAD_WARN_S:.2f} s "
            "(median gap minus the minimum gap)"
        )
    if samples is not None and samples < OFFSET_SAMPLES_WARN:
        messages.append(f"px4_offset_samples is {samples}, fewer than {OFFSET_SAMPLES_WARN}")
    return messages


def _optional_number(
    path: Path | None,
    data: dict[str, object],
    key: str,
    detail: str,
) -> float | None:
    """Return one numeric field, or ``None`` when the key is absent."""

    if key not in data:
        return None
    raw = data[key]
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise SpawnError(f"{path} {key} must be {detail}")
    return float(raw)


def _prefer(
    override: float | tuple[float, float, float] | None,
    override_source: str,
    fallback: float | tuple[float, float, float] | None,
    fallback_source: str,
) -> tuple[float | tuple[float, float, float] | None, str | None]:
    """Use the CLI value when it was passed, otherwise the file value."""

    if override is not None:
        return override, override_source
    if fallback is not None:
        return fallback, fallback_source
    return None, None
