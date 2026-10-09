"""spawn.json written by ``trajectory_eval record``.

The file has these fields and no others:

* ``spawn_xyz`` and ``spawn_yaw`` from the live ``world`` -> ``spawn`` transform
  (ENU metres and yaw radians)
* ``px4_offset_s``, the lowest ``t_ros - t_px4`` gap between ``/clock`` and
  ``/fmu/out/vehicle_odometry`` over at least 50 messages
* ``px4_offset_spread_s``, max gap minus that lowest gap
* ``px4_offset_samples``, how many gaps were collected
* ``px4_offset_reason``, why the numbers look the way they do
"""

from __future__ import annotations

import json
from pathlib import Path

from trajectory_eval.frames import quat_xyzw_to_wxyz, yaw_from_quat

SPAWN_FIELDS = (
    "spawn_xyz",
    "spawn_yaw",
    "px4_offset_s",
    "px4_offset_spread_s",
    "px4_offset_samples",
    "px4_offset_reason",
)
MIN_PX4_OFFSET_SAMPLES = 50


def yaw_from_xyzw(x: float, y: float, z: float, w: float) -> float:
    """ENU yaw (radians) of a ROS ``(x, y, z, w)`` quaternion."""
    return yaw_from_quat(quat_xyzw_to_wxyz((x, y, z, w)))


def build_spawn_document(
    translation: tuple[float, float, float] | None,
    rotation_xyzw: tuple[float, float, float, float] | None,
    gaps_s: list[float],
) -> dict:
    """The six agreed fields. Missing measurements stay null and say why."""
    reasons: list[str] = []
    if translation is None or rotation_xyzw is None:
        spawn_xyz = None
        spawn_yaw = None
        reasons.append("world -> spawn transform was not available")
    else:
        spawn_xyz = [float(translation[0]), float(translation[1]), float(translation[2])]
        spawn_yaw = yaw_from_xyzw(*rotation_xyzw)
        reasons.append("spawn_xyz and spawn_yaw are the live world -> spawn transform in ENU")

    samples = len(gaps_s)
    if samples < MIN_PX4_OFFSET_SAMPLES:
        offset_s = None
        spread_s = None
        reasons.append(
            "px4_offset_s needs at least "
            f"{MIN_PX4_OFFSET_SAMPLES} vehicle_odometry samples against /clock, got {samples}"
        )
    else:
        lowest = min(gaps_s)
        offset_s = float(lowest)
        spread_s = float(max(gaps_s) - lowest)
        reasons.append(
            "px4_offset_s is the lowest t_ros - t_px4 gap between /clock and "
            "/fmu/out/vehicle_odometry"
        )

    document = {
        "spawn_xyz": spawn_xyz,
        "spawn_yaw": spawn_yaw,
        "px4_offset_s": offset_s,
        "px4_offset_spread_s": spread_s,
        "px4_offset_samples": samples,
        "px4_offset_reason": "; ".join(reasons),
    }
    if tuple(document) != SPAWN_FIELDS:
        raise ValueError(f"spawn.json fields must be {SPAWN_FIELDS}")
    return document


def write_spawn_json(path: Path, document: dict) -> None:
    """Write ``spawn.json`` with the agreed fields only."""
    payload = {key: document[key] for key in SPAWN_FIELDS}
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
