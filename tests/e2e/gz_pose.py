"""Gazebo pose text, and the ENU to PX4 NED comparison.

Gazebo Harmonic worlds used by PX4 are East-North-Up. PX4 local position
is North-East-Down. Grading stays in the Gazebo world. The error log is
the distance between that pose and vehicle_local_position.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

from grading import yaw_from_quat


def gazebo_env() -> dict:
    """Environment for `gz topic`, including the sim process partition.

    `docker exec` does not inherit variables the PX4 startup script
    exported only for its own shell. Copy GZ_* from the running `gz sim`
    so the pose subscription joins the same Gazebo partition.
    """

    env = os.environ.copy()
    proc_root = Path("/proc")
    if not proc_root.is_dir():
        return env
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\x00", b" ")
        except OSError:
            continue
        if b"gz sim" not in command:
            continue
        try:
            raw = (entry / "environ").read_bytes().split(b"\x00")
        except OSError:
            continue
        for item in raw:
            if not item or b"=" not in item:
                continue
            key, value = item.split(b"=", 1)
            name = key.decode("utf-8", errors="ignore")
            if name.startswith("GZ_"):
                env[name] = value.decode("utf-8", errors="ignore")
        break
    return env


def tilt_deg(qx: float, qy: float, qz: float, qw: float) -> float:
    """Angle between the body z axis and world up, in degrees."""

    z_up = 1.0 - 2.0 * (qx * qx + qy * qy)
    z_up = max(-1.0, min(1.0, z_up))
    return math.degrees(math.acos(z_up))


def ned_from_enu(x: float, y: float, z: float, origin: tuple[float, float, float]):
    """World ENU position to a NED offset from `origin` (also world ENU)."""

    ox, oy, oz = origin
    north = y - oy
    east = x - ox
    down = -(z - oz)
    return north, east, down


def setpoint_world(north: float, east: float, down: float, origin: tuple[float, float, float]):
    """NED setpoint, relative to origin, back into world ENU."""

    ox, oy, oz = origin
    return ox + east, oy + north, oz - down


def px4_gt_error_m(px4_ned, gz_enu, origin) -> float:
    north, east, down = ned_from_enu(gz_enu[0], gz_enu[1], gz_enu[2], origin)
    return math.sqrt(
        (px4_ned[0] - north) ** 2
        + (px4_ned[1] - east) ** 2
        + (px4_ned[2] - down) ** 2
    )


def error_stats(samples: list[dict]) -> dict | None:
    values = [
        float(row["px4_gt_error_m"])
        for row in samples
        if row.get("px4_gt_error_m") is not None
    ]
    if not values:
        return None
    return {
        "mean": round(sum(values) / len(values), 4),
        "max": round(max(values), 4),
        "count": len(values),
    }


def _block_value(block: str, field: str) -> float | None:
    needle = field + ":"
    for line in block.splitlines():
        stripped = line.strip()
        if stripped.startswith(needle):
            raw = stripped.split(":", 1)[1].strip()
            try:
                return float(raw)
            except ValueError:
                return None
    return None


def _pose_blocks(text: str) -> list[str]:
    blocks = []
    current = []
    depth = 0
    for line in text.splitlines():
        if line.strip() == "pose {":
            current = [line]
            depth = 1
            continue
        if depth == 0:
            continue
        current.append(line)
        depth += line.count("{")
        depth -= line.count("}")
        if depth <= 0:
            blocks.append("\n".join(current))
            current = []
            depth = 0
    return blocks


def _one_pose(block: str) -> dict | None:
    name = None
    for line in block.splitlines():
        stripped = line.strip()
        if stripped.startswith("name:"):
            name = stripped.split(":", 1)[1].strip().strip('"')
            break
    if not name:
        return None
    position = block.split("position {", 1)
    orientation = block.split("orientation {", 1)
    if len(position) < 2 or len(orientation) < 2:
        return None
    pos = position[1].split("}", 1)[0]
    quat = orientation[1].split("}", 1)[0]
    values = {
        "x": _block_value(pos, "x"),
        "y": _block_value(pos, "y"),
        "z": _block_value(pos, "z"),
        "qx": _block_value(quat, "x"),
        "qy": _block_value(quat, "y"),
        "qz": _block_value(quat, "z"),
        "qw": _block_value(quat, "w"),
    }
    if any(value is None for value in values.values()):
        return None
    yaw = yaw_from_quat(values["qx"], values["qy"], values["qz"], values["qw"])
    return {
        "name": name,
        "x": values["x"],
        "y": values["y"],
        "z": values["z"],
        "qx": values["qx"],
        "qy": values["qy"],
        "qz": values["qz"],
        "qw": values["qw"],
        "yaw": yaw,
        "tilt_deg": tilt_deg(values["qx"], values["qy"], values["qz"], values["qw"]),
    }


def match_score(name: str, wanted: str) -> int:
    """Prefer the exact model, then the PX4 suffix `_0`."""

    if name == wanted:
        return 2
    if name == f"{wanted}_0":
        return 1
    return 0


def parse_rtf(text: str) -> float | None:
    """Pull real_time_factor out of `gz topic` text for /world/<name>/stats."""

    for line in text.splitlines():
        if "real_time_factor" not in line or ":" not in line:
            continue
        raw = line.split(":", 1)[1].strip()
        try:
            return float(raw)
        except ValueError:
            continue
    return None


def parse_pose_v(text: str, model_name: str) -> dict | None:
    """Pick the vehicle pose out of a `gz.msgs.Pose_V` text dump."""

    best = None
    best_score = 0
    for block in _pose_blocks(text):
        pose = _one_pose(block)
        if pose is None:
            continue
        score = match_score(pose["name"], model_name)
        if score > best_score:
            best = pose
            best_score = score
    return best
