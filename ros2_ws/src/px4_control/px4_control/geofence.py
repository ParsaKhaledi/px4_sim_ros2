"""Wall distance, goal rejection, and braking-limited speed.

Simulation may drop one text file per world at
``includes/gz/worlds/walls/<World>.txt``. Until those files exist the loader
returns an empty wall set, which leaves speed unlimited and accepts every goal.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class WallSegment:
    """A 2D line segment in local ENU metres (x east, y north)."""

    x1: float
    y1: float
    x2: float
    y2: float


@dataclass(frozen=True)
class WallLoad:
    walls: tuple[WallSegment, ...]
    path: str
    missing: bool


def load_wall_file(path: str | None) -> WallLoad:
    """Load wall segments. An empty path or a missing file is an empty set."""
    if path is None or str(path).strip() == '':
        return WallLoad((), '', False)
    file_path = Path(path)
    if not file_path.is_file():
        return WallLoad((), str(file_path), True)
    walls: list[WallSegment] = []
    for line_number, raw in enumerate(file_path.read_text(encoding='utf-8').splitlines(), start=1):
        text = raw.split('#', 1)[0].strip()
        if not text:
            continue
        parts = text.split()
        if len(parts) != 4:
            raise ValueError(f'{file_path}:{line_number}: expected "x1 y1 x2 y2", got {raw!r}')
        try:
            x1, y1, x2, y2 = (float(part) for part in parts)
        except ValueError as exc:
            raise ValueError(f'{file_path}:{line_number}: {raw!r} is not four numbers') from exc
        walls.append(WallSegment(x1, y1, x2, y2))
    return WallLoad(tuple(walls), str(file_path), False)


def closest_point(x: float, y: float, wall: WallSegment) -> tuple[float, float]:
    """Closest point on a segment to ``(x, y)``."""
    abx = wall.x2 - wall.x1
    aby = wall.y2 - wall.y1
    length_sq = abx * abx + aby * aby
    if length_sq < 1e-12:
        return wall.x1, wall.y1
    scale = ((x - wall.x1) * abx + (y - wall.y1) * aby) / length_sq
    scale = max(0.0, min(1.0, scale))
    return wall.x1 + scale * abx, wall.y1 + scale * aby


def distance_to_wall(x: float, y: float, wall: WallSegment) -> float:
    cx, cy = closest_point(x, y, wall)
    return math.hypot(x - cx, y - cy)


def max_speed_toward_wall(distance: float, radius: float, margin: float, a_brake: float) -> float:
    """``sqrt(2 * a_brake * max(0, d - r - margin))``."""
    if a_brake < 0.0:
        raise ValueError('a_brake must be non-negative')
    clearance = max(0.0, distance - radius - margin)
    return math.sqrt(2.0 * a_brake * clearance)


def _into_wall_direction(x: float, y: float, wall: WallSegment) -> tuple[float, float, float] | None:
    """Unit direction from the vehicle into the wall, plus the distance.

    Returns ``None`` when the vehicle sits on the segment.
    """
    cx, cy = closest_point(x, y, wall)
    away_x = x - cx
    away_y = y - cy
    distance = math.hypot(away_x, away_y)
    if distance < 1e-6:
        return None
    return -away_x / distance, -away_y / distance, distance


def limit_planar_velocity(
    vx: float,
    vy: float,
    x: float,
    y: float,
    walls: tuple[WallSegment, ...] | list[WallSegment],
    radius: float,
    margin: float,
    a_brake: float,
) -> tuple[float, float]:
    """Reduce the component of velocity aimed at each wall.

    An empty wall set returns the command unchanged. Speed parallel to a wall
    is left alone.
    """
    if not walls:
        return vx, vy
    limited_x, limited_y = vx, vy
    for wall in walls:
        approach = _into_wall_direction(x, y, wall)
        if approach is None:
            return 0.0, 0.0
        into_x, into_y, distance = approach
        speed_into = limited_x * into_x + limited_y * into_y
        if speed_into <= 0.0:
            continue
        allowed = max_speed_toward_wall(distance, radius, margin, a_brake)
        if speed_into > allowed:
            excess = speed_into - allowed
            limited_x -= into_x * excess
            limited_y -= into_y * excess
    return limited_x, limited_y


def max_speed_along(
    x: float,
    y: float,
    dir_x: float,
    dir_y: float,
    walls: tuple[WallSegment, ...] | list[WallSegment],
    radius: float,
    margin: float,
    a_brake: float,
) -> float:
    """Largest speed along a unit direction that still respects every wall."""
    norm = math.hypot(dir_x, dir_y)
    if norm < 1e-9 or not walls:
        return math.inf
    dir_x /= norm
    dir_y /= norm
    cap = math.inf
    for wall in walls:
        approach = _into_wall_direction(x, y, wall)
        if approach is None:
            return 0.0
        into_x, into_y, distance = approach
        closing = dir_x * into_x + dir_y * into_y
        if closing <= 1e-6:
            continue
        allowed = max_speed_toward_wall(distance, radius, margin, a_brake)
        cap = min(cap, allowed / closing)
    return cap


def goal_rejection(
    x: float,
    y: float,
    walls: tuple[WallSegment, ...] | list[WallSegment],
    radius: float,
    margin: float,
) -> str | None:
    """Return a reason when a goal point is inside the wall margin."""
    if not walls:
        return None
    limit = radius + margin
    closest = min(distance_to_wall(x, y, wall) for wall in walls)
    if closest < limit:
        return (
            f'goal ({x:.2f}, {y:.2f}) is {closest:.2f} m from a wall; '
            f'need at least {limit:.2f} m (radius {radius:.2f} + margin {margin:.2f})'
        )
    return None


def path_rejection(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    walls: tuple[WallSegment, ...] | list[WallSegment],
    radius: float,
    margin: float,
    samples: int = 21,
) -> str | None:
    """Reject a straight path that enters the wall margin."""
    if not walls:
        return None
    steps = max(2, samples)
    for index in range(steps):
        scale = index / (steps - 1)
        reason = goal_rejection(
            x0 + (x1 - x0) * scale,
            y0 + (y1 - y0) * scale,
            walls,
            radius,
            margin,
        )
        if reason is not None:
            return f'path enters a wall margin: {reason}'
    return None
