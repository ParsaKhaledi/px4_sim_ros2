"""Wall distance, goal rejection, and braking-limited speed.

Walls are the JSON maps at ``includes/gz/walls/<world>.json``. The document
matches the writer on the simulation branch: ``frame`` is ``world_enu`` or
``local``, and each segment has ``start`` and ``end`` as ``[x, y]`` metres.
``world_enu`` is Gazebo world ENU. Those points stay in the file until the
vehicle is landed, EKF2 yaw is aligned, and the ``world`` -> ``spawn``
transform is known. The world-to-local rotation is then the local heading
minus that spawn heading, converted in ``frames.py``. ``local`` is already
the vehicle frame and is not moved.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from px4_control.frames import world_enu_point_to_local, world_local_rotation

WORLD_FRAME = 'world_enu'
LOCAL_FRAME = 'local'
SEGMENT_KINDS = frozenset({'box', 'cylinder', 'mesh'})
_TF_MISSING = (
    'geofence off: static TF world -> spawn is not available; '
    'world_enu walls are not installed'
)
_ARM_BEFORE_ALIGN = (
    'geofence off: armed before the wall frame was aligned to EKF2; '
    'world_enu walls stay off'
)
_HEADING_RESET_LANDED = 'EKF2 heading reset while landed; re-deriving the wall frame'
_HEADING_RESET_FLIGHT = (
    'EKF2 heading reset in flight; braking to hold and turning the geofence off'
)


@dataclass(frozen=True)
class WallSegment:
    """A 2D line segment in local ENU metres (x east, y north)."""

    x1: float
    y1: float
    x2: float
    y2: float


@dataclass(frozen=True)
class SpawnFrame:
    """Static TF ``world`` -> ``spawn`` on the ground plane. Yaw is ENU."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class LocalFix:
    """Planar sample from ``vehicle_local_position``. x is north, y is east."""

    x: float
    y: float
    heading: float
    xy_valid: bool
    heading_reset_counter: int


@dataclass(frozen=True)
class WorldLocalFrame:
    """Frozen match between the spawn TF and the local position."""

    rotation: float
    spawn_x: float
    spawn_y: float
    spawn_yaw: float
    local_north: float
    local_east: float
    local_heading: float
    heading_reset_counter: int

    def to_local(self, x: float, y: float) -> tuple[float, float]:
        return world_enu_point_to_local(
            x, y, self.spawn_x, self.spawn_y, self.rotation, self.local_east, self.local_north,
        )


@dataclass(frozen=True)
class WallLoad:
    """``walls`` are local ENU. ``raw`` keeps the file coordinates.

    ``needs_alignment`` is set for a ``world_enu`` file whose points are not
    in local ENU yet. ``walls`` is then empty so world coordinates are not
    used as local ones.
    """

    walls: tuple[WallSegment, ...]
    path: str
    missing: bool
    frame: str = ''
    raw: tuple[WallSegment, ...] = ()
    needs_alignment: bool = False


@dataclass(frozen=True)
class AlignmentStep:
    """One update of :class:`WallAlignment`. ``brake`` asks the caller to hold."""

    logs: tuple[str, ...] = ()
    brake: bool = False


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Yaw of a ``world`` -> ``spawn`` quaternion. xyzw order, ENU."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def derive_frame(spawn: SpawnFrame, local: LocalFix) -> WorldLocalFrame:
    """Match one landed local sample to the spawn TF."""
    return WorldLocalFrame(
        rotation=world_local_rotation(local.heading, spawn.yaw),
        spawn_x=spawn.x,
        spawn_y=spawn.y,
        spawn_yaw=spawn.yaw,
        local_north=local.x,
        local_east=local.y,
        local_heading=local.heading,
        heading_reset_counter=int(local.heading_reset_counter),
    )


def _point(value, label: str) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f'{label} must be [x, y], got {value!r}')
    try:
        return float(value[0]), float(value[1])
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{label} must be two numbers, got {value!r}') from exc


def _segments_from_document(document: dict, file_path: Path) -> tuple[str, tuple[WallSegment, ...]]:
    frame = document.get('frame')
    if frame not in (WORLD_FRAME, LOCAL_FRAME):
        raise ValueError(f'{file_path}: frame must be world_enu or local, got {frame!r}')
    units = document.get('units', 'm')
    if units != 'm':
        raise ValueError(f'{file_path}: units must be m, got {units!r}')
    segments = document.get('segments')
    if not isinstance(segments, list):
        raise ValueError(f'{file_path}: segments must be a list')
    count = document.get('segment_count')
    if count is not None and int(count) != len(segments):
        raise ValueError(
            f'{file_path}: segment_count {count} does not match {len(segments)} segments'
        )
    walls: list[WallSegment] = []
    for index, segment in enumerate(segments):
        if not isinstance(segment, dict):
            raise ValueError(f'{file_path}: segments[{index}] must be an object')
        kind = segment.get('kind')
        if kind is not None and kind not in SEGMENT_KINDS:
            raise ValueError(f'{file_path}: segments[{index}].kind must be box, cylinder, or mesh')
        source = segment.get('source')
        if source is not None and not isinstance(source, str):
            raise ValueError(f'{file_path}: segments[{index}].source must be a string')
        x1, y1 = _point(segment.get('start'), f'{file_path}: segments[{index}].start')
        x2, y2 = _point(segment.get('end'), f'{file_path}: segments[{index}].end')
        walls.append(WallSegment(x1, y1, x2, y2))
    return frame, tuple(walls)


def _finite_spawn(spawn: SpawnFrame | None) -> bool:
    return (
        spawn is not None
        and math.isfinite(spawn.x)
        and math.isfinite(spawn.y)
        and math.isfinite(spawn.yaw)
    )


def _usable_fix(local: LocalFix | None) -> bool:
    return (
        local is not None
        and local.xy_valid
        and math.isfinite(local.x)
        and math.isfinite(local.y)
        and math.isfinite(local.heading)
    )


def _format_frame(alignment: WorldLocalFrame) -> str:
    return (
        f'rotation={alignment.rotation:.4f} rad, '
        f'spawn world ENU ({alignment.spawn_x:.3f}, {alignment.spawn_y:.3f}) '
        f'yaw={alignment.spawn_yaw:.4f}, '
        f'local NED ({alignment.local_north:.3f}, {alignment.local_east:.3f}) '
        f'heading={alignment.local_heading:.4f}'
    )


def load_wall_file(path: str | None) -> WallLoad:
    """Load a wall JSON file. An empty path or a missing file is an empty set.

    A ``world_enu`` file is returned with ``needs_alignment`` set and no local
    walls. :class:`WallAlignment` installs them once the spawn TF and a landed
    EKF2 sample match.
    """
    if path is None or str(path).strip() == '':
        return WallLoad((), '', False)
    file_path = Path(path)
    if not file_path.is_file():
        return WallLoad((), str(file_path), True)
    if file_path.suffix.lower() == '.txt':
        raise ValueError(
            f'{file_path}: the text wall list is not a wall format; '
            'use includes/gz/walls/<world>.json'
        )
    try:
        document = json.loads(file_path.read_text(encoding='utf-8'))
    except json.JSONDecodeError as exc:
        raise ValueError(f'{file_path}: not a JSON wall map: {exc}') from exc
    if not isinstance(document, dict):
        raise ValueError(f'{file_path}: wall map must be a JSON object')
    frame, raw = _segments_from_document(document, file_path)
    if frame == WORLD_FRAME:
        return WallLoad((), str(file_path), False, frame, raw, True)
    return WallLoad(raw, str(file_path), False, frame, raw, False)


class WallAlignment:
    """Derive, freeze, and invalidate the world ENU to local ENU frame.

    While landed and before arming, the first sample with EKF2 yaw aligned
    and a valid local position is matched to the spawn TF. Arming freezes
    that match. A heading-reset counter change re-derives the match while
    landed, and asks for a brake-hold when the vehicle is in flight.
    """

    def __init__(self, loaded: WallLoad) -> None:
        self.loaded = loaded
        self.frame: WorldLocalFrame | None = None
        self.frozen = False
        self._counter: int | None = None
        self._tf_missing_logged = False
        self._unaligned_arm_logged = False

    @property
    def walls(self) -> tuple[WallSegment, ...]:
        return self.loaded.walls

    def update(
        self,
        *,
        spawn: SpawnFrame | None,
        local: LocalFix | None,
        yaw_aligned: bool,
        landed: bool,
        armed: bool,
    ) -> AlignmentStep:
        if self.loaded.frame != WORLD_FRAME:
            return AlignmentStep()
        logs: list[str] = []
        rederived = False
        if (
            local is not None
            and self._counter is not None
            and int(local.heading_reset_counter) != self._counter
        ):
            self._counter = int(local.heading_reset_counter)
            self._clear_installed()
            if not landed:
                return AlignmentStep(logs=(_HEADING_RESET_FLIGHT,), brake=True)
            logs.append(_HEADING_RESET_LANDED)
            rederived = True
        if self.frozen and self.frame is not None:
            return AlignmentStep(logs=tuple(logs))
        if not _finite_spawn(spawn):
            if self.frame is None and not self._tf_missing_logged:
                self._tf_missing_logged = True
                logs.append(_TF_MISSING)
            return AlignmentStep(logs=tuple(logs))
        if (
            landed
            and yaw_aligned
            and spawn is not None
            and local is not None
            and _usable_fix(local)
            and self.frame is None
        ):
            alignment = derive_frame(spawn, local)
            self._install(alignment)
            self._counter = int(local.heading_reset_counter)
            logs.append(f'wall frame: {_format_frame(alignment)}')
        if armed and self.frame is not None and not self.frozen:
            self.frozen = True
            if rederived:
                logs.append(
                    f'wall frame re-frozen after a landed heading reset: '
                    f'rotation={self.frame.rotation:.4f} rad'
                )
            else:
                logs.append(
                    f'wall frame frozen at arming: rotation={self.frame.rotation:.4f} rad'
                )
        if armed and self.frame is None and self._counter is None and not self._unaligned_arm_logged:
            self._unaligned_arm_logged = True
            logs.append(_ARM_BEFORE_ALIGN)
        return AlignmentStep(logs=tuple(logs))

    def _install(self, alignment: WorldLocalFrame) -> None:
        walls = tuple(
            WallSegment(*alignment.to_local(wall.x1, wall.y1), *alignment.to_local(wall.x2, wall.y2))
            for wall in self.loaded.raw
        )
        self.frame = alignment
        self.loaded = WallLoad(
            walls, self.loaded.path, False, self.loaded.frame, self.loaded.raw, False,
        )

    def _clear_installed(self) -> None:
        loaded = self.loaded
        self.frame = None
        self.frozen = False
        self.loaded = WallLoad((), loaded.path, loaded.missing, loaded.frame, loaded.raw, True)


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
        # Clearance along the motion, not the perpendicular gap divided into
        # the speed. On a diagonal those two are not the same.
        clearance = max(0.0, distance - radius - margin)
        along_clearance = clearance / closing
        cap = min(cap, math.sqrt(2.0 * a_brake * along_clearance))
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
