import json
import math

import pytest

from px4_control.frames import enu_yaw_to_ned
from px4_control.geofence import (
    LocalFix,
    SpawnFrame,
    WallAlignment,
    WallSegment,
    goal_rejection,
    limit_planar_velocity,
    load_wall_file,
    max_speed_along,
    max_speed_toward_wall,
    path_rejection,
    yaw_from_quaternion,
)


def test_speed_limit_formula():
    assert max_speed_toward_wall(3.0, 0.5, 0.5, 2.0) == pytest.approx(math.sqrt(8.0))
    assert max_speed_toward_wall(1.0, 0.5, 0.5, 2.0) == 0.0
    assert max_speed_toward_wall(0.2, 0.5, 0.5, 2.0) == 0.0


def test_empty_walls_do_not_limit_or_reject():
    assert limit_planar_velocity(1.0, -0.5, 0.0, 0.0, (), 0.3, 0.4, 1.0) == (1.0, -0.5)
    assert goal_rejection(0.0, 0.0, (), 0.3, 0.4) is None
    assert path_rejection(0.0, 0.0, 5.0, 5.0, (), 0.3, 0.4) is None
    assert max_speed_along(0.0, 0.0, 1.0, 0.0, (), 0.3, 0.4, 1.0) == math.inf


def test_velocity_into_a_wall_is_capped():
    wall = WallSegment(0.0, 2.0, 4.0, 2.0)
    # Vehicle at y=0, wall at y=2. Moving +y (north) toward the wall.
    # distance = 2, radius 0.3, margin 0.2, clearance = 1.5, a=1
    # v_max = sqrt(2 * 1 * 1.5) = sqrt(3)
    limited_x, limited_y = limit_planar_velocity(0.2, 5.0, 1.0, 0.0, (wall,), 0.3, 0.2, 1.0)
    assert limited_y == pytest.approx(math.sqrt(3.0))
    assert limited_x == pytest.approx(0.2)


def test_diagonal_speed_uses_clearance_along_the_motion():
    wall = WallSegment(0.0, 2.0, 4.0, 2.0)
    # Head-on: clearance 1.5 m, a=1, cap = sqrt(3).
    assert max_speed_along(1.0, 0.0, 0.0, 1.0, (wall,), 0.3, 0.2, 1.0) == pytest.approx(math.sqrt(3.0))
    # 45 degrees: along-track clearance is 1.5 / cos(45), not the head-on speed
    # divided by the closing component.
    closing = 1.0 / math.sqrt(2.0)
    expected = math.sqrt(2.0 * 1.5 / closing)
    diagonal = max_speed_along(1.0, 0.0, 1.0, 1.0, (wall,), 0.3, 0.2, 1.0)
    assert diagonal == pytest.approx(expected)
    assert diagonal < math.sqrt(3.0) / closing


def test_goal_and_path_rejection():
    wall = WallSegment(0.0, 0.0, 0.0, 5.0)
    assert goal_rejection(0.2, 1.0, (wall,), 0.3, 0.4) is not None
    assert goal_rejection(2.0, 1.0, (wall,), 0.3, 0.4) is None
    assert path_rejection(2.0, 1.0, 0.1, 1.0, (wall,), 0.3, 0.4) is not None


def _document(frame: str, segments: list[dict]) -> str:
    return json.dumps({
        'world': 'fixture',
        'frame': frame,
        'units': 'm',
        'slice_z_m': 0.5,
        'min_height_m': 0.25,
        'min_segment_m': 0.05,
        'description': 'test map',
        'segment_count': len(segments),
        'unresolved_uris': [],
        'segments': segments,
    })


def _north_wall(tmp_path, name: str):
    """Spawn (1, 2), wall 1 m north of it running 1 m east, in world ENU."""
    path = tmp_path / name
    path.write_text(
        _document('world_enu', [
            {'start': [1.0, 3.0], 'end': [2.0, 3.0], 'source': 'wall', 'kind': 'box'},
        ]),
        encoding='utf-8',
    )
    return load_wall_file(str(path))


def _fix(heading: float, counter: int = 0, north: float = 0.0, east: float = 0.0) -> LocalFix:
    return LocalFix(north, east, heading, True, counter)


def test_wall_json_local_passes_through(tmp_path):
    path = tmp_path / 'room.json'
    path.write_text(
        _document('local', [
            {'start': [0.0, 0.0], 'end': [1.0, 0.0], 'source': 'link/collision', 'kind': 'box'},
            {'start': [1.0, 0.0], 'end': [1.0, 2.0], 'source': 'link/collision', 'kind': 'box'},
        ]),
        encoding='utf-8',
    )
    loaded = load_wall_file(str(path))
    assert loaded.missing is False
    assert loaded.needs_alignment is False
    assert loaded.frame == 'local'
    assert loaded.walls == (
        WallSegment(0.0, 0.0, 1.0, 0.0),
        WallSegment(1.0, 0.0, 1.0, 2.0),
    )
    runtime = WallAlignment(loaded)
    step = runtime.update(
        spawn=SpawnFrame(1.0, 2.0, math.pi / 2.0),
        local=_fix(0.0),
        yaw_aligned=True,
        landed=True,
        armed=False,
    )
    assert step.logs == ()
    assert runtime.walls == loaded.walls
    missing = load_wall_file(str(tmp_path / 'absent.json'))
    assert missing.missing is True
    assert missing.walls == ()
    assert load_wall_file('').walls == ()


def test_spawn_yaw_90_deg_wall_stays_north_for_vision_and_gps(tmp_path):
    # ENU yaw +90 deg faces north, so the NED world heading is 0. A transform
    # that subtracts that ENU yaw would swing this north wall around to the east.
    loaded = _north_wall(tmp_path, 'office.json')
    assert loaded.needs_alignment is True
    assert loaded.walls == ()
    assert loaded.raw == (WallSegment(1.0, 3.0, 2.0, 3.0),)
    spawn = SpawnFrame(1.0, 2.0, math.pi / 2.0)
    world_heading = enu_yaw_to_ned(spawn.yaw)
    assert world_heading == pytest.approx(0.0)

    vision = WallAlignment(loaded)
    vision_step = vision.update(
        spawn=spawn, local=_fix(0.0), yaw_aligned=True, landed=True, armed=False,
    )
    assert vision.frame is not None
    assert vision.frame.rotation == pytest.approx(0.0)
    assert vision.frozen is False
    wall = vision.walls[0]
    assert (wall.x1, wall.y1, wall.x2, wall.y2) == pytest.approx((0.0, 1.0, 1.0, 1.0))
    assert any(line.startswith('wall frame:') for line in vision_step.logs)

    gps = WallAlignment(loaded)
    # Local north 0.4 m, east -0.2 m. The wall is still 1 m north of the vehicle.
    gps_step = gps.update(
        spawn=spawn,
        local=_fix(world_heading, north=0.4, east=-0.2),
        yaw_aligned=True,
        landed=True,
        armed=False,
    )
    assert gps.frame is not None
    assert gps.frame.rotation == pytest.approx(0.0)
    wall = gps.walls[0]
    assert (wall.x1, wall.y1, wall.x2, wall.y2) == pytest.approx((-0.2, 1.4, 0.8, 1.4))
    assert any(line.startswith('wall frame:') for line in gps_step.logs)

    held = gps.update(
        spawn=spawn,
        local=_fix(world_heading, north=0.4, east=-0.2),
        yaw_aligned=True,
        landed=True,
        armed=True,
    )
    assert gps.frozen is True
    assert any('frozen at arming' in line for line in held.logs)
    assert gps.walls[0] == wall
    again = gps.update(
        spawn=spawn,
        local=_fix(world_heading, north=0.4, east=-0.2),
        yaw_aligned=True,
        landed=True,
        armed=True,
    )
    assert again.logs == ()


def test_world_frame_alias_is_rejected_and_text_is_not_a_format(tmp_path):
    path = tmp_path / 'alias.json'
    path.write_text(
        _document('world', [{'start': [4.0, 6.0], 'end': [4.0, 8.0], 'kind': 'cylinder'}]),
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='world_enu or local'):
        load_wall_file(str(path))
    half = math.sqrt(0.5)
    assert yaw_from_quaternion(0.0, 0.0, half, half) == pytest.approx(math.pi / 2.0)
    text = path.with_suffix('.txt')
    text.write_text('0 0 1 0\n', encoding='utf-8')
    with pytest.raises(ValueError, match='not a wall format'):
        load_wall_file(str(text))


def test_missing_spawn_tf_leaves_the_geofence_off(tmp_path):
    loaded = _north_wall(tmp_path, 'office.json')
    runtime = WallAlignment(loaded)
    step = runtime.update(
        spawn=None, local=_fix(0.0), yaw_aligned=True, landed=True, armed=False,
    )
    assert runtime.walls == ()
    assert any(line.startswith('geofence off:') for line in step.logs)
    quiet = runtime.update(
        spawn=None, local=_fix(0.0), yaw_aligned=True, landed=True, armed=False,
    )
    assert quiet.logs == ()
    assert runtime.walls == ()


def test_heading_reset_rederives_while_landed_and_brakes_in_flight(tmp_path):
    # Spawn ENU yaw 0 faces east. World NED heading is +90 deg.
    # Vision heading 0 rotates a north wall to the west. GPS heading matches
    # the world heading and leaves the wall to the north.
    path = tmp_path / 'east.json'
    path.write_text(
        _document('world_enu', [
            {'start': [0.0, 5.0], 'end': [1.0, 5.0], 'source': 'wall', 'kind': 'box'},
        ]),
        encoding='utf-8',
    )
    loaded = load_wall_file(str(path))
    spawn = SpawnFrame(0.0, 0.0, 0.0)
    world_heading = enu_yaw_to_ned(0.0)
    assert world_heading == pytest.approx(math.pi / 2.0)
    runtime = WallAlignment(loaded)
    waiting = runtime.update(
        spawn=spawn, local=_fix(0.0), yaw_aligned=False, landed=True, armed=False,
    )
    assert waiting.logs == ()
    assert runtime.walls == ()

    vision = runtime.update(
        spawn=spawn, local=_fix(0.0), yaw_aligned=True, landed=True, armed=False,
    )
    assert runtime.frame is not None
    assert runtime.frame.rotation == pytest.approx(-math.pi / 2.0)
    wall = runtime.walls[0]
    assert (wall.x1, wall.y1, wall.x2, wall.y2) == pytest.approx((-5.0, 0.0, -5.0, 1.0))
    assert any(line.startswith('wall frame:') for line in vision.logs)

    gps = runtime.update(
        spawn=spawn,
        local=_fix(world_heading, counter=1),
        yaw_aligned=True,
        landed=True,
        armed=False,
    )
    assert runtime.frozen is False
    assert runtime.frame is not None
    assert runtime.frame.rotation == pytest.approx(0.0)
    wall = runtime.walls[0]
    assert (wall.x1, wall.y1, wall.x2, wall.y2) == pytest.approx((0.0, 5.0, 1.0, 5.0))
    assert any('re-deriving' in line for line in gps.logs)

    flight = runtime.update(
        spawn=spawn,
        local=_fix(world_heading, counter=2),
        yaw_aligned=True,
        landed=False,
        armed=True,
    )
    assert flight.brake is True
    assert any('in flight' in line for line in flight.logs)
    assert runtime.walls == ()
    assert runtime.frozen is False
