import math

import pytest

from px4_control.geofence import (
    WallSegment,
    goal_rejection,
    limit_planar_velocity,
    load_wall_file,
    max_speed_along,
    max_speed_toward_wall,
    path_rejection,
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


def test_goal_and_path_rejection():
    wall = WallSegment(0.0, 0.0, 0.0, 5.0)
    assert goal_rejection(0.2, 1.0, (wall,), 0.3, 0.4) is not None
    assert goal_rejection(2.0, 1.0, (wall,), 0.3, 0.4) is None
    assert path_rejection(2.0, 1.0, 0.1, 1.0, (wall,), 0.3, 0.4) is not None


def test_wall_file_format(tmp_path):
    path = tmp_path / 'room.txt'
    path.write_text('# comment\n\n0 0 1 0\n1 0 1 2\n', encoding='utf-8')
    loaded = load_wall_file(str(path))
    assert loaded.missing is False
    assert len(loaded.walls) == 2
    assert loaded.walls[0] == WallSegment(0.0, 0.0, 1.0, 0.0)
    missing = load_wall_file(str(tmp_path / 'absent.txt'))
    assert missing.missing is True
    assert missing.walls == ()
    assert load_wall_file('') .walls == ()
    path.write_text('0 0 1\n', encoding='utf-8')
    with pytest.raises(ValueError):
        load_wall_file(str(path))
