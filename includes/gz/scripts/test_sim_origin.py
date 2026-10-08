"""World-origin offset tests. The spawn, not the world origin, is SIM_ORIGIN_*."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from sim_origin import (
    DEFAULT_ALT,
    DEFAULT_LAT,
    DEFAULT_LON,
    apply_origin_to_worlds,
    ecef_to_geodetic,
    enu_to_geodetic,
    geodetic_to_ecef,
    origin_from_env,
    parse_spawn_xyz,
    rewrite_spherical_coordinates,
    world_origin_from_spawn,
)


def test_geodetic_round_trip():
    lat, lon, alt = ecef_to_geodetic(*geodetic_to_ecef(DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT))
    assert abs(lat - DEFAULT_LAT) < 1e-10
    assert abs(lon - DEFAULT_LON) < 1e-10
    assert abs(alt - DEFAULT_ALT) < 1e-4


def test_zero_spawn_keeps_the_coordinate():
    lat, lon, alt = world_origin_from_spawn(DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT, (0.0, 0.0, 0.0))
    assert abs(lat - DEFAULT_LAT) < 1e-10
    assert abs(lon - DEFAULT_LON) < 1e-10
    assert abs(alt - DEFAULT_ALT) < 1e-4


def test_default_pose_puts_spawn_on_the_campus_point():
    """Default spawn is 3 m west and 1.6 m south of the world origin."""
    origin = origin_from_env({
        "SIM_ORIGIN_LAT": str(DEFAULT_LAT),
        "SIM_ORIGIN_LON": str(DEFAULT_LON),
        "SIM_ORIGIN_ALT": str(DEFAULT_ALT),
        "PX4_GZ_MODEL_POSE": "-3,-1.6,0,0,0,3.14",
    })
    back = enu_to_geodetic(
        origin["spawn_x"], origin["spawn_y"], origin["spawn_z"],
        origin["origin_lat"], origin["origin_lon"], origin["origin_alt"],
    )
    assert abs(back[0] - DEFAULT_LAT) < 1e-8
    assert abs(back[1] - DEFAULT_LON) < 1e-8
    assert abs(back[2] - DEFAULT_ALT) < 1e-3
    # World origin is east and north of a spawn in the negative quadrant.
    assert origin["origin_lat"] > DEFAULT_LAT
    assert origin["origin_lon"] > DEFAULT_LON
    assert abs(origin["origin_alt"] - DEFAULT_ALT) < 1e-3


def test_spawn_height_lowers_the_world_origin():
    lat, lon, alt = world_origin_from_spawn(DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT, (0.0, 0.0, 2.0))
    assert abs(lat - DEFAULT_LAT) < 1e-8
    assert abs(lon - DEFAULT_LON) < 1e-8
    assert abs(alt - (DEFAULT_ALT - 2.0)) < 1e-3


def test_parse_pose_blanks():
    assert parse_spawn_xyz("") == (0.0, 0.0, 0.0)
    assert parse_spawn_xyz("-3,-1.6,0,0,0,3.14") == (-3.0, -1.6, 0.0)


def test_rewrite_updates_every_block_and_inserts_when_missing(tmp_path):
    source = """<sdf><world name='husarion_office'>
    <spherical_coordinates>
      <latitude_deg>50.0</latitude_deg>
      <longitude_deg>19.0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>90</heading_deg>
    </spherical_coordinates>
    <state><spherical_coordinates>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates></state>
    </world></sdf>
    """
    updated, count = rewrite_spherical_coordinates(source, DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT)
    assert count == 2
    assert updated.count(f"{DEFAULT_LAT:.10f}") == 2
    assert updated.count("ENU") == 2
    assert "<heading_deg>0.0000</heading_deg>" in updated
    assert "50.0" not in updated

    bare = "<sdf><world name='empty'></world></sdf>"
    inserted, count = rewrite_spherical_coordinates(bare, 1.0, 2.0, 3.0)
    assert count == 1
    assert "<latitude_deg>1.0000000000</latitude_deg>" in inserted

    world = tmp_path / "default.sdf"
    world.write_text(source, encoding="utf-8")
    other = tmp_path / "skip_me.sdf"
    other.write_text(source, encoding="utf-8")
    written = apply_origin_to_worlds(tmp_path, DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT, {"husarion_office.sdf"})
    assert written == ["default.sdf:2"]
    assert "50.0" in other.read_text(encoding="utf-8")
