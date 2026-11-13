"""World-origin offset tests. The spawn, not the world origin, is SIM_ORIGIN_*."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from sim_origin import (
    DEFAULT_ALT,
    DEFAULT_LAT,
    DEFAULT_LON,
    DEFAULT_POSE,
    apply_origin_to_worlds,
    ecef_to_geodetic,
    enu_to_geodetic,
    geodetic_to_ecef,
    main,
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
        "PX4_GZ_MODEL_POSE": DEFAULT_POSE,
    })
    assert origin["spawn_z"] == 0.15
    back = enu_to_geodetic(
        origin["spawn_x"], origin["spawn_y"], 0.0,
        origin["origin_lat"], origin["origin_lon"], origin["origin_alt"],
    )
    assert abs(back[0] - DEFAULT_LAT) < 1e-8
    assert abs(back[1] - DEFAULT_LON) < 1e-8
    assert abs(back[2] - DEFAULT_ALT) < 1e-3
    # World origin is east and north of a spawn in the negative quadrant.
    assert origin["origin_lat"] > DEFAULT_LAT
    assert origin["origin_lon"] > DEFAULT_LON
    assert abs(origin["origin_alt"] - DEFAULT_ALT) < 1e-3


def test_pose_z_does_not_change_origin_altitude():
    flat = world_origin_from_spawn(DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT, (-3.0, -1.6, 0.0))
    raised = world_origin_from_spawn(DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT, (-3.0, -1.6, 0.15))
    assert abs(flat[2] - DEFAULT_ALT) < 1e-6
    assert abs(raised[2] - DEFAULT_ALT) < 1e-6
    assert abs(flat[0] - raised[0]) < 1e-12
    assert abs(flat[1] - raised[1]) < 1e-12


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
    assert "\n      <latitude_deg>" in updated
    assert "<heading_deg>0.0000</heading_deg>" in updated
    indented = (
        "    <spherical_coordinates>\n"
        "      <latitude_deg>1</latitude_deg>\n"
        "    </spherical_coordinates>\n"
    )
    rewritten, _count = rewrite_spherical_coordinates(indented, DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT)
    assert rewritten.startswith("    <spherical_coordinates>\n")
    assert "\n        <spherical_coordinates>" not in rewritten
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


def test_dry_run_prints_exports_without_writing(tmp_path, monkeypatch, capsys):
    world = tmp_path / "default.sdf"
    original = (
        "<sdf><world name='w'>\n"
        "    <spherical_coordinates><latitude_deg>1</latitude_deg></spherical_coordinates>\n"
        "</world></sdf>\n"
    )
    world.write_text(original, encoding="utf-8")
    monkeypatch.setenv("PX4_GZ_WORLDS", str(tmp_path))
    monkeypatch.setenv("HOME", str(tmp_path / "empty-home"))
    monkeypatch.setenv("SIM_ORIGIN_LAT", str(DEFAULT_LAT))
    monkeypatch.setenv("SIM_ORIGIN_LON", str(DEFAULT_LON))
    monkeypatch.setenv("SIM_ORIGIN_ALT", str(DEFAULT_ALT))
    monkeypatch.setenv("PX4_GZ_MODEL_POSE", DEFAULT_POSE)
    assert main(["--dry-run"]) == 0
    assert world.read_text(encoding="utf-8") == original
    dry = capsys.readouterr().out
    assert "export PX4_HOME_LAT=35.7048522177" in dry
    assert "world origin: 35.7048522177 51.4095380435 1205.0000" in dry
    assert main([]) == 0
    assert "35.7048522177" in world.read_text(encoding="utf-8")
    live = capsys.readouterr().out
    assert live.splitlines()[0].startswith("export PX4_HOME_LAT=")
