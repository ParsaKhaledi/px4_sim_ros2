"""IMU, air pressure, and NavSat belong on x500_base.

The magnetometer stays a world plugin. apt_world carries it so PX4 sees a
compass. default keeps the copy PX4 ships in server.config.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from patch_x500_sensor_systems import (
    SENSOR_SYSTEMS,
    ensure_model_systems,
    main,
    strip_sensor_systems,
)

WORLDS = Path(__file__).resolve().parents[1] / "worlds"
FORBIDDEN = tuple(name for _filename, name in SENSOR_SYSTEMS)

# Shape of a PX4 world that still carries the four systems next to the
# rendering Sensors plugin. PX4 v1.17 default.sdf itself has no plugins;
# server.config injects these, and older world copies inline them.
DEFAULT_SDF_SAMPLE = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-air-speed-system" name="gz::sim::systems::AirSpeed"/>
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <sensor name="imu_sensor" type="imu"/>
  </world>
</sdf>
"""

BASE_MODEL = """<sdf>
  <model name="x500_base">
    <link name="base_link">
      <sensor name="imu_sensor" type="imu">
        <always_on>1</always_on>
      </sensor>
      <sensor name="air_pressure_sensor" type="air_pressure"/>
      <sensor name="magnetometer_sensor" type="magnetometer"/>
      <sensor name="navsat_sensor" type="navsat"/>
    </link>
  </model>
</sdf>
"""


def test_missing_launched_world_is_an_error(monkeypatch, capsys):
    monkeypatch.setattr("patch_x500_sensor_systems.candidate_base_models", lambda: [])
    monkeypatch.setattr("patch_x500_sensor_systems.candidate_worlds", lambda _name: [])
    monkeypatch.setattr("patch_x500_sensor_systems.candidate_server_configs", lambda: [])
    assert main(["apt_world"]) == 1
    assert "apt_world.sdf not found" in capsys.readouterr().err


def test_repo_worlds_keep_rendering_and_only_apt_world_has_a_compass():
    worlds = sorted(WORLDS.glob("*.sdf"))
    assert [path.name for path in worlds] == ["apt_world.sdf"]
    text = worlds[0].read_text(encoding="utf-8")
    for name in FORBIDDEN:
        assert name not in text, name
    assert "gz::sim::systems::Magnetometer" in text
    assert 'filename="gz-sim-magnetometer-system"' in text
    assert "gz::sim::systems::Sensors" in text
    assert "ogre2" in text
    assert "gz::sim::systems::Physics" in text


def test_x500_base_patch_is_idempotent_and_keeps_sensors():
    once, added = ensure_model_systems(BASE_MODEL)
    assert [name for _filename, name in SENSOR_SYSTEMS] == added
    for _filename, name in SENSOR_SYSTEMS:
        assert once.count(name) == 1
    twice, added_again = ensure_model_systems(once)
    assert added_again == []
    assert twice == once
    assert '<sensor name="imu_sensor" type="imu">' in twice
    assert '<sensor name="air_pressure_sensor" type="air_pressure"/>' in twice
    assert '<sensor name="magnetometer_sensor" type="magnetometer"/>' in twice
    assert '<sensor name="navsat_sensor" type="navsat"/>' in twice
    assert twice.count("<sensor ") == BASE_MODEL.count("<sensor ")


def test_world_strip_leaves_the_magnetometer_world_plugin():
    stripped, removed = strip_sensor_systems(DEFAULT_SDF_SAMPLE)
    assert removed == [
        "gz::sim::systems::Imu",
        "gz::sim::systems::AirPressure",
        "gz::sim::systems::NavSat",
    ]
    for name in FORBIDDEN:
        assert name not in stripped
    assert "gz::sim::systems::Magnetometer" in stripped
    for kept in (
        "gz::sim::systems::Physics",
        "gz::sim::systems::UserCommands",
        "gz::sim::systems::SceneBroadcaster",
        "gz::sim::systems::Contact",
        "gz::sim::systems::AirSpeed",
        "gz::sim::systems::ApplyLinkWrench",
        "gz::sim::systems::Sensors",
        "ogre2",
        '<sensor name="imu_sensor" type="imu"/>',
    ):
        assert kept in stripped
    again, removed_again = strip_sensor_systems(stripped)
    assert removed_again == []
    assert again == stripped


def test_apt_world_keeps_its_compass_and_drops_the_server_copy(tmp_path, monkeypatch):
    world = tmp_path / "apt_world.sdf"
    world.write_text(
        "<sdf><world name='apt_world'>"
        '<plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>'
        '<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>'
        "</world></sdf>",
        encoding="utf-8",
    )
    server = tmp_path / "server.config"
    server.write_text(
        "<server_config>"
        '<plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>'
        '<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>'
        "</server_config>",
        encoding="utf-8",
    )
    monkeypatch.setattr("patch_x500_sensor_systems.candidate_base_models", lambda: [])
    monkeypatch.setattr("patch_x500_sensor_systems.candidate_worlds", lambda _name: [world])
    monkeypatch.setattr("patch_x500_sensor_systems.candidate_server_configs", lambda: [server])
    assert main(["apt_world"]) == 0
    kept = world.read_text(encoding="utf-8")
    assert "gz::sim::systems::Magnetometer" in kept
    assert "gz::sim::systems::Imu" not in kept
    dropped = server.read_text(encoding="utf-8")
    assert "gz::sim::systems::Magnetometer" not in dropped
    assert "gz::sim::systems::Imu" not in dropped


def test_default_keeps_the_server_config_compass(tmp_path, monkeypatch):
    world = tmp_path / "default.sdf"
    world.write_text(
        "<sdf><world name='default'>"
        '<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>'
        '<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>'
        "</world></sdf>",
        encoding="utf-8",
    )
    server = tmp_path / "server.config"
    server.write_text(
        "<server_config>"
        '<plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>'
        '<plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>'
        "</server_config>",
        encoding="utf-8",
    )
    monkeypatch.setattr("patch_x500_sensor_systems.candidate_base_models", lambda: [])
    monkeypatch.setattr("patch_x500_sensor_systems.candidate_worlds", lambda _name: [world])
    monkeypatch.setattr("patch_x500_sensor_systems.candidate_server_configs", lambda: [server])
    assert main(["default"]) == 0
    launched = world.read_text(encoding="utf-8")
    assert "gz::sim::systems::Physics" in launched
    assert "gz::sim::systems::Imu" not in launched
    assert "gz::sim::systems::Magnetometer" not in launched
    kept = server.read_text(encoding="utf-8")
    assert "gz::sim::systems::Magnetometer" in kept
    assert "gz::sim::systems::NavSat" not in kept
