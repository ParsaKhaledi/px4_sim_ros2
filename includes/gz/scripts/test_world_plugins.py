"""Sensor systems belong on x500_base, not in world files."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from patch_x500_sensor_systems import (
    SENSOR_SYSTEMS,
    ensure_model_systems,
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


def test_repo_worlds_have_no_sensor_systems():
    worlds = sorted(WORLDS.glob("*.sdf"))
    assert worlds, f"no worlds in {WORLDS}"
    for path in worlds:
        text = path.read_text(encoding="utf-8")
        for name in FORBIDDEN:
            assert name not in text, path.name
        assert "gz::sim::systems::Sensors" in text, path.name
        assert "ogre2" in text, path.name
        assert "gz::sim::systems::Physics" in text, path.name


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


def test_world_strip_removes_only_the_four_sensor_systems():
    stripped, removed = strip_sensor_systems(DEFAULT_SDF_SAMPLE)
    assert removed == [
        "gz::sim::systems::Imu",
        "gz::sim::systems::AirPressure",
        "gz::sim::systems::NavSat",
        "gz::sim::systems::Magnetometer",
    ]
    for name in FORBIDDEN:
        assert name not in stripped
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
