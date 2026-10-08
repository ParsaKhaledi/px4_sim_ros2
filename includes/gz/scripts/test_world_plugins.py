"""Every repo world must load the sensors PX4's compass and GPS need."""

from pathlib import Path

WORLDS = Path(__file__).resolve().parents[1] / "worlds"
REQUIRED = ("magnetometer", "navsat", "imu", "air-pressure", "air_pressure", "sensors")


def test_worlds_declare_px4_sensor_systems():
    worlds = sorted(WORLDS.glob("*.sdf"))
    assert worlds, f"no worlds in {WORLDS}"
    for path in worlds:
        text = path.read_text(encoding="utf-8").lower()
        assert "magnetometer" in text, path.name
        assert "navsat" in text, path.name
        assert "systems::imu" in text or "imu-system" in text, path.name
        assert "air-pressure" in text or "air_pressure" in text or "airpressure" in text, path.name
        assert "sensors" in text, path.name
        assert "ogre2" in text, path.name
