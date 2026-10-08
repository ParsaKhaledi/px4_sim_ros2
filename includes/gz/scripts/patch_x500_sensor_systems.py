#!/usr/bin/env python3
"""Put IMU, air-pressure, magnetometer, and NavSat systems on x500_base.

Those four systems publish every matching sensor in the simulation. Loading
one twice gives every sensor two publishers and twice the rate, so each must
appear once. This script adds them to PX4's ``x500_base`` model and removes
them from the world file Gazebo is about to load, and from PX4's
``server.config``, which otherwise injects the same four for every world.

Sensor elements on the model are not added or removed. A second x500 in the
same world would load the systems again, once per model. Multi-vehicle would
need the systems back in one shared place. That is out of scope.
"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

# filename, system name. Order is the order added to a model that has none.
SENSOR_SYSTEMS = (
    ("gz-sim-imu-system", "gz::sim::systems::Imu"),
    ("gz-sim-air-pressure-system", "gz::sim::systems::AirPressure"),
    ("gz-sim-magnetometer-system", "gz::sim::systems::Magnetometer"),
    ("gz-sim-navsat-system", "gz::sim::systems::NavSat"),
)
SENSOR_SYSTEM_NAMES = {name for _filename, name in SENSOR_SYSTEMS}

_PLUGIN = re.compile(
    r"[ \t]*<plugin\b(?:[^>]*/>|[^>]*>.*?</plugin>)[ \t]*\n?",
    re.DOTALL,
)
_NAME = re.compile(r'\bname="([^"]+)"')


def plugin_block(filename: str, system_name: str) -> str:
    return f'    <plugin filename="{filename}" name="{system_name}" />\n'


def ensure_model_systems(model_xml: str) -> tuple[str, list[str]]:
    """Insert any missing sensor systems before ``</model>``. Sensors stay put."""
    missing = [(filename, name) for filename, name in SENSOR_SYSTEMS if name not in model_xml]
    if not missing:
        return model_xml, []
    close = model_xml.rfind("</model>")
    if close < 0:
        raise ValueError("model.sdf has no </model> element")
    block = "".join(plugin_block(filename, name) for filename, name in missing)
    updated = model_xml[:close] + block + model_xml[close:]
    return updated, [name for _filename, name in missing]


def strip_sensor_systems(text: str) -> tuple[str, list[str]]:
    """Remove the four sensor-system plugins. Leave every other plugin."""
    removed: list[str] = []

    def replace(match: re.Match[str]) -> str:
        tag = match.group(0)
        name_match = _NAME.search(tag)
        name = name_match.group(1) if name_match else ""
        if name not in SENSOR_SYSTEM_NAMES:
            return tag
        removed.append(name)
        return ""

    return _PLUGIN.sub(replace, text), removed


def short_name(system_name: str) -> str:
    return system_name.rsplit("::", 1)[-1]


def removal_line(path: Path, removed: list[str]) -> str:
    listed = ", ".join(short_name(name) for name in removed) if removed else "none"
    return f"removed sensor systems from {path.name}: {listed}"


def unique_paths(paths: list[Path]) -> list[Path]:
    found: list[Path] = []
    seen: set[Path] = set()
    for path in paths:
        try:
            key = path.resolve()
        except OSError:
            key = path
        if key in seen:
            continue
        seen.add(key)
        found.append(path)
    return found


def candidate_base_models() -> list[Path]:
    """x500_base model.sdf paths, same lookup as the ground-truth patch."""
    raw: list[Path] = []
    env_models = os.environ.get("PX4_GZ_MODELS", "")
    if env_models:
        raw.append(Path(env_models) / "x500_base" / "model.sdf")
    home = Path.home()
    raw.append(home / "PX4-Autopilot" / "Tools" / "simulation" / "gz" / "models" / "x500_base" / "model.sdf")
    raw.append(Path("/home/px4/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf"))
    return unique_paths(raw)


def candidate_worlds(world_name: str) -> list[Path]:
    """The world file Gazebo loads for this run."""
    name = world_name[:-4] if world_name.endswith(".sdf") else world_name
    filename = f"{name}.sdf"
    directories: list[Path] = []
    if os.environ.get("PX4_GZ_WORLDS"):
        directories.append(Path(os.environ["PX4_GZ_WORLDS"]))
    home = Path.home()
    directories.append(home / "PX4-Autopilot" / "Tools" / "simulation" / "gz" / "worlds")
    directories.append(Path("/home/px4/PX4-Autopilot/Tools/simulation/gz/worlds"))
    return unique_paths([directory / filename for directory in directories])


def candidate_server_configs() -> list[Path]:
    """PX4 server.config, which loads the four systems for every world."""
    raw: list[Path] = []
    for key in ("GZ_SIM_SERVER_CONFIG_PATH", "PX4_GZ_SERVER_CONFIG"):
        value = os.environ.get(key, "")
        if value:
            raw.append(Path(value))
    home = Path.home()
    raw.append(home / "PX4-Autopilot" / "src" / "modules" / "simulation" / "gz_bridge" / "server.config")
    raw.append(Path("/home/px4/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config"))
    return unique_paths(raw)


def patch_model(path: Path) -> None:
    original = path.read_text(encoding="utf-8")
    updated, added = ensure_model_systems(original)
    if updated == original:
        print(f"sensor systems already present on {path}")
        return
    path.write_text(updated, encoding="utf-8")
    print(f"added sensor systems to {path.name}: {', '.join(short_name(name) for name in added)}")


def strip_file(path: Path) -> list[str]:
    original = path.read_text(encoding="utf-8")
    updated, removed = strip_sensor_systems(original)
    if updated != original:
        path.write_text(updated, encoding="utf-8")
    print(removal_line(path, removed))
    return removed


def main(argv: list[str] | None = None) -> int:
    world_name = "default"
    if argv is None:
        argv = sys.argv[1:]
    if argv:
        world_name = argv[0]
    patched_model = False
    for path in candidate_base_models():
        if path.is_file():
            patch_model(path)
            patched_model = True
    if not patched_model:
        print(
            "x500_base model.sdf not found; sensor systems were not added. "
            "This is expected outside the PX4 SITL container.",
            file=sys.stderr,
        )
    stripped_world = False
    for path in candidate_worlds(world_name):
        if path.is_file():
            strip_file(path)
            stripped_world = True
    if not stripped_world:
        print(
            f"world {world_name}.sdf not found; sensor systems were not stripped from a world. "
            "This is expected outside the PX4 SITL container.",
            file=sys.stderr,
        )
    for path in candidate_server_configs():
        if path.is_file():
            strip_file(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
