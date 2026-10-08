"""Place the drone spawn on a configured geographic coordinate.

``SIM_ORIGIN_LAT``, ``SIM_ORIGIN_LON`` and ``SIM_ORIGIN_ALT`` are the
latitude, longitude and AMSL elevation of the spawn pose
(``PX4_GZ_MODEL_POSE``), not of the Gazebo world origin. The world origin
is offset from that point by the spawn translation. Heading stays 0, so
the ENU world axes stay east, north and up.

PX4 v1.17 gz SITL reads GPS from the model's NavSat sensor, which uses the
world ``<spherical_coordinates>``. After the world is ready, ``px4-rc.gzsim``
also calls ``/world/<name>/set_spherical_coordinates`` when ``PX4_HOME_LAT``,
``PX4_HOME_LON`` and ``PX4_HOME_ALT`` are all set. In that version those
three variables are the world origin, not the vehicle home. This script
exports them as the computed world origin so the service and the SDF agree.
The vehicle home is the first GPS fix, which is the spawn.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
from pathlib import Path

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

DEFAULT_LAT = 35.7048378
DEFAULT_LON = 51.4095049
DEFAULT_ALT = 1205.0
DEFAULT_POSE = "-3,-1.6,0,0,0,3.14"

_BLOCK = re.compile(
    r"<spherical_coordinates\b[^>]*>.*?</spherical_coordinates>",
    re.DOTALL,
)
_WORLD = re.compile(r"(<world\b[^>]*>)")


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
    """WGS84 geodetic coordinates to ECEF metres."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    normal = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (normal + alt_m) * cos_lat * cos_lon
    y = (normal + alt_m) * cos_lat * sin_lon
    z = (normal * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def ecef_to_geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    """ECEF metres to WGS84 latitude, longitude and altitude."""
    lon = math.atan2(y, x)
    horizontal = math.hypot(x, y)
    lat = math.atan2(z, horizontal * (1.0 - WGS84_E2))
    alt = 0.0
    for _ in range(12):
        sin_lat = math.sin(lat)
        normal = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        alt = horizontal / math.cos(lat) - normal
        lat = math.atan2(z, horizontal * (1.0 - WGS84_E2 * normal / (normal + alt)))
    return math.degrees(lat), math.degrees(lon), alt


def enu_to_geodetic(east_m: float, north_m: float, up_m: float,
                    lat0_deg: float, lon0_deg: float, alt0_m: float) -> tuple[float, float, float]:
    """Geodetic point reached by an ENU offset from a reference."""
    lat = math.radians(lat0_deg)
    lon = math.radians(lon0_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    dx = -sin_lon * east_m - sin_lat * cos_lon * north_m + cos_lat * cos_lon * up_m
    dy = cos_lon * east_m - sin_lat * sin_lon * north_m + cos_lat * sin_lon * up_m
    dz = cos_lat * north_m + sin_lat * up_m
    ox, oy, oz = geodetic_to_ecef(lat0_deg, lon0_deg, alt0_m)
    return ecef_to_geodetic(ox + dx, oy + dy, oz + dz)


def parse_spawn_xyz(pose: str) -> tuple[float, float, float]:
    """Read x, y, z metres from ``PX4_GZ_MODEL_POSE`` (``x,y,z,roll,pitch,yaw``)."""
    parts = [part.strip() for part in (pose or "").split(",")]
    values = []
    for part in parts[:3]:
        try:
            values.append(float(part))
        except ValueError:
            values.append(0.0)
    while len(values) < 3:
        values.append(0.0)
    return values[0], values[1], values[2]


def world_origin_from_spawn(lat_deg: float, lon_deg: float, alt_m: float,
                            spawn_xyz: tuple[float, float, float],
                            heading_deg: float = 0.0) -> tuple[float, float, float]:
    """World-origin latitude, longitude and elevation for a spawn coordinate.

    With ``heading_deg`` 0 the Gazebo ENU axes are east, north and up, so the
    spawn translation is the ENU vector from the world origin to the drone.
    The world origin is the spawn coordinate plus the opposite of that vector.
    """
    if heading_deg != 0.0:
        raise ValueError("only heading_deg 0 is supported")
    east, north, up = spawn_xyz
    return enu_to_geodetic(-east, -north, -up, lat_deg, lon_deg, alt_m)


def spherical_block(lat_deg: float, lon_deg: float, alt_m: float, heading_deg: float = 0.0) -> str:
    """One world spherical-coordinates element, indented for an SDF world."""
    # The opening tag keeps the indent already on its line. Children and the
    # closing tag carry their own indent so a rewrite does not double-indent
    # only the first line.
    return (
        "<spherical_coordinates>\n"
        "      <surface_model>EARTH_WGS84</surface_model>\n"
        "      <world_frame_orientation>ENU</world_frame_orientation>\n"
        f"      <latitude_deg>{lat_deg:.10f}</latitude_deg>\n"
        f"      <longitude_deg>{lon_deg:.10f}</longitude_deg>\n"
        f"      <elevation>{alt_m:.4f}</elevation>\n"
        f"      <heading_deg>{heading_deg:.4f}</heading_deg>\n"
        "    </spherical_coordinates>"
    )


def rewrite_spherical_coordinates(text: str, lat_deg: float, lon_deg: float,
                                  alt_m: float, heading_deg: float = 0.0) -> tuple[str, int]:
    """Set every spherical-coordinates block. Insert one if the world has none."""
    block = spherical_block(lat_deg, lon_deg, alt_m, heading_deg)

    def replace(_match: re.Match[str]) -> str:
        return block

    updated, count = _BLOCK.subn(replace, text)
    if count == 0:
        updated, inserted = _WORLD.subn("\\1\n    " + block + "\n", text, count=1)
        count = inserted
    return updated, count


def origin_from_env(environ: dict[str, str] | None = None) -> dict[str, float]:
    """Spawn coordinate and the world origin computed from it."""
    env = os.environ if environ is None else environ
    lat = float(env.get("SIM_ORIGIN_LAT") or DEFAULT_LAT)
    lon = float(env.get("SIM_ORIGIN_LON") or DEFAULT_LON)
    alt = float(env.get("SIM_ORIGIN_ALT") or DEFAULT_ALT)
    pose = env.get("PX4_GZ_MODEL_POSE") or DEFAULT_POSE
    spawn = parse_spawn_xyz(pose)
    origin_lat, origin_lon, origin_alt = world_origin_from_spawn(lat, lon, alt, spawn)
    return {
        "spawn_lat": lat,
        "spawn_lon": lon,
        "spawn_alt": alt,
        "spawn_x": spawn[0],
        "spawn_y": spawn[1],
        "spawn_z": spawn[2],
        "origin_lat": origin_lat,
        "origin_lon": origin_lon,
        "origin_alt": origin_alt,
    }


def px4_world_dirs() -> list[Path]:
    """Directories PX4 actually loads worlds from, when they exist."""
    candidates = []
    if os.environ.get("PX4_GZ_WORLDS"):
        candidates.append(Path(os.environ["PX4_GZ_WORLDS"]))
    home = Path(os.environ.get("HOME", "/home/px4"))
    candidates.append(home / "PX4-Autopilot" / "Tools" / "simulation" / "gz" / "worlds")
    candidates.append(Path("/home/px4/PX4-Autopilot/Tools/simulation/gz/worlds"))
    found = []
    seen = set()
    for path in candidates:
        resolved = str(path)
        if resolved in seen or not path.is_dir():
            continue
        seen.add(resolved)
        found.append(path)
    return found


def repo_world_names(worlds_dir: Path | None = None) -> set[str]:
    """SDF file names shipped under ``includes/gz/worlds``."""
    directory = worlds_dir or Path(__file__).resolve().parents[1] / "worlds"
    if not directory.is_dir():
        return set()
    return {path.name for path in directory.glob("*.sdf")}


def apply_origin_to_worlds(dest: Path, lat_deg: float, lon_deg: float, alt_m: float,
                           names: set[str] | None = None) -> list[str]:
    """Rewrite spherical coordinates in ``dest``.

    ``names`` limits the edit to those files plus ``default.sdf`` (the world
    PX4 loads when ``World=default``). ``None`` updates every SDF in ``dest``.
    """
    if not dest.is_dir():
        return []
    written = []
    for path in sorted(dest.glob("*.sdf")):
        if names is not None and path.name not in names and path.name != "default.sdf":
            continue
        text = path.read_text(encoding="utf-8")
        updated, count = rewrite_spherical_coordinates(text, lat_deg, lon_deg, alt_m)
        if count == 0 or updated == text:
            continue
        path.write_text(updated, encoding="utf-8")
        written.append(f"{path.name}:{count}")
    return written


def shell_exports(origin: dict[str, float]) -> str:
    """Assignments for ``eval``. ``PX4_HOME_*`` is the world origin."""
    return "\n".join([
        f"export PX4_HOME_LAT={origin['origin_lat']:.10f}",
        f"export PX4_HOME_LON={origin['origin_lon']:.10f}",
        f"export PX4_HOME_ALT={origin['origin_alt']:.4f}",
        f"export SIM_ORIGIN_LAT={origin['spawn_lat']:.10f}",
        f"export SIM_ORIGIN_LON={origin['spawn_lon']:.10f}",
        f"export SIM_ORIGIN_ALT={origin['spawn_alt']:.4f}",
    ])


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Set the Gazebo world origin so the spawn pose is SIM_ORIGIN_*.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the world origin and PX4_HOME_* exports without writing worlds",
    )
    args = parser.parse_args(argv)
    origin = origin_from_env()
    names = repo_world_names()
    written: list[str] = []
    dirs = px4_world_dirs()
    if not args.dry_run:
        for directory in dirs:
            written.extend(apply_origin_to_worlds(
                directory, origin["origin_lat"], origin["origin_lon"], origin["origin_alt"], names,
            ))
    print(shell_exports(origin))
    if args.dry_run:
        print(
            "world origin: "
            f"{origin['origin_lat']:.10f} {origin['origin_lon']:.10f} {origin['origin_alt']:.4f}"
        )
    print(
        "sim origin: spawn "
        f"{origin['spawn_lat']:.8f}, {origin['spawn_lon']:.8f}, {origin['spawn_alt']:.3f} m "
        f"at xyz {origin['spawn_x']},{origin['spawn_y']},{origin['spawn_z']}",
        file=sys.stderr,
    )
    print(
        "sim origin: world "
        f"{origin['origin_lat']:.8f}, {origin['origin_lon']:.8f}, {origin['origin_alt']:.3f} m "
        "(ENU, heading 0). PX4_HOME_* is this world origin.",
        file=sys.stderr,
    )
    if written:
        print("sim origin: updated " + ", ".join(written), file=sys.stderr)
    elif not dirs:
        print(
            "sim origin: PX4 worlds directory not found; "
            "px4-rc.gzsim still applies PX4_HOME_* after the world starts",
            file=sys.stderr,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
