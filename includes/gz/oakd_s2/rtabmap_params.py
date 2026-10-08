#!/usr/bin/env python3
"""Pick the RTAB-Map ini for VISION_PROFILE and the camera-mode arguments.

The launch file loads the ini through ``cfg:=`` (``config_path``). The SLAM
node drops keys that start with ``Odom``. The odometry node keeps the keys
that belong to odometry. ``-d`` and the stereo-versus-RGB-D keys stay on
``args`` / ``odom_args``, because one ini cannot hold both camera modes.

Startup writes the effective profile, size, rate, and every parameter to
stderr. Lines that begin with ``rtabmap_param source=ini`` are the file.
Lines with ``source=camera`` are the mode overrides. A health check can
compare the ini lines to the file.
"""

from __future__ import annotations

import argparse
import shlex
import sys
from pathlib import Path

import geometry as geo


INI_DIR = Path(__file__).resolve().parents[1] / "startFiles" / "rtabmap_profiles"
CAMERAS = ("stereo", "rgbd", "rgbd-wrapper")


def ini_path(profile_name: str) -> Path:
    name = str(profile_name).strip().lower()
    if name not in geo.PROFILES:
        raise ValueError(f"VISION_PROFILE must be cpu, full, or hw, got {profile_name}")
    path = INI_DIR / f"{name}.ini"
    if not path.is_file():
        raise FileNotFoundError(f"missing RTAB-Map profile file {path}")
    return path


def parse_rtabmap_ini(text: str) -> dict[str, str]:
    """Read a ``[Core]`` ini. Backslashes in keys become slashes, as readINI does."""
    section = ""
    params: dict[str, str] = {}
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith(";") or line.startswith("#"):
            continue
        if line.startswith("[") and line.endswith("]"):
            section = line[1:-1].strip()
            continue
        if section != "Core" or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip().replace("\\", "/")
        if not key:
            raise ValueError(f"empty RTAB-Map key in {line}")
        params[key] = value.strip()
    if not params:
        raise ValueError("RTAB-Map ini has no [Core] parameters")
    return params


def load_profile_ini(profile_name: str) -> tuple[Path, dict[str, str]]:
    path = ini_path(profile_name)
    return path, parse_rtabmap_ini(path.read_text(encoding="utf-8"))


def camera_overrides(profile_name: str, camera: str) -> tuple[list[tuple[str, str]], list[tuple[str, str]]]:
    """Slam pairs, then odometry pairs, for this camera mode.

    ``cpu`` stereo still sets ``Stereo/MaxDisparity`` 64. RGB-D does not.
    """
    name = str(profile_name).strip().lower()
    if name not in geo.PROFILES:
        raise ValueError(f"VISION_PROFILE must be cpu, full, or hw, got {profile_name}")
    if camera == "stereo":
        slam = [("Grid/MaxGroundHeight", "1.0"), ("Grid/MaxObstacleHeight", "2.0")]
        if name == "cpu":
            slam.append(("Stereo/MaxDisparity", "64"))
        return slam, []
    if camera == "rgbd":
        return (
            [("Grid/MaxGroundHeight", "0.5"), ("Grid/MaxObstacleHeight", "2.2")],
            [("Vis/DepthAsMask", "true")],
        )
    if camera == "rgbd-wrapper":
        return (
            [
                ("Grid/MaxGroundHeight", "0.5"),
                ("Grid/MaxObstacleHeight", "2.2"),
                ("Grid/RayTracing", "true"),
                ("Grid/3D", "true"),
                ("Grid/FlatObstacleDetected", "true"),
            ],
            [("Vis/DepthAsMask", "true")],
        )
    raise ValueError(f"camera must be stereo, rgbd, or rgbd-wrapper, got {camera}")


def effective_parameters(profile_name: str, camera: str) -> dict[str, str]:
    """Ini values plus the camera-mode overrides. This is what the nodes run."""
    _path, params = load_profile_ini(profile_name)
    merged = dict(params)
    slam, odom = camera_overrides(profile_name, camera)
    for key, value in slam + odom:
        merged[key] = value
    return merged


def launch_strings(profile_name: str, camera: str) -> tuple[Path, str, str]:
    """``cfg`` path, ``args`` (includes ``-d``), and ``odom_args``."""
    path, _params = load_profile_ini(profile_name)
    slam, odom = camera_overrides(profile_name, camera)
    args = ["-d"]
    for key, value in slam:
        args.extend((f"--{key}", value))
    odom_args: list[str] = []
    for key, value in odom:
        odom_args.extend((f"--{key}", value))
    return path, " ".join(args), " ".join(odom_args)


def _rate_text(value: float) -> str:
    if float(value).is_integer():
        return str(int(value))
    return str(value)


def startup_log_lines(profile: geo.VisionProfile, camera: str) -> list[str]:
    path, params = load_profile_ini(profile.name)
    slam, odom = camera_overrides(profile.name, camera)
    lines = [
        (
            f"vision profile={profile.name} camera={camera} "
            f"stereo={profile.stereo_width}x{profile.stereo_height} "
            f"color={profile.color_width}x{profile.color_height} "
            f"camera_hz={_rate_text(profile.camera_hz)} "
            f"imu_hz={_rate_text(profile.imu_hz)} ini={path}"
        )
    ]
    # No GPU line here. This process runs in the Rtabmap container, which
    # does not render and has no /dev. The PX4 container warns when it writes
    # the SDFs Gazebo draws.
    warning = geo.sub_hd_warning(profile)
    if warning:
        lines.append(warning)
    for key in sorted(params):
        lines.append(f"rtabmap_param source=ini {key}={params[key]}")
    for key, value in slam + odom:
        lines.append(f"rtabmap_param source=camera {key}={value}")
    return lines


def shell_assignments(profile: geo.VisionProfile, camera: str) -> str:
    path, args, odom = launch_strings(profile.name, camera)
    return (
        f"RTAB_CFG={shlex.quote(str(path))}\n"
        f"RTAB_ARGS={shlex.quote(args)}\n"
        f"RTAB_ODOM={shlex.quote(odom)}\n"
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Resolve the RTAB-Map profile for this run.")
    parser.add_argument("--camera", required=True, choices=CAMERAS)
    parser.add_argument("--shell", action="store_true", help="Print RTAB_CFG, RTAB_ARGS, and RTAB_ODOM.")
    args = parser.parse_args(argv)
    profile = geo.profile_from_env()
    for line in startup_log_lines(profile, args.camera):
        print(line, file=sys.stderr)
    if args.shell:
        sys.stdout.write(shell_assignments(profile, args.camera))
    return 0


if __name__ == "__main__":
    sys.exit(main())
