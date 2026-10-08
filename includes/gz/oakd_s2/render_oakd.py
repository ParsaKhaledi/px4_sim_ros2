#!/usr/bin/env python3
"""Render the OAK-D S2 SDF and URDF from includes/gz/oakd_s2/geometry.py.

Run with no arguments to refresh the model files and the x500 URDF from the
CAM_* environment variables. ``--urdf-only`` is what the state publisher
container runs. ``--patch-x500`` writes the same mount into PX4's x500_depth
model, which is the file Gazebo actually includes.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import geometry as geo
from profiles import rate_text
from sdf import fmt, indent, render_sdf
from urdf import render_urdf
from x500_patch import POSE_RE, patch_x500_depth_file, patch_x500_depth_xml

# Names that used to live in this file. Callers can still import them here.
__all__ = [
    "GZ_ROOT",
    "POSE_RE",
    "RGBD_SDF",
    "STEREO_SDF",
    "URDF_PATH",
    "fmt",
    "gazebo_startup_warnings",
    "indent",
    "main",
    "patch_x500_depth_file",
    "patch_x500_depth_xml",
    "rate_text",
    "render_sdf",
    "render_urdf",
    "write_models",
]

GZ_ROOT = Path(__file__).resolve().parent.parent
STEREO_SDF = GZ_ROOT / "models" / "OakD-Lite-stereo" / "model.sdf"
RGBD_SDF = GZ_ROOT / "models" / "OakD-Lite-rgbd" / "model.sdf"
URDF_PATH = GZ_ROOT / "x500_tf_publisher" / "x500_urdf.urdf"


def write_models(
    mount: geo.Mount | None = None,
    urdf_only: bool = False,
    profile: geo.VisionProfile | None = None,
) -> None:
    """Write the URDF, and both SDFs unless ``urdf_only`` is set."""
    mount = mount or geo.mount_from_env()
    profile = geo.profile_from_env() if profile is None else profile
    # The URDF carries the mount and the frame tree. Image size and rate live
    # in the SDF, which Gazebo renders. Both are written from this one call.
    URDF_PATH.write_text(render_urdf(mount), encoding="utf-8")
    if urdf_only:
        return
    STEREO_SDF.write_text(render_sdf("stereo", mount, profile), encoding="utf-8")
    RGBD_SDF.write_text(render_sdf("rgbd", mount, profile), encoding="utf-8")


def gazebo_startup_warnings(
    profile: geo.VisionProfile,
    *,
    write_sdf: bool,
    gpu_present: bool | None = None,
) -> list[str]:
    """Warnings for a render. The GPU line is only for the SDFs Gazebo draws.

    The PX4 container writes those SDFs and has ``/dev``. The state publisher
    writes the URDF only. RTAB-Map does not render.
    """
    lines: list[str] = []
    if write_sdf:
        gpu_warning = geo.no_gpu_warning(profile, gpu_present=gpu_present)
        if gpu_warning:
            lines.append(gpu_warning)
    hd = geo.sub_hd_warning(profile)
    if hd:
        lines.append(hd)
    return lines


def main(argv: list[str] | None = None) -> int:
    """Render the models from the environment, or patch one x500_depth file."""
    parser = argparse.ArgumentParser(description="Render the OAK-D S2 SDF and URDF.")
    parser.add_argument("--urdf-only", action="store_true")
    parser.add_argument("--patch-x500", type=Path, default=None)
    args = parser.parse_args(argv)
    mount = geo.mount_from_env()
    profile = geo.profile_from_env()
    if args.patch_x500:
        pose = patch_x500_depth_file(args.patch_x500, mount)
        print(f"x500_depth camera pose set to {pose}")
        return 0
    write_models(mount, urdf_only=args.urdf_only, profile=profile)
    what = "URDF" if args.urdf_only else "stereo SDF, RGB-D SDF, and URDF"
    for warning in gazebo_startup_warnings(profile, write_sdf=not args.urdf_only):
        print(warning, file=sys.stderr)
    print(
        f"Wrote {what} for mount {mount.pose_text()} "
        f"profile {profile.name} stereo {profile.stereo_width}x{profile.stereo_height} "
        f"color {profile.color_width}x{profile.color_height} "
        f"camera {rate_text(profile.camera_hz)} Hz imu {rate_text(profile.imu_hz)} Hz"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
