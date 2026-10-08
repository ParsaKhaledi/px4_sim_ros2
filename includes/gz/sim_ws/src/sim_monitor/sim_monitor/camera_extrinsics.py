"""Static camera transforms parsed from the x500_depth and Oak-D SDFs.

The mount comes from the x500_depth fixed joint (PX4-gazebo-models:
``.12 .03 .242 0 0 0``). The child frame is the link name in the Oak-D
model that is actually loaded. On this branch that link is
``OakD-Lite/base_link``. The upstream joint child is ``camera_link``; the
parser uses the model link so the TF name matches the simulated body.

Sensor poses, including the downward pitch on this branch's cameras, are
read from the sensor ``<pose>``. ``CAM_PITCH_DEG`` (positive looks down),
``CAM_X``, ``CAM_Y``, and ``CAM_Z`` override the mount when they are set.
Those are the same variables the vision branch uses for the mount.

Gazebo cameras look along +X. The optical child is the REP-103 frame
(z forward, x right, y down), ``rpy = (-pi/2, 0, -pi/2)``. That rotation
is not in the SDF.
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

REPO_GZ = Path(__file__).resolve().parents[4]
REFERENCE_X500 = Path(__file__).resolve().parent / "data" / "x500_depth_model.sdf"
OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)
BASE_FRAME = "base_link"


@dataclass(frozen=True)
class StaticTF:
    parent: str
    child: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


def parse_pose(text: str) -> tuple[float, float, float, float, float, float]:
    """``x y z roll pitch yaw`` in metres and radians."""
    parts = text.split()
    if len(parts) != 6:
        raise ValueError(f"pose needs 6 numbers, got {text!r}")
    x, y, z, roll, pitch, yaw = (float(part) for part in parts)
    return x, y, z, roll, pitch, yaw


def _first_pose(element: ET.Element) -> tuple[float, float, float, float, float, float] | None:
    pose = element.find("pose")
    if pose is None or not (pose.text and pose.text.strip()):
        return None
    return parse_pose(pose.text)


def mount_pose_from_x500(text: str) -> tuple[float, float, float, float, float, float]:
    """Camera joint pose, or the Oak-D include pose when the joint has none."""
    root = ET.fromstring(text)
    for joint in root.iter("joint"):
        pose = _first_pose(joint)
        if pose is not None:
            return pose
    for include in root.iter("include"):
        pose = _first_pose(include)
        if pose is not None:
            return pose
    raise ValueError("x500_depth SDF has no camera pose")


def apply_mount_env(
    pose: tuple[float, float, float, float, float, float],
    environ: dict[str, str] | None = None,
) -> tuple[float, float, float, float, float, float]:
    """Replace mount fields that the vision branch exposes in the environment."""
    env = os.environ if environ is None else environ
    x, y, z, roll, pitch, yaw = pose

    def _override(name: str, current: float) -> float:
        raw = env.get(name, "")
        if raw is None or str(raw).strip() == "":
            return current
        return float(raw)

    x = _override("CAM_X", x)
    y = _override("CAM_Y", y)
    z = _override("CAM_Z", z)
    pitch_deg = env.get("CAM_PITCH_DEG", "")
    if pitch_deg is not None and str(pitch_deg).strip() != "":
        # Positive degrees look down, which is a positive pitch about Y.
        pitch = math.radians(float(pitch_deg))
    return x, y, z, roll, pitch, yaw


def _camera_kind(camera_type: str | None) -> str:
    kind = "rgbd" if camera_type is None or str(camera_type).strip() == "" else str(camera_type).strip().lower()
    if kind in {"stereo"}:
        return "stereo"
    if kind in {"rgbd", "rgbd-d"}:
        return "rgbd"
    raise ValueError(f"CameraType must be rgbd or stereo, got {camera_type!r}")


def oak_variant_name(camera_type: str | None) -> str:
    return "OakD-Lite-stereo" if _camera_kind(camera_type) == "stereo" else "OakD-Lite-rgbd"


def _existing(paths: list[Path]) -> Path | None:
    for path in paths:
        if path.is_file():
            return path
    return None


def x500_model_path() -> Path:
    """Live PX4 x500_depth model, or the reference copy in this package."""
    candidates: list[Path] = []
    models = os.environ.get("PX4_GZ_MODELS", "")
    if models:
        candidates.append(Path(models) / "x500_depth" / "model.sdf")
    candidates.append(Path("/home/px4/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf"))
    candidates.append(Path.home() / "PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf")
    candidates.append(REFERENCE_X500)
    found = _existing(candidates)
    if found is None:
        raise FileNotFoundError("x500_depth model.sdf was not found")
    return found


def oak_model_path(camera_type: str | None = None) -> Path:
    """Installed ``OakD-Lite`` model after the start copy, else the repo variant."""
    variant = oak_variant_name(camera_type if camera_type is not None else os.environ.get("CameraType", ""))
    candidates: list[Path] = []
    models = os.environ.get("PX4_GZ_MODELS", "")
    if models:
        candidates.append(Path(models) / "OakD-Lite" / "model.sdf")
    px4_models = Path("/home/px4/PX4-Autopilot/Tools/simulation/gz/models")
    candidates.append(px4_models / "OakD-Lite" / "model.sdf")
    candidates.append(REPO_GZ / "models" / variant / "model.sdf")
    found = _existing(candidates)
    if found is None:
        raise FileNotFoundError(f"{variant} model.sdf was not found")
    return found


def _sensor_frame(sensor: ET.Element) -> str:
    frame = sensor.findtext("gz_frame_id")
    if frame and frame.strip():
        return frame.strip()
    name = sensor.get("name")
    if not name:
        raise ValueError("sensor has no name and no gz_frame_id")
    return name


def optical_child(frame: str) -> str | None:
    """Optical child of a Gazebo camera frame. None when ``frame`` is already optical."""
    if frame.endswith("_optical_frame"):
        return None
    if frame.endswith("_frame"):
        return frame[: -len("_frame")] + "_optical_frame"
    return frame + "_optical_frame"


def camera_transforms(
    x500_sdf: str,
    oak_sdf: str,
    environ: dict[str, str] | None = None,
) -> list[StaticTF]:
    """``base_link`` to the camera link, then each sensor and its optical frame."""
    mount = apply_mount_env(mount_pose_from_x500(x500_sdf), environ)
    oak = ET.fromstring(oak_sdf)
    link = oak.find(".//link")
    if link is None or not link.get("name"):
        raise ValueError("Oak-D SDF has no link")
    camera_link = link.get("name")
    transforms = [
        StaticTF(BASE_FRAME, camera_link, mount[:3], mount[3:]),
    ]
    for sensor in list(link):
        if sensor.tag != "sensor":
            continue
        kind = (sensor.get("type") or "").strip()
        if kind not in {"imu", "camera", "depth_camera"}:
            continue
        pose = _first_pose(sensor)
        if pose is None:
            pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        frame = _sensor_frame(sensor)
        transforms.append(StaticTF(camera_link, frame, pose[:3], pose[3:]))
        if kind in {"camera", "depth_camera"}:
            child = optical_child(frame)
            if child is not None:
                transforms.append(StaticTF(frame, child, (0.0, 0.0, 0.0), OPTICAL_RPY))
    return transforms


def load_camera_transforms(environ: dict[str, str] | None = None) -> list[StaticTF]:
    """Read the models from disk and return the static transforms."""
    env = os.environ if environ is None else environ
    camera_type = env.get("CameraType", "")
    x500 = x500_model_path().read_text(encoding="utf-8")
    oak = oak_model_path(camera_type).read_text(encoding="utf-8")
    return camera_transforms(x500, oak, env)
