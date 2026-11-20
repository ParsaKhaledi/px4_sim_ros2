"""Static transform tree. No Gazebo.

The chain is the RTAB-Map launch contract plus the x500 URDF:

    map -> odom -> base_link -> camera_link -> camera frame -> optical frame

``map`` and ``odom`` are not URDF links. ``world`` is a second root and is
not a parent of ``base_link``.
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)
CHAIN = (
    "map",
    "odom",
    "base_link",
    "camera_link",
    "camera_rgb_frame",
    "camera_rgb_optical_frame",
)


@dataclass
class TreeReport:
    errors: list[str] = field(default_factory=list)
    findings: list[dict] = field(default_factory=list)
    parents: dict[str, str] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return not self.errors


def _repo(root: Path | None) -> Path:
    if root is not None:
        return root
    return Path(__file__).resolve().parents[2]


def _xyz(text: str) -> tuple[float, float, float, float, float, float]:
    parts = [float(item) for item in text.split()]
    if len(parts) != 6:
        raise ValueError(f"expected 6 pose numbers, got {text!r}")
    return parts[0], parts[1], parts[2], parts[3], parts[4], parts[5]


def _urdf_joints(path: Path) -> list[dict]:
    tree = ET.parse(path)
    joints = []
    for joint in tree.getroot().findall("joint"):
        origin = joint.find("origin")
        parent = joint.find("parent")
        child = joint.find("child")
        xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
        rpy = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
        pose = _xyz(f"{xyz} {rpy}")
        joints.append({
            "name": joint.get("name") or "",
            "parent": parent.get("link") if parent is not None else "",
            "child": child.get("link") if child is not None else "",
            "pose": pose,
        })
    return joints


def _add_parent(parents: dict[str, list[str]], child: str, parent: str) -> None:
    parents.setdefault(child, [])
    if parent not in parents[child]:
        parents[child].append(parent)


def _optical_ok(rotation) -> bool:
    """Optical +z forward, +x right, +y down in the camera link."""
    import numpy as np

    matrix = np.asarray(rotation, dtype=float)
    forward = matrix @ np.array([0.0, 0.0, 1.0])
    right = matrix @ np.array([1.0, 0.0, 0.0])
    down = matrix @ np.array([0.0, 1.0, 0.0])
    return (
        np.allclose(forward, [1.0, 0.0, 0.0], atol=1e-6)
        and np.allclose(right, [0.0, -1.0, 0.0], atol=1e-6)
        and np.allclose(down, [0.0, 0.0, -1.0], atol=1e-6)
    )


def check_tree(root: Path | None = None) -> TreeReport:
    """Read the URDF, Oak-D SDFs, and RTAB-Map launch. Do not start Gazebo."""
    from px4_control.frames import rpy_to_rot

    repo = _repo(root)
    report = TreeReport()
    urdf_path = repo / "includes/gz/x500_tf_publisher/x500_urdf.urdf"
    joints = _urdf_joints(urdf_path)
    grouped: dict[str, list[str]] = {}
    by_child = {item["child"]: item for item in joints}
    for item in joints:
        _add_parent(grouped, item["child"], item["parent"])

    launch = (repo / "includes/gz/startFiles/gz_start_rtabmap.sh").read_text(encoding="utf-8")
    if "frame_id:=base_link" not in launch or "vo_frame_id:=odom" not in launch:
        report.errors.append("RTAB-Map launch does not publish odom -> base_link")
    else:
        _add_parent(grouped, "base_link", "odom")
    if "map_frame_id:=map" not in launch:
        report.errors.append("RTAB-Map launch does not set map_frame_id:=map")
    else:
        _add_parent(grouped, "odom", "map")
    if "odom_frame_id:=" in launch:
        report.errors.append("RTAB-Map launch sets odom_frame_id; that consumes TF odom")

    spawn = (repo / "includes/gz/sim_ws/src/sim_monitor/sim_monitor/spawn_frame.py").read_text(encoding="utf-8")
    if 'child_frame_id = "base_link"' in spawn:
        report.errors.append("spawn_frame parents base_link; odom is already its parent")

    for child, parents in sorted(grouped.items()):
        if len(parents) > 1:
            report.errors.append(f"{child} has parents {parents}")
        else:
            report.parents[child] = parents[0]

    link = CHAIN[-1]
    seen = [link]
    while link in report.parents:
        link = report.parents[link]
        seen.append(link)
    seen.reverse()
    if tuple(seen) != CHAIN:
        report.errors.append(f"chain is {seen}, expected {list(CHAIN)}")

    for name in (
        "stereo_left_camera_optical_frame",
        "stereo_right_camera_optical_frame",
        "camera_rgb_optical_frame",
    ):
        joint = by_child.get(name)
        if joint is None:
            report.errors.append(f"URDF has no joint for {name}")
            continue
        roll, pitch, yaw = joint["pose"][3:]
        if not all(math.isclose(got, want, abs_tol=1e-5) for got, want in zip((roll, pitch, yaw), OPTICAL_RPY)):
            report.errors.append(f"{name} rpy is {(roll, pitch, yaw)}, expected {OPTICAL_RPY}")
        elif not _optical_ok(rpy_to_rot(roll, pitch, yaw)):
            report.errors.append(f"{name} rpy is not z-forward, x-right, y-down")

    _sensor_poses(repo, by_child, report)
    _frame_ids(repo, report)
    _check_camera_mount(repo, by_child, report)
    _findings(repo, by_child, report)
    return report


def _sensor_poses(repo: Path, by_child: dict, report: TreeReport) -> None:
    from geometry import sensor_layouts

    layouts = sensor_layouts()
    urdf_names = {
        "stereo_left": "stereo_left_camera_frame",
        "stereo_right": "stereo_right_camera_frame",
        "rgb": "camera_rgb_frame",
    }
    sdf_paths = {
        "rgb": repo / "includes/gz/models/OakD-Lite-rgbd/model.sdf",
        "stereo_left": repo / "includes/gz/models/OakD-Lite-stereo/model.sdf",
        "stereo_right": repo / "includes/gz/models/OakD-Lite-stereo/model.sdf",
    }
    for key, link in urdf_names.items():
        joint = by_child.get(link)
        if joint is None:
            report.errors.append(f"URDF has no {link}")
            continue
        xyz = joint["pose"][:3]
        want = layouts[key]
        if not all(math.isclose(got, exp, abs_tol=1e-4) for got, exp in zip(xyz, want)):
            report.errors.append(f"URDF {link} xyz {xyz} != geometry {want}")
        text = sdf_paths[key].read_text(encoding="utf-8")
        needle = f"{want[0]:.6f} {want[1]:.6f} {want[2]:.6f}"
        if needle not in text:
            report.errors.append(f"{sdf_paths[key].name} has no sensor pose {needle}")


def _frame_ids(repo: Path, report: TreeReport) -> None:
    expected = {
        repo / "includes/gz/models/OakD-Lite-rgbd/model.sdf": "camera_rgb_optical_frame",
        repo / "includes/gz/models/OakD-Lite-stereo/model.sdf": "stereo_left_camera_optical_frame",
    }
    for path, frame in expected.items():
        text = path.read_text(encoding="utf-8")
        if f"<gz_frame_id>{frame}</gz_frame_id>" not in text:
            report.errors.append(f"{path.name} image frame is not {frame}")
        if f"<optical_frame_id>{frame}</optical_frame_id>" not in text:
            report.errors.append(f"{path.name} camera_info frame is not {frame}")
    stereo = (repo / "includes/gz/models/OakD-Lite-stereo/model.sdf").read_text(encoding="utf-8")
    right = "stereo_right_camera_optical_frame"
    if stereo.count(f"<optical_frame_id>{right}</optical_frame_id>") < 1:
        report.errors.append("stereo camera_info is missing the right optical frame")
    if stereo.count(f"<gz_frame_id>{right}</gz_frame_id>") < 1:
        report.errors.append("stereo image is missing the right optical frame")


def _check_camera_mount(repo: Path, by_child: dict, report: TreeReport) -> None:
    """URDF camera_joint against geometry.Mount(), the pose Gazebo gets at runtime.

    ``render_oakd.py --patch-x500`` writes Mount() into the PX4 copy of
    x500_depth. The committed model.sdf is the unpatched overlay (pitch 0)
    and is not the mount the flight uses.
    """
    from geometry import Mount
    from render_oakd import patch_x500_depth_xml

    mount = Mount()
    joint = by_child.get("camera_link")
    if joint is None:
        report.errors.append("URDF has no camera_link joint")
        return
    expected = (mount.x, mount.y, mount.z, 0.0, mount.pitch_rad, 0.0)
    pose = joint["pose"]
    if not all(math.isclose(got, want, abs_tol=1e-4) for got, want in zip(pose, expected)):
        report.errors.append(f"URDF camera_joint {tuple(round(v, 6) for v in pose)} != Mount() {expected}")
    committed = (repo / "includes/gz/models/x500_depth/model.sdf").read_text(encoding="utf-8")
    patched = patch_x500_depth_xml(committed, mount)
    needle = mount.pose_text()
    if patched.count(needle) < 2:
        report.errors.append(
            "render_oakd.py --patch-x500 did not write Mount() onto the include and CameraJoint"
        )


def _findings(repo: Path, by_child: dict, report: TreeReport) -> None:
    yaml = (repo / "ros2_ws/src/px4_control/config/px4_control.yaml").read_text(encoding="utf-8")
    # base_link is the URDF root. The 0.24 m figure is a comment, not a joint.
    if "0.24" in yaml and "base_link" not in by_child:
        report.findings.append({
            "id": "base_link_height",
            "detail": (
                "px4_control.yaml says base_link is 0.24 m above the model origin "
                "and the lidar is 5 cm below it. The URDF root is base_link, so "
                "that offset is not in the static tree. camera_joint z=0.242 is "
                "the camera mount, not this offset."
            ),
        })

    profile = (repo / "includes/gz/startFiles/rtabmap_profiles/cpu.ini").read_text(encoding="utf-8")
    if re.search(r"RGBD[/\\]StartAtOrigin=true", profile):
        report.findings.append({
            "id": "start_at_origin",
            "detail": (
                "cpu.ini sets RGBD/StartAtOrigin=true, so the first RTAB-Map pose is "
                "identity and the odom axes lie on the body at lock. The vision bridge "
                "treats an identity quaternion as facing East. At a non-zero spawn yaw "
                "those two disagree by that yaw. The fake-vision flight keeps world-ENU "
                "axes and only shifts the origin."
            ),
        })

