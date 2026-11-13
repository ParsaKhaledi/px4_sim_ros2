"""Static checks for RTAB-Map / Nav2 TF launch contracts.

See docs/tf_requirements.md. These do not start ROS.
"""

from __future__ import annotations

from pathlib import Path

import yaml

GZ = Path(__file__).resolve().parents[1]
START = GZ / "startFiles"
NAV2 = GZ / "Params" / "nav2" / "nav2_params.yaml"
URDF = GZ / "x500_tf_publisher" / "x500_urdf.urdf"

RTABMAP_SCRIPTS = (
    START / "gz_start_rtabmap.sh",
    START / "gz_start_rtabmap_rgbd.sh",
    START / "gz_start_rtabmap_stereo.sh",
)


def test_rtabmap_scripts_use_base_link_and_sim_time():
    for path in RTABMAP_SCRIPTS:
        text = path.read_text(encoding="utf-8")
        assert "frame_id:=base_link" in text, path.name
        assert "oak-d-base-frame" not in text, path.name
        assert "frame_id:=camera_rgb_frame" not in text, path.name
        assert "use_sim_time:=false" not in text, path.name
        assert "use_sim_time:=${USE_SIM_TIME" in text or "use_sim_time:=true" in text, path.name
        assert "map_frame_id:=map" in text, path.name
        assert "vo_frame_id:=odom" in text, path.name
        # odom_frame_id would make SLAM consume TF odom instead of VO — must stay unset.
        assert "odom_frame_id:=" not in text, path.name


def test_nav2_frames_match_rtabmap_base_link():
    data = yaml.safe_load(NAV2.read_text(encoding="utf-8"))
    assert data["bt_navigator"]["ros__parameters"]["global_frame"] == "map"
    assert data["bt_navigator"]["ros__parameters"]["robot_base_frame"] == "base_link"
    assert data["bt_navigator"]["ros__parameters"]["odom_topic"] == "/rtabmap/odom"
    local = data["local_costmap"]["local_costmap"]["ros__parameters"]
    assert local["global_frame"] == "odom"
    assert local["robot_base_frame"] == "base_link"
    glob = data["global_costmap"]["global_costmap"]["ros__parameters"]
    assert glob["global_frame"] == "map"
    assert glob["robot_base_frame"] == "base_link"
    assert local["voxel_layer"]["pointcloud"]["topic"] == "/rtabmap/cloud_obstacles"
    assert glob["obstacle_layer"]["pointcloud"]["topic"] == "/rtabmap/cloud_obstacles"


def test_urdf_has_base_link_and_optical_frames():
    text = URDF.read_text(encoding="utf-8")
    for name in (
        'link name="base_link"',
        "camera_rgb_optical_frame",
        "stereo_left_camera_optical_frame",
        "imu_link",
    ):
        assert name in text
    # Camera module root: oakd_s2 overlay uses camera_link; older Fuel Oak-D used OakD-Lite/base_link.
    assert ('link name="camera_link"' in text) or ("OakD-Lite/base_link" in text)


def test_spawn_frame_never_parents_base_link():
    spawn = (GZ / "sim_ws" / "src" / "sim_monitor" / "sim_monitor" / "spawn_frame.py").read_text(
        encoding="utf-8"
    )
    assert 'child_frame_id = "spawn"' in spawn
    assert "base_link_gt" in spawn
    assert 'child_frame_id = "base_link"' not in spawn
    assert "GT_CHILD_FRAME" in spawn
