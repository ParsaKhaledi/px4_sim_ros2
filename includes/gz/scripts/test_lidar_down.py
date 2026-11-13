"""Downward-lidar patch: one ray, exact PX4 names and poses, off when LIDAR_DOWN=0."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from patch_x500_lidar_down import lidar_fragment, patch_text

ORIGINAL = "<sdf><model name='x500_depth'>\n  <link name='base_link'/>\n</model></sdf>\n"


def test_lidar_down_is_inserted_once_with_px4_names_and_poses():
    patched = patch_text(ORIGINAL, {})
    assert patch_text(patched, {}) == patched
    assert lidar_fragment(30) in patched
    assert '<uri>model://LW20</uri>' in patched
    assert '<pose relative_to="base_link">0 0 -0.079 0 1.57 0</pose>' in patched
    assert 'name="lidar_model_joint"' in patched
    assert "<parent>base_link</parent>" in patched
    assert "<child>lw20_link</child>" in patched
    assert 'name="lidar_sensor_joint"' in patched
    assert "<child>lidar_sensor_link</child>" in patched
    assert '<link name="lidar_sensor_link">' in patched
    assert '<pose relative_to="base_link">0 0 -0.05 0 1.57 0</pose>' in patched
    assert "<mass>0.001</mass>" in patched
    assert "<ixx>0.00001</ixx>" in patched
    assert "<sensor name='lidar' type='gpu_lidar'>" in patched
    assert "<gz_frame_id>lidar_sensor_link</gz_frame_id>" in patched
    assert "<pose>0 0 0 3.14 0 0</pose>" in patched
    assert "<update_rate>30</update_rate>" in patched
    assert patched.count("<samples>1</samples>") == 2
    assert patched.count("<min_angle>0</min_angle>") == 2
    assert patched.count("<max_angle>0</max_angle>") == 2
    assert "<min>0.1</min>" in patched
    assert "<max>100.0</max>" in patched
    assert "<resolution>0.01</resolution>" in patched
    assert "<always_on>1</always_on>" in patched
    assert "<visualize>false</visualize>" in patched
    assert "<link name='base_link'/>" in patched
    assert patched.count("<sensor name='lidar'") == 1
    assert patched.count('<link name="lidar_sensor_link">') == 1


def test_lidar_down_off_removes_the_sensor_and_stays_off():
    patched = patch_text(ORIGINAL, {})
    off = patch_text(patched, {"LIDAR_DOWN": "0"})
    assert off == ORIGINAL
    assert patch_text(ORIGINAL, {"LIDAR_DOWN": "0"}) == ORIGINAL
    rated = patch_text(ORIGINAL, {"LIDAR_DOWN_RATE_HZ": "15"})
    removed = patch_text(rated, {"LIDAR_DOWN": "0"})
    assert "lidar_sensor_link" not in removed
    assert "model://LW20" not in removed
    assert "lidar_model_joint" not in removed
    assert "<link name='base_link'/>" in removed
    assert patch_text(removed, {"LIDAR_DOWN": "0"}) == removed


def test_lidar_rate_change_stays_a_single_sensor():
    fast = patch_text(ORIGINAL, {"LIDAR_DOWN_RATE_HZ": "15"})
    assert "<update_rate>15</update_rate>" in fast
    assert patch_text(fast, {"LIDAR_DOWN_RATE_HZ": "15"}) == fast
    slowed = patch_text(fast, {})
    assert "<update_rate>30</update_rate>" in slowed
    assert slowed.count("<sensor name='lidar'") == 1
    assert slowed.count('<link name="lidar_sensor_link">') == 1
    assert patch_text(slowed, {}) == slowed
