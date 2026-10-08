from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "includes" / "gz"))

import patch_dds_topics  # noqa: E402

SAMPLE = """
publications:
  - topic: /fmu/out/vehicle_local_position
    type: px4_msgs::msg::VehicleLocalPosition

subscriptions:
  - topic: /fmu/in/distance_sensor
    type: px4_msgs::msg::DistanceSensor
"""


def test_patch_adds_the_outbound_topic_once():
    once, changed = patch_dds_topics.ensure_distance_sensor(SAMPLE)
    assert changed
    assert once.count("/fmu/out/distance_sensor") == 1
    assert "px4_msgs::msg::DistanceSensor" in once.split("subscriptions:")[0]
    twice, again = patch_dds_topics.ensure_distance_sensor(once)
    assert not again
    assert twice == once


def test_dockerfiles_patch_before_the_firmware_build():
    root = Path(__file__).resolve().parents[1]
    for name in ("Dockerfile_px4_sim_NO_GPU", "Dockerfile_px4_sim_with_GPU"):
        text = (root / "dockerFile" / name).read_text(encoding="utf-8")
        patch = text.index("patch_dds_topics.py")
        build = text.index("make check_px4_sitl_default")
        assert patch < build
