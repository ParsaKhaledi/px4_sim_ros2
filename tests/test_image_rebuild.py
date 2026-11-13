from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import image_rebuild_paths  # noqa: E402


def test_dockerfile_and_manifest_rebuild():
    assert image_rebuild_paths.needs_rebuild(["dockerFile/Dockerfile_px4_sim_NO_GPU"])
    assert image_rebuild_paths.needs_rebuild(["versions.env"])
    assert image_rebuild_paths.needs_rebuild(["includes/gz/patch_dds_topics.py"])
    assert image_rebuild_paths.needs_rebuild(["ros2_ws/src/demo/package.xml"])


def test_runtime_files_do_not_rebuild():
    assert not image_rebuild_paths.needs_rebuild([
        "compose.yml",
        "config/px4/params/sim.params",
        "includes/gz/worlds/apt_world.sdf",
        "scripts/run_e2e.sh",
        "docs/templates/FOLDER_README.md",
    ])
