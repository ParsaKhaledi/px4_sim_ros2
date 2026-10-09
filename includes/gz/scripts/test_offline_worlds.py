"""Offline-world checks that do not need the network."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fuel_assets import FILE_URL, find_fuel_references
from patch_x500_ground_truth import (
    COVARIANCE_TOPIC,
    candidate_models,
    covariance_topic_problems,
    ground_truth_model_names,
    ground_truth_models,
    main as patch_ground_truth_main,
    patch_text,
    watched_model_sdfs,
)


def test_fuel_file_url_pattern():
    url = "https://fuel.gazebosim.org/1.0/openrobotics/models/trashbin/1/files/meshes/TrashBin.obj"
    match = FILE_URL.fullmatch(url)
    assert match is not None
    assert match.group(2) == "trashbin"
    assert match.group(4) == "meshes/TrashBin.obj"


def test_missing_worlds_directory_fails():
    from check_offline_worlds import main

    assert main(["/tmp/px4-worlds-missing-for-test"]) == 1


def test_find_fuel_references(tmp_path):
    world = tmp_path / "demo.sdf"
    world.write_text("<uri>https://fuel.gazebosim.org/1.0/openrobotics/models/table/3/files/Table_Diffuse.jpg</uri>\n")
    hits = find_fuel_references(tmp_path)
    assert len(hits) == 1
    clean = tmp_path / "clean.sdf"
    clean.write_text("<uri>model://table/Table_Diffuse.jpg</uri>\n")
    world.unlink()
    assert find_fuel_references(tmp_path) == []


def test_ground_truth_plugin_is_inserted_once():
    original = "<sdf><model name='x500_depth'>\n</model></sdf>\n"
    patched = patch_text(original)
    assert "OdometryPublisher" in patched
    assert "/ground_truth/odom" in patched
    assert f"<odom_covariance_topic>{COVARIANCE_TOPIC}</odom_covariance_topic>" in patched
    assert covariance_topic_problems(patched) == []
    assert patched.count("OdometryPublisher") == 1
    assert "base_link_gt" in patched
    assert patch_text(patched) == patched
    stale = patched.replace("base_link_gt", "base_link")
    assert "base_link_gt" in patch_text(stale)


def test_existing_plugin_gains_a_safe_covariance_topic():
    original = "<sdf><model name='x500_depth'>\n</model></sdf>\n"
    patched = patch_text(original)
    older = patched.replace(
        f"\n      <odom_covariance_topic>{COVARIANCE_TOPIC}</odom_covariance_topic>",
        "",
    )
    assert "odom_covariance_topic" not in older
    repaired = patch_text(older)
    assert f"<odom_covariance_topic>{COVARIANCE_TOPIC}</odom_covariance_topic>" in repaired
    assert covariance_topic_problems(repaired) == []
    assert patch_text(repaired) == repaired

    leaky = repaired.replace(
        COVARIANCE_TOPIC,
        "/model/x500_depth_0/odometry_with_covariance",
    )
    sealed = patch_text(leaky)
    assert f"<odom_covariance_topic>{COVARIANCE_TOPIC}</odom_covariance_topic>" in sealed
    assert "/model/" not in sealed
    assert patch_text(sealed) == sealed


def test_model_sdfs_keep_odometry_off_the_px4_vision_topic():
    missing = (
        '<plugin filename="gz-sim-odometry-publisher-system" '
        'name="gz::sim::systems::OdometryPublisher">'
        "<odom_topic>/ground_truth/odom</odom_topic>"
        "</plugin>"
    )
    prefixed = missing.replace(
        "</plugin>",
        "<odom_covariance_topic>/model/x500_depth_0/odometry_with_covariance</odom_covariance_topic></plugin>",
    )
    assert covariance_topic_problems(missing) == ["OdometryPublisher has no odom_covariance_topic"]
    assert covariance_topic_problems(prefixed) == [
        "odom_covariance_topic /model/x500_depth_0/odometry_with_covariance starts with /model/"
    ]
    problems = []
    for path in watched_model_sdfs():
        found = covariance_topic_problems(path.read_text(encoding="utf-8"))
        problems.extend(f"{path}: {item}" for item in found)
    assert problems == []


def test_candidate_model_paths_are_unique(monkeypatch):
    monkeypatch.setattr(
        "patch_x500_ground_truth.Path.home",
        staticmethod(lambda: Path("/home/px4")),
    )
    monkeypatch.setenv("PX4_GZ_MODELS", "/home/px4/PX4-Autopilot/Tools/simulation/gz/models")
    resolved = [path.resolve() for path in candidate_models()]
    assert len(resolved) == 1


def test_ground_truth_targets_include_spawned_x500(monkeypatch):
    monkeypatch.delenv("PX4_SIM_MODEL", raising=False)
    monkeypatch.setenv("PX4_GZ_MODEL", "x500")
    assert ground_truth_model_names() == ["x500_depth", "x500"]
    monkeypatch.setenv("PX4_GZ_MODEL", "x500_depth_0")
    assert ground_truth_model_names() == ["x500_depth"]
    monkeypatch.delenv("PX4_GZ_MODEL")
    monkeypatch.setenv("PX4_SIM_MODEL", "gz_x500")
    assert ground_truth_model_names() == ["x500_depth", "x500"]


def test_main_patches_spawned_x500(tmp_path, monkeypatch):
    models = tmp_path / "models"
    sdf = "<sdf><model name='m'>\n</model></sdf>\n"
    for name in ("x500", "x500_depth"):
        folder = models / name
        folder.mkdir(parents=True)
        (folder / "model.sdf").write_text(sdf, encoding="utf-8")
    monkeypatch.setenv("PX4_GZ_MODELS", str(models))
    monkeypatch.setenv("PX4_GZ_MODEL", "x500")
    monkeypatch.delenv("PX4_SIM_MODEL", raising=False)
    monkeypatch.setattr(
        "patch_x500_ground_truth.Path.home",
        staticmethod(lambda: tmp_path / "no-px4-home"),
    )
    assert patch_ground_truth_main() == 0
    plain = (models / "x500" / "model.sdf").read_text(encoding="utf-8")
    depth = (models / "x500_depth" / "model.sdf").read_text(encoding="utf-8")
    assert "OdometryPublisher" in plain
    assert "/ground_truth/odom" in plain
    assert "OdometryPublisher" in depth
    names = {path.parent.name for path in ground_truth_models()}
    assert "x500" in names
    assert "x500_depth" in names
