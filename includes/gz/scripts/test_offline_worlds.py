"""Offline-world checks that do not need the network."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from fuel_assets import FILE_URL, find_fuel_references
from patch_x500_ground_truth import candidate_models, patch_text


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
    assert patched.count("OdometryPublisher") == 1
    assert "base_link_gt" in patched
    assert patch_text(patched) == patched
    stale = patched.replace("base_link_gt", "base_link")
    assert "base_link_gt" in patch_text(stale)


def test_candidate_model_paths_are_unique(monkeypatch):
    monkeypatch.setattr(
        "patch_x500_ground_truth.Path.home",
        staticmethod(lambda: Path("/home/px4")),
    )
    monkeypatch.setenv("PX4_GZ_MODELS", "/home/px4/PX4-Autopilot/Tools/simulation/gz/models")
    resolved = [path.resolve() for path in candidate_models()]
    assert len(resolved) == 1
