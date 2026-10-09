from pathlib import Path


def test_ci_is_one_headless_hover():
    names = {path.name for path in Path(".github/workflows").glob("*.yml")}
    assert names == {"ci.yml"}
    assert not Path(".github/actions").exists()
    text = Path(".github/workflows/ci.yml").read_text(encoding="utf-8")
    assert "Dockerfile_px4_sim_NO_GPU" in text
    assert "Dockerfile_px4_sim_with_GPU" not in text
    assert "Headless hover/smoke" in text
    assert 'CameraType: none' in text
    assert "PX4_GZ_MODEL: x500" in text
    assert "./scripts/smoke_test.sh" in text
    assert "px4_offboard.py" not in text
    assert "microxrce_offboard.py" not in text
    assert "run_e2e.sh" not in text
    assert "continue-on-error" not in text
    assert "docker load" in text
    assert 'cache_from="type=gha,scope=${cache_scope},version=2"' in text
    assert "mode=max,version=2,timeout=30m" in text
    assert "ACTIONS_RESULTS_URL" in text
