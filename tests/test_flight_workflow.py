from pathlib import Path


def test_ci_runs_none_rgbd_stereo_flight_matrix():
    names = {path.name for path in Path(".github/workflows").glob("*.yml")}
    assert names == {"ci.yml"}
    assert not Path(".github/actions").exists()
    text = Path(".github/workflows/ci.yml").read_text(encoding="utf-8")
    assert "Dockerfile_px4_sim_NO_GPU" in text
    assert "Dockerfile_px4_sim_with_GPU" not in text
    # Permanent matrix: plain x500 smoke + OAK-D RGB-D/stereo hover+out-and-back.
    assert "matrix:" in text
    assert "camera: [none, rgbd, stereo]" in text
    assert "CameraType: none" in text
    assert "CameraType: rgbd" in text
    assert "CameraType: stereo" in text
    assert "PX4_GZ_MODEL: x500" in text
    assert "PX4_GZ_MODEL: x500_depth" in text
    assert "VISION_PROFILE: cpu" in text
    assert "./scripts/smoke_test.sh" in text
    assert "./scripts/ci_rgbd_flight.sh" in text
    assert "./scripts/ci_stereo_flight.sh" in text
    camera_script = Path("scripts/ci_camera_flight.sh").read_text(encoding="utf-8")
    assert "unset E2E_HOVER_ONLY" in camera_script
    assert "test_out_and_back.py" in camera_script
    assert "x500_tf_publisher" in camera_script
    assert "a+rwX" in camera_script
    assert "ESTIMATION_MODE=gps" in camera_script
    assert "PX4_PARAM_EKF2_MAG_TYPE=0" in camera_script
    assert "GZ_USE_XVFB" in camera_script
    assert "unset HEADLESS" in camera_script
    assert "ESTIMATION_MODE: gps" in text
    installer = Path(
        "includes/gz/params/install_px4_control_params.bash"
    ).read_text(encoding="utf-8")
    assert "EKF2_MAG_TYPE=0" in installer
    xvfb = Path("compose.xvfb.yml").read_text(encoding="utf-8")
    assert "HEADLESS_BACKEND: xvfb" in xvfb
    assert 'HEADLESS: ""' in xvfb
    start = Path("includes/gz/startFiles/gz_start_px4_gz_sim.sh").read_text(
        encoding="utf-8"
    )
    assert "unset HEADLESS" in start
    assert "Xvfb GUI" in start
    stack = Path("scripts/compose_stack.sh").read_text(encoding="utf-8")
    assert "network_mode" not in stack  # DDS netns lives in compose.ci.yml
    assert "x500_tf_publisher" in stack
    assert "GZ_USE_XVFB" in stack
    assert "E2E_HOVER_ONLY=1" not in Path("scripts/ci_rgbd_flight.sh").read_text(
        encoding="utf-8"
    )
    assert "E2E_HOVER_ONLY=1" not in Path("scripts/ci_stereo_flight.sh").read_text(
        encoding="utf-8"
    )
    assert "microxrce_offboard.py" not in text
    assert "continue-on-error" not in text
    assert "docker load" in text
    assert 'cache_from="type=gha,scope=${cache_scope},version=2"' in text
    assert "mode=max,version=2,timeout=30m" in text
    assert "ACTIONS_RESULTS_URL" in text
