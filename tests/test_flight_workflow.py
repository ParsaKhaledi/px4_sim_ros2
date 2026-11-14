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
    assert "ESTIMATION_MODE=vision" in camera_script
    assert "EKF2_EV_CTRL=15" in camera_script
    assert "EKF2_MAG_TYPE=5" in camera_script or "MAG_TYPE=5" in camera_script
    assert "export PX4_PARAM_SYS_HAS_MAG=0" not in camera_script
    assert "ESTIMATION_MODE=gps" not in camera_script
    assert "export PX4_PARAM_EKF2_MAG_TYPE=0" not in camera_script
    assert "GZ_USE_XVFB" in camera_script
    assert "unset HEADLESS" in camera_script
    assert "E2E_CHECK_CAMERAS" in camera_script
    assert "E2E_CHECK_CAMERAS=0" not in camera_script
    # Soft-GL band loosening from the gps bring-up path must stay gone.
    assert "E2E_YAW_TOLERANCE_DEG=25" not in camera_script
    assert "E2E_LEG_TOLERANCE_M=0.15" not in camera_script
    assert "PX4_GZ_SIM_RENDER_ENGINE" in camera_script
    assert "ESTIMATION_MODE: vision" in text
    assert "EKF2_EV_CTRL: \"15\"" in text
    # Ogre Soft-GL aborts; CI must not force ogre.
    assert "PX4_GZ_SIM_RENDER_ENGINE: ogre" not in text
    installer = Path(
        "includes/gz/params/install_px4_control_params.bash"
    ).read_text(encoding="utf-8")
    assert "EKF2_MAG_TYPE=5" in installer
    assert "EKF2_GPS_CTRL=0" in installer
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
    offboard = Path("tests/e2e/px4_offboard.py").read_text(encoding="utf-8")
    assert "vehicle_visual_odometry" in offboard
    assert "ever_received" in offboard
    assert "cs_ev_pos" in offboard
    assert "vision_tracking_lost" in offboard or "VisionLost" in offboard
    grading = Path("tests/e2e/grading.py").read_text(encoding="utf-8")
    assert "_fmt_seconds" in grading
    assert "microxrce_offboard.py" not in text
    assert "continue-on-error" not in text
    assert "docker load" in text
    assert 'cache_from="type=gha,scope=${cache_scope},version=2"' in text
    assert "mode=max,version=2,timeout=30m" in text
    assert "ACTIONS_RESULTS_URL" in text
