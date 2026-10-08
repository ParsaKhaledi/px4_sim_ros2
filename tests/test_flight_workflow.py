from pathlib import Path


def _step(text: str, name: str) -> str:
    marker = f"name: {name}"
    start = text.index(marker)
    rest = text[start + len(marker):]
    nxt = rest.find("\n      - name:")
    if nxt < 0:
        nxt = len(rest)
    return rest[:nxt]


def test_camera_flight_is_non_blocking():
    text = Path(".github/workflows/_build.yml").read_text(encoding="utf-8")
    plain = _step(text, "Flight test (plain x500)")
    camera = _step(text, "Flight test (x500_depth, software rendering)")
    assert "continue-on-error" not in plain
    assert "continue-on-error: true" in camera
    assert "PR #20" in camera
    assert "camera_link" in camera
    assert 'GZ_CAMERA_UPDATE_RATE: "10"' in camera
    assert 'HEALTH_CAMERA_MIN_HZ: "1"' in camera
    assert "VISION_PROFILE: cpu" in camera
    assert "GZ_CAMERA_WIDTH" not in camera
    assert "GZ_CAMERA_HEIGHT" not in camera
    assert 'E2E_ATTEMPT_TIMEOUT: "480"' in plain
    assert "timeout-minutes: 15" in plain


def test_flight_job_loads_the_image_instead_of_building():
    workflow = Path(".github/workflows/_build.yml").read_text(encoding="utf-8")
    ci = Path(".github/workflows/ci.yml").read_text(encoding="utf-8")
    assert "reuse_image: true" in ci
    assert "upload_image: true" in ci
    assert "cancel-in-progress: true" in ci
    assert "if: ${{ !inputs.reuse_image }}" in workflow
    assert "docker load" in workflow
    assert "download-artifact" in workflow
    assert "upload-artifact" in workflow
    assert "timeout-minutes: 10" in ci
    assert "timeout-minutes: 20" in ci
    start = Path("includes/gz/startFiles/gz_start_px4_gz_sim.sh").read_text(encoding="utf-8")
    assert "gz_${MODEL}_${WORLD}" not in start
    assert "PX4_GZ_WORLD" in start
    assert 'make px4_sitl "gz_${MODEL}"' not in start
    assert "make px4_sitl_default" in start
    e2e = Path("scripts/run_e2e.sh").read_text(encoding="utf-8")
    assert 'chmod 777 "${ROOT}/logs" "${ROOT}/logs/flights" "${FLIGHTS}"' in e2e
