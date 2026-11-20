"""The fake-vision entry point. Limits and EKF2 values stay in the script."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_clean_limits_and_ekf_profile():
    script = (ROOT / "scripts/ci_fake_vision.sh").read_text(encoding="utf-8")
    example = (ROOT / ".env.example").read_text(encoding="utf-8")
    assert "E2E_LEG_TOLERANCE_M=0.03" in example
    assert "E2E_YAW_TOLERANCE_DEG=5" in example
    assert "E2E_RETURN_TOLERANCE_M=0.05" in example
    for line in (
        "EKF2_GPS_CTRL=0",
        "EKF2_MAG_TYPE=5",
        "EKF2_EV_CTRL=15",
        "EKF2_HGT_REF=0",
        "EKF2_EV_DELAY=0",
        "PX4_PARAM_EKF2_GPS_CTRL=0",
        "PX4_PARAM_EKF2_MAG_TYPE=5",
        "PX4_PARAM_EKF2_EV_CTRL=15",
        "PX4_PARAM_EKF2_HGT_REF=0",
        "PX4_PARAM_EKF2_EV_DELAY=0",
        "PX4_PARAM_NAV_DLL_ACT=0",
        "CameraType=none",
        "PX4_GZ_MODEL=x500",
        "FAKE_VISION=1",
        'E2E_FLIGHT_DIR="${ROOT}/logs/flights/fake_vision/${MODE}"',
        'cp "${E2E_FLIGHT_DIR}/trajectory.json" "${E2E_FLIGHT_DIR}/grade.json"',
    ):
        assert line in script
    clean, _, dropout = script.partition('if [ "${MODE}" = "clean" ]; then')
    assert "E2E_LEG_TOLERANCE_M=0.08" in clean
    assert "E2E_RETURN_TOLERANCE_M=0.12" in clean
    assert "E2E_YAW_TOLERANCE_DEG=10" in clean
    assert "E2E_LEG_TOLERANCE_M=0.08" not in dropout
    assert "E2E_RETURN_TOLERANCE_M=0.12" not in dropout
    assert "E2E_YAW_TOLERANCE_DEG=10" not in dropout
    assert "apply_clean_limits" in script
    assert ".github/workflows" not in script
