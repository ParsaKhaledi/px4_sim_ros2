import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "HealthCheck"))

import healthcheck  # noqa: E402


def test_auto_detect_prefers_publisher():
    chosen = healthcheck.choose_topic(
        ["/fmu/out/vehicle_status", "/fmu/out/vehicle_status_v1"],
        {"/fmu/out/vehicle_status", "/fmu/out/vehicle_status_v1"},
        {"/fmu/out/vehicle_status": 0, "/fmu/out/vehicle_status_v1": 1},
    )
    assert chosen == "/fmu/out/vehicle_status_v1"


def test_auto_detect_falls_back_to_existing_topic():
    chosen = healthcheck.choose_topic(
        ["/fmu/out/vehicle_status", "/fmu/out/vehicle_status_v1"],
        {"/fmu/out/vehicle_status"},
        {},
    )
    assert chosen == "/fmu/out/vehicle_status"


def test_auto_detect_prefers_version_when_neither_has_a_publisher():
    chosen = healthcheck.choose_topic(
        ["/fmu/out/vehicle_status", "/fmu/out/vehicle_status_v1"],
        {"/fmu/out/vehicle_status", "/fmu/out/vehicle_status_v1"},
        {"/fmu/out/vehicle_status": 0, "/fmu/out/vehicle_status_v1": 0},
    )
    assert chosen == "/fmu/out/vehicle_status_v1"


def test_software_camera_rate_override():
    spec = {"group": "camera", "min_rate_hz": 5}
    assert healthcheck.effective_min_rate(spec, {}) == 5
    assert healthcheck.effective_min_rate(spec, {"HEALTH_CAMERA_MIN_HZ": "1"}) == 1
    core = {"group": "core", "min_rate_hz": 20}
    assert healthcheck.effective_min_rate(core, {"HEALTH_CAMERA_MIN_HZ": "1"}) == 20


def test_absent_optional_topic_skips():
    status, rate, reason = healthcheck.judge_rate(0, 0, 2.0, 25, True, False)
    assert status == "skip"
    assert rate == 0
    assert "absent" in reason


def test_ground_truth_floor_tracks_the_sim_clock():
    px4 = healthcheck.load_yaml(
        Path(__file__).resolve().parents[1] / "HealthCheck" / "checks" / "px4.yaml"
    )
    spec = next(item for item in px4["topics"] if item.get("topic") == "/ground_truth/odom")
    assert spec["model_rate_hz"] == 50
    assert spec["step_rate_hz"] == 250
    assert spec["rate_margin"] == 0.5

    def floor(clock_hz):
        return healthcheck.effective_min_rate(spec, {}, clock_hz)

    # apt_world on line-sim: /clock 163.5 Hz, ground truth 33.5 Hz.
    slow = floor(163.5)
    assert slow == 163.5 * (50 / 250) * 0.5
    slow_ok, slow_rate, _reason = healthcheck.judge_rate(1, 67, 2.0, slow, False, False)
    assert slow_ok == "ok"
    assert slow_rate == 33.5
    # default world on line-control: /clock 224 Hz, ground truth 44 Hz.
    fast = floor(224.0)
    fast_ok, fast_rate, _reason = healthcheck.judge_rate(1, 88, 2.0, fast, False, False)
    assert fast_ok == "ok"
    assert fast_rate == 44
    # Dead, and far below the same run's clock, still fail.
    silent, silent_rate, _reason = healthcheck.judge_rate(1, 0, 2.0, slow, True, False)
    assert silent == "fail"
    assert silent_rate == 0
    stalled, stalled_rate, _reason = healthcheck.judge_rate(1, 10, 2.0, slow, False, False)
    assert stalled == "fail"
    assert stalled_rate == 5
    # No clock sample: the floor stays positive so 0 Hz fails.
    missing = floor(0)
    assert missing > 0
    missing_fail, _, _reason = healthcheck.judge_rate(1, 0, 2.0, missing, True, False)
    assert missing_fail == "fail"


def test_slow_topic_fails():
    status, rate, _reason = healthcheck.judge_rate(1, 4, 2.0, 20, False, False)
    assert status == "fail"
    assert rate == 2


def test_camera_env_filter():
    spec = {"when_env": {"CameraType": "stereo"}}
    assert not healthcheck.topic_matches_env(spec, {"CameraType": "rgbd"})
    assert healthcheck.topic_matches_env(spec, {"CameraType": "stereo"})
    assert healthcheck.topic_matches_env(spec, {}) is False


def test_jsonl_roundtrip(tmp_path):
    path = tmp_path / "health" / "px4.jsonl"
    healthcheck.append_jsonl(path, {"service": "PX4", "status": "ok", "rate_hz": 50})
    row = json.loads(path.read_text(encoding="utf-8").strip())
    assert row["status"] == "ok"


def test_check_files_cover_services():
    checks = Path(__file__).resolve().parents[1] / "HealthCheck" / "checks"
    for name in healthcheck.SERVICE_FILES.values():
        data = healthcheck.load_yaml(checks / name)
        assert isinstance(data, dict)
    px4 = healthcheck.load_yaml(checks / "px4.yaml")
    topics = []
    for spec in px4["topics"]:
        topics.append(spec.get("topic") or ",".join(spec.get("auto_detect", [])))
    assert "/clock" in topics
    assert "/ground_truth/odom" in topics
    assert any("vehicle_status_v1" in topic for topic in topics)
    assert any("vehicle_local_position_v1" in topic for topic in topics)
    status = next(
        spec for spec in px4["topics"]
        if any(name.endswith("vehicle_status_v1") for name in spec.get("auto_detect", []))
    )
    assert status["auto_detect"][0].endswith("_v1")
