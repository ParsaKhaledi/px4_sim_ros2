import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "e2e"))

from report import build_report, crash_reset_count, needs_render_fallback, write_stub  # noqa: E402
from test_out_and_back import driver_name  # noqa: E402


def test_driver_falls_back_when_the_package_is_missing():
    assert driver_name() == "px4_offboard"


def test_report_lists_each_crash_and_counts_resets():
    attempts = [
        {"attempt": 1, "status": "crashed", "crash_reason": "tilt", "passed": False},
        {"attempt": 2, "status": "passed", "passed": True, "checks": [{"name": "height", "passed": True}]},
    ]
    report = build_report(attempts)
    assert report["passed"]
    assert report["status"] == "passed"
    assert report["crash_resets"] == 1
    assert report["crash_reasons"] == [{"attempt": 1, "crash_reason": "tilt"}]
    assert crash_reset_count(attempts) == 1


def test_every_attempt_crashed():
    attempts = [
        {"attempt": 1, "status": "crashed", "crash_reason": "tilt"},
        {"attempt": 2, "status": "crashed", "crash_reason": "low_altitude"},
        {"attempt": 3, "status": "crashed", "crash_reason": "odometry_timeout"},
    ]
    report = build_report(attempts)
    assert not report["passed"]
    assert report["crash_resets"] == 2
    assert [row["crash_reason"] for row in report["crash_reasons"]] == [
        "tilt",
        "low_altitude",
        "odometry_timeout",
    ]


def test_blank_frames_ask_for_the_xvfb_fallback():
    attempts = [
        {
            "attempt": 1,
            "status": "failed",
            "passed": False,
            "camera_reason": "blank_camera",
            "gz_rtf": {"mean": 0.4, "min": 0.3, "max": 0.5, "samples": 4},
        }
    ]
    report = build_report(attempts)
    assert report["camera_reason"] == "blank_camera"
    assert report["gz_rtf"]["mean"] == 0.4
    assert needs_render_fallback(attempts)


def test_a_crash_does_not_switch_renderers():
    attempts = [{"attempt": 1, "status": "crashed", "crash_reason": "tilt", "camera_reason": "no_camera_frames"}]
    assert not needs_render_fallback(attempts)


def test_stub_for_a_killed_attempt(tmp_path):
    path = tmp_path / "attempt-1.json"
    write_stub(path, 137, 1)
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["status"] == "crashed"
    assert payload["crash_reason"] == "attempt exited 137"
