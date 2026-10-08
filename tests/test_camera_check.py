import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "e2e"))

from camera_check import CameraTracker, is_blank, pixel_stats, rtf_summary  # noqa: E402


def test_a_flat_frame_is_blank():
    stats = pixel_stats(bytes(200), "mono8")
    assert stats["variance"] == 0.0
    assert is_blank(stats, 1.0)


def test_a_varied_frame_is_not_blank():
    data = bytes([index % 256 for index in range(400)])
    stats = pixel_stats(data, "rgb8")
    assert stats["count"] > 0
    assert not is_blank(stats, 1.0)


def test_depth_samples_are_little_endian():
    data = (10).to_bytes(2, "little") + (40000).to_bytes(2, "little")
    stats = pixel_stats(data, "16UC1")
    assert stats["count"] == 2
    assert stats["mean"] > 1000


def test_tracker_fails_when_a_topic_never_arrives():
    tracker = CameraTracker(min_variance=1.0)
    report = tracker.report(now_s=10.0)
    assert not report["passed"]
    assert report["reason"] == "no_camera_frames"


def test_tracker_fails_a_black_image_and_keeps_a_real_one():
    tracker = CameraTracker(min_variance=1.0)
    black = bytes(640 * 480)
    varied = bytes([(index * 17) % 251 for index in range(640 * 480)])
    tracker.add("/camera/rgb/image_raw", "mono8", 640, 480, black, 1.0)
    tracker.add("/camera/depth/image_raw", "mono8", 640, 480, varied, 1.0)
    tracker.add("/camera/depth/image_raw", "mono8", 640, 480, varied, 2.0)
    report = tracker.report(now_s=2.0)
    assert report["reason"] == "blank_camera"
    assert report["topics"]["/camera/rgb/image_raw"]["blank"]
    depth = report["topics"]["/camera/depth/image_raw"]
    assert not depth["blank"]
    assert depth["rate_hz"] == 2.0


def test_rtf_summary_records_the_spread():
    summary = rtf_summary([0.3, 0.6, 0.45])
    assert summary == {"mean": 0.45, "min": 0.3, "max": 0.6, "samples": 3}
    assert rtf_summary([]) is None
