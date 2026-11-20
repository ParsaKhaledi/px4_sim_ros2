"""The static frame chain, and the mismatches this check only reports."""

from tree_report import check_tree


def test_chain_has_one_parent_and_optical_frames_match():
    report = check_tree()
    assert report.errors == []
    assert report.ok
    assert report.parents["odom"] == "map"
    assert report.parents["base_link"] == "odom"
    assert report.parents["camera_link"] == "base_link"
    assert report.parents["camera_rgb_optical_frame"] == "camera_rgb_frame"


def test_known_frame_mismatches_are_reported():
    # These are findings, not failures. Dropping the check would hide them.
    # The committed x500_depth overlay has pitch 0; Gazebo gets Mount() at
    # container start, so that difference is not a finding.
    report = check_tree()
    ids = {item["id"] for item in report.findings}
    assert "base_link_height" in ids
    assert "start_at_origin" in ids
    assert "sdf_mount_pitch" not in ids
    assert report.errors == []
