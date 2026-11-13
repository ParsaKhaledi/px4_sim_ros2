"""The flight test accepts stereo or rgbd."""

import pytest

from px4_control.camera_type import parse_camera_type


def test_environment_camera_type():
    assert parse_camera_type([], {'CameraType': 'stereo'}) == 'stereo'
    assert parse_camera_type([], {'CameraType': 'RGBD'}) == 'rgbd'


def test_flag_overrides_the_environment():
    assert parse_camera_type(['--camera-type', 'rgbd'], {'CameraType': 'stereo'}) == 'rgbd'
    assert parse_camera_type(['--camera-type=stereo'], {}) == 'stereo'


def test_positional_camera_type():
    assert parse_camera_type(['stereo'], {}) == 'stereo'


def test_missing_or_unknown_camera_type_is_rejected():
    with pytest.raises(ValueError, match='stereo or rgbd'):
        parse_camera_type([], {})
    with pytest.raises(ValueError, match='got'):
        parse_camera_type(['mono'], {})
