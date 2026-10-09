"""Wall-map geometry tests."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from wall_geometry import (
    pose_matrix,
    segments_from_box,
    triangle_slice,
)


def test_box_slice_is_a_rectangle():
    # 1 m by 0.2 m footprint, 2 m tall, centered at the origin.
    segments = segments_from_box(np.array([1.0, 0.2, 2.0]), np.eye(4), 0.5)
    assert len(segments) == 4
    lengths = sorted(round(float(np.hypot(b[0] - a[0], b[1] - a[1])), 3) for a, b in segments)
    assert lengths == [0.2, 0.2, 1.0, 1.0]


def test_short_floor_box_is_ignored():
    segments = segments_from_box(np.array([4.0, 4.0, 0.05]), np.eye(4), 0.5)
    assert segments == []


def test_translated_box_moves_the_footprint():
    transform = pose_matrix(np.array([2.0, -1.0, 0.0, 0.0, 0.0, 0.0]))
    segments = segments_from_box(np.array([1.0, 1.0, 2.0]), transform, 0.5)
    points = np.array([point for segment in segments for point in segment])
    assert abs(points[:, 0].mean() - 2.0) < 1e-6
    assert abs(points[:, 1].mean() + 1.0) < 1e-6


def test_triangle_slice():
    segment = triangle_slice(
        np.array([0.0, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 2.0]),
        1.0,
    )
    assert segment is not None
    start, end = segment
    assert start[1] == 0.0 and end[1] == 0.0
