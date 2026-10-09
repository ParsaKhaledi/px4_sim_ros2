"""spawn.json has the agreed fields and nothing else."""

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from trajectory_eval.frames import quat_from_rpy
from trajectory_eval.spawn import (
    MIN_PX4_OFFSET_SAMPLES,
    SPAWN_FIELDS,
    build_spawn_document,
    write_spawn_json,
    yaw_from_xyzw,
)


def _xyzw(yaw: float) -> tuple[float, float, float, float]:
    w, x, y, z = quat_from_rpy(0.0, 0.0, yaw)
    return (float(x), float(y), float(z), float(w))


def test_yaw_from_the_spawn_quaternion():
    assert abs(yaw_from_xyzw(*_xyzw(0.0))) < 1e-12
    assert abs(yaw_from_xyzw(*_xyzw(np.pi)) - np.pi) < 1e-9


def test_offset_is_the_lowest_gap_after_fifty_samples():
    gaps = [1.0 + 0.01 * index for index in range(MIN_PX4_OFFSET_SAMPLES)]
    document = build_spawn_document((-3.0, -1.6, 0.0), _xyzw(np.pi), gaps)
    assert tuple(document) == SPAWN_FIELDS
    assert document["spawn_xyz"] == [-3.0, -1.6, 0.0]
    assert abs(document["spawn_yaw"] - np.pi) < 1e-9
    assert document["px4_offset_s"] == 1.0
    assert abs(document["px4_offset_spread_s"] - 0.49) < 1e-9
    assert document["px4_offset_samples"] == MIN_PX4_OFFSET_SAMPLES
    assert "lowest t_ros - t_px4" in document["px4_offset_reason"]


def test_short_recording_leaves_the_offset_empty():
    document = build_spawn_document(None, None, [0.2] * (MIN_PX4_OFFSET_SAMPLES - 1))
    assert document["spawn_xyz"] is None
    assert document["spawn_yaw"] is None
    assert document["px4_offset_s"] is None
    assert document["px4_offset_spread_s"] is None
    assert document["px4_offset_samples"] == MIN_PX4_OFFSET_SAMPLES - 1
    assert "was not available" in document["px4_offset_reason"]
    assert "got 49" in document["px4_offset_reason"]


def test_written_file_has_only_the_agreed_fields(tmp_path):
    document = build_spawn_document((-3.0, -1.6, 0.0), _xyzw(0.0), [0.5] * MIN_PX4_OFFSET_SAMPLES)
    document["extra"] = "no"
    path = tmp_path / "spawn.json"
    write_spawn_json(path, document)
    written = json.loads(path.read_text(encoding="utf-8"))
    assert tuple(written) == SPAWN_FIELDS
    assert "extra" not in written
