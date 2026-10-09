"""Metric tests on synthetic trajectories."""

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from trajectory_eval.frames import quat_from_rpy
from trajectory_eval.metrics import (
    PoseSample,
    absolute_trajectory_error,
    align_se3,
    apply_se3,
    final_position_error,
    gps_noise,
    height_error,
    pair_to_ground_truth,
    relative_pose_error,
    yaw_error,
)
from trajectory_eval.report import write_report
from trajectory_eval.tum import read_tum, write_tum


def _line(xs, y_fn, z_fn, yaw_fn=None):
    samples = []
    for index, x in enumerate(xs):
        yaw = 0.0 if yaw_fn is None else yaw_fn(x)
        samples.append(PoseSample(
            float(index),
            np.array([x, y_fn(x), z_fn(x)]),
            quat_from_rpy(0.0, 0.0, yaw),
        ))
    return samples


def test_align_se3_recovers_rigid_transform_without_scale():
    source = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.2],
        [2.0, 0.5, -0.3],
    ])
    yaw = np.deg2rad(90.0)
    rotation = np.array([
        [np.cos(yaw), -np.sin(yaw), 0.0],
        [np.sin(yaw), np.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ])
    translation = np.array([5.0, -2.0, 1.5])
    target = apply_se3(source, rotation, translation)
    found_r, found_t = align_se3(source, target)
    np.testing.assert_allclose(found_r, rotation, atol=1e-8)
    np.testing.assert_allclose(found_t, translation, atol=1e-8)
    assert abs(np.linalg.det(found_r) - 1.0) < 1e-8


def test_constant_offset_is_removed_by_takeoff_frame_and_by_alignment():
    xs = np.linspace(0.0, 8.0, 17)
    ground_truth = _line(xs, lambda x: 0.0, lambda x: 1.0)
    estimate = _line(xs, lambda x: 3.0, lambda x: 1.0)
    ate = absolute_trajectory_error(estimate, ground_truth)
    assert ate["unaligned_relative_to_takeoff"]["rmse"] < 1e-9
    assert ate["aligned_se3"]["rmse"] < 1e-8


def test_lateral_drift_rpe_and_final_error():
    xs = np.linspace(0.0, 10.0, 21)
    ground_truth = _line(xs, lambda x: 0.0, lambda x: 0.0)
    # 0.1 m of cross-track error per metre travelled.
    estimate = _line(xs, lambda x: 0.1 * x, lambda x: 0.0)
    ate = absolute_trajectory_error(estimate, ground_truth)
    assert ate["unaligned_relative_to_takeoff"]["rmse"] > 0.4
    rpe = relative_pose_error(estimate, ground_truth, 1.0)
    assert rpe["count"] >= 5
    assert abs(rpe["drift_m_per_m"] - 0.1) < 0.02
    assert abs(rpe["drift_percent"] - 10.0) < 2.0
    rpe5 = relative_pose_error(estimate, ground_truth, 5.0)
    assert abs(rpe5["trans_rmse_m"] - 0.5) < 0.05
    final = final_position_error(estimate, ground_truth)
    assert abs(final["unaligned_relative_to_takeoff_m"] - 1.0) < 1e-6
    climbing = _line(xs, lambda x: 0.0, lambda x: 0.05 * x)
    height = height_error(climbing, ground_truth)
    assert abs(height["max"] - 0.5) < 1e-9
    yaw = yaw_error(estimate, ground_truth)
    assert yaw["relative_to_start_rad"]["rmse"] < 1e-9


def test_yaw_error_detects_a_growing_heading():
    xs = np.linspace(0.0, 4.0, 9)
    ground_truth = _line(xs, lambda x: 0.0, lambda x: 0.0)
    estimate = _line(xs, lambda x: 0.0, lambda x: 0.0, yaw_fn=lambda x: 0.1 * x)
    yaw = yaw_error(estimate, ground_truth)
    assert yaw["relative_to_start_rad"]["max"] == 0.4 or abs(yaw["relative_to_start_rad"]["max"] - 0.4) < 1e-9


def test_gps_noise_on_a_biased_cloud():
    xs = np.linspace(0, np.pi, 9)
    ground_truth = _line(xs, lambda x: 0.0, lambda x: 0.0)
    gps = _line(xs, lambda x: 0.2 * np.sin(x), lambda x: 0.0)
    noise = gps_noise(gps, ground_truth)
    assert noise["position"]["rmse"] > 0.1
    assert abs(noise["mean_residual_enu_m"]["north"]) > 0.05
    assert abs(noise["std_enu_m"]["up"]) < 1e-9


def test_resample_pairs_different_rates(tmp_path):
    ground_truth = _line(np.linspace(0, 4, 9), lambda x: 0.0, lambda x: 0.0)
    for sample, x in zip(ground_truth, np.linspace(0, 4, 9)):
        sample.t = float(x)
    sparse = _line([0.0, 4.0], lambda x: x / 4.0, lambda x: 0.0)
    sparse[0].t = 0.0
    sparse[1].t = 4.0
    paired, reference = pair_to_ground_truth(sparse, ground_truth)
    assert len(paired) == len(reference) == len(ground_truth)
    assert abs(paired[len(paired) // 2].position[1] - 0.5) < 1e-9
    write_tum(tmp_path / "ground_truth.tum", ground_truth)
    loaded = read_tum(tmp_path / "ground_truth.tum")
    np.testing.assert_allclose(loaded[3].position, ground_truth[3].position)
    document = write_report(tmp_path / "out", {"ground_truth": ground_truth, "rtabmap": sparse})
    assert document["trajectories"]["rtabmap"]["available"]
    assert (tmp_path / "out" / "metrics.json").is_file()
    assert (tmp_path / "out" / "metrics.csv").is_file()
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        saved = json.loads((tmp_path / "out" / "metrics.json").read_text(encoding="utf-8"))
        assert saved["plots_skipped"] == "matplotlib is not installed"
        assert document["plots_skipped"] == "matplotlib is not installed"
    else:
        assert (tmp_path / "out" / "trajectories_xy.png").is_file()
        assert (tmp_path / "out" / "ate_unaligned.png").is_file()
