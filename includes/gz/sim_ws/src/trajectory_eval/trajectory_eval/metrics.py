"""Trajectory error metrics.

ATE is reported two ways:

* unaligned, relative to takeoff: each trajectory's positions are shifted by
  its own first sample, then compared. A constant offset from the spawn pose
  does not dominate the score.
* SE(3) aligned, no scale: Umeyama/Kabsch finds one rotation and translation
  that best maps the estimate onto ground truth. Scale is fixed at 1.

RPE is the relative-pose translation error on segments whose ground-truth
length is about 1 m and about 5 m. Drift per metre is that error divided by
the segment length.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from trajectory_eval.frames import quat_normalize, quat_to_rot, wrap_pi, yaw_from_quat


@dataclass
class PoseSample:
    """One pose on the evaluation timeline."""

    t: float
    position: np.ndarray
    quat_wxyz: np.ndarray

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float).reshape(3)
        self.quat_wxyz = quat_normalize(self.quat_wxyz)


def align_se3(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return R, t such that ``R @ source + t`` matches ``target``.

    ``source`` and ``target`` are (N, 3). Scale is not estimated. ``det(R)`` is +1.
    """
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    if source.shape != target.shape or source.ndim != 2 or source.shape[1] != 3:
        raise ValueError("source and target must both be (N, 3)")
    if len(source) < 3:
        raise ValueError("SE(3) alignment needs at least 3 poses")
    src_mean = source.mean(axis=0)
    dst_mean = target.mean(axis=0)
    centered_src = source - src_mean
    centered_dst = target - dst_mean
    covariance = centered_src.T @ centered_dst
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt = vt.copy()
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = dst_mean - rotation @ src_mean
    return rotation, translation


def apply_se3(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return (rotation @ np.asarray(points, dtype=float).T).T + translation


def position_errors(estimate: np.ndarray, reference: np.ndarray) -> np.ndarray:
    return np.linalg.norm(np.asarray(estimate) - np.asarray(reference), axis=1)


def error_stats(errors: np.ndarray) -> dict[str, float]:
    errors = np.asarray(errors, dtype=float)
    if len(errors) == 0:
        return {"rmse": float("nan"), "mean": float("nan"), "max": float("nan"), "count": 0}
    return {
        "rmse": float(np.sqrt(np.mean(errors ** 2))),
        "mean": float(np.mean(errors)),
        "max": float(np.max(errors)),
        "count": int(len(errors)),
    }


def takeoff_relative_positions(samples: list[PoseSample]) -> np.ndarray:
    """Positions with the first sample moved to the origin. Orientation is kept."""
    origin = samples[0].position
    return np.array([sample.position - origin for sample in samples], dtype=float)


def absolute_trajectory_error(estimate: list[PoseSample], reference: list[PoseSample]) -> dict:
    """ATE relative to takeoff and after a rigid SE(3) alignment with no scale."""
    if len(estimate) != len(reference) or len(estimate) == 0:
        raise ValueError("estimate and reference must be the same non-empty length")
    est = takeoff_relative_positions(estimate)
    ref = takeoff_relative_positions(reference)
    unaligned = error_stats(position_errors(est, ref))
    result = {"unaligned_relative_to_takeoff": unaligned, "aligned_se3": None}
    if len(estimate) >= 3:
        rotation, translation = align_se3(
            np.array([s.position for s in estimate]),
            np.array([s.position for s in reference]),
        )
        aligned = apply_se3(np.array([s.position for s in estimate]), rotation, translation)
        reference_xyz = np.array([s.position for s in reference])
        result["aligned_se3"] = error_stats(position_errors(aligned, reference_xyz))
        result["alignment"] = {
            "rotation": rotation.tolist(),
            "translation_m": translation.tolist(),
            "scale": 1.0,
        }
    return result


def _relative_translation(a: PoseSample, b: PoseSample) -> np.ndarray:
    """Translation of pose b expressed in pose a's frame."""
    delta = b.position - a.position
    return quat_to_rot(a.quat_wxyz).T @ delta


def relative_pose_error(estimate: list[PoseSample], reference: list[PoseSample], distance_m: float,
                        tolerance: float = 0.25) -> dict:
    """RPE for ground-truth segments near ``distance_m``.

    ``drift_m_per_m`` is the RMSE of the translation error divided by the
    nominal segment length. ``drift_percent`` is that ratio times 100.
    """
    if len(estimate) != len(reference):
        raise ValueError("estimate and reference length mismatch")
    path = np.zeros(len(reference))
    for i in range(1, len(reference)):
        step = np.linalg.norm(reference[i].position - reference[i - 1].position)
        path[i] = path[i - 1] + step
    errors = []
    lengths = []
    i = 0
    while i < len(reference) - 1:
        target = path[i] + distance_m
        j = int(np.searchsorted(path, target, side="left"))
        if j >= len(reference) or j <= i:
            break
        length = path[j] - path[i]
        if abs(length - distance_m) <= tolerance * distance_m and length > 1e-6:
            delta_est = _relative_translation(estimate[i], estimate[j])
            delta_ref = _relative_translation(reference[i], reference[j])
            errors.append(float(np.linalg.norm(delta_est - delta_ref)))
            lengths.append(float(length))
            i = j
        else:
            i += 1
    stats = error_stats(np.array(errors) if errors else np.array([]))
    if errors:
        drift = stats["rmse"] / distance_m
    else:
        drift = float("nan")
    return {
        "segment_m": distance_m,
        "trans_rmse_m": stats["rmse"],
        "trans_mean_m": stats["mean"],
        "trans_max_m": stats["max"],
        "count": stats["count"],
        "drift_m_per_m": float(drift),
        "drift_percent": float(drift * 100.0) if errors else float("nan"),
        "mean_segment_m": float(np.mean(lengths)) if lengths else float("nan"),
    }


def final_position_error(estimate: list[PoseSample], reference: list[PoseSample]) -> dict:
    """Distance between the last poses, before and after SE(3) alignment."""
    est_rel = estimate[-1].position - estimate[0].position
    ref_rel = reference[-1].position - reference[0].position
    unaligned = float(np.linalg.norm(est_rel - ref_rel))
    aligned = None
    if len(estimate) >= 3:
        rotation, translation = align_se3(
            np.array([s.position for s in estimate]),
            np.array([s.position for s in reference]),
        )
        mapped = apply_se3(estimate[-1].position.reshape(1, 3), rotation, translation)[0]
        aligned = float(np.linalg.norm(mapped - reference[-1].position))
    return {
        "unaligned_relative_to_takeoff_m": unaligned,
        "aligned_se3_m": aligned,
    }


def height_error(estimate: list[PoseSample], reference: list[PoseSample]) -> dict:
    """Vertical error after removing each trajectory's initial height."""
    est_z = np.array([s.position[2] - estimate[0].position[2] for s in estimate])
    ref_z = np.array([s.position[2] - reference[0].position[2] for s in reference])
    return error_stats(np.abs(est_z - ref_z))


def yaw_error(estimate: list[PoseSample], reference: list[PoseSample]) -> dict:
    """Yaw error in radians.

    ``absolute`` is wrap(yaw_est - yaw_gt). ``relative_to_start`` removes each
    trajectory's initial yaw so a constant heading offset is not counted as drift.
    """
    absolute = []
    relative = []
    yaw_est0 = yaw_from_quat(estimate[0].quat_wxyz)
    yaw_ref0 = yaw_from_quat(reference[0].quat_wxyz)
    for est, ref in zip(estimate, reference):
        yaw_est = yaw_from_quat(est.quat_wxyz)
        yaw_ref = yaw_from_quat(ref.quat_wxyz)
        absolute.append(abs(wrap_pi(yaw_est - yaw_ref)))
        relative.append(abs(wrap_pi((yaw_est - yaw_est0) - (yaw_ref - yaw_ref0))))
    return {
        "absolute_rad": error_stats(np.array(absolute)),
        "relative_to_start_rad": error_stats(np.array(relative)),
    }


def gps_noise(gps: list[PoseSample], reference: list[PoseSample]) -> dict:
    """Residual of GPS local-ENU positions against ground truth, both relative to their first sample."""
    est = takeoff_relative_positions(gps)
    ref = takeoff_relative_positions(reference)
    residual = est - ref
    norms = np.linalg.norm(residual, axis=1)
    return {
        "position": error_stats(norms),
        "std_enu_m": {
            "east": float(np.std(residual[:, 0])),
            "north": float(np.std(residual[:, 1])),
            "up": float(np.std(residual[:, 2])),
        },
        "mean_residual_enu_m": {
            "east": float(np.mean(residual[:, 0])),
            "north": float(np.mean(residual[:, 1])),
            "up": float(np.mean(residual[:, 2])),
        },
    }


def slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    """Spherical linear interpolation. Quaternions are (w, x, y, z)."""
    a = quat_normalize(q0)
    b = quat_normalize(q1)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    if dot > 0.9995:
        return quat_normalize(a + u * (b - a))
    theta = np.arccos(np.clip(dot, -1.0, 1.0))
    return (np.sin((1.0 - u) * theta) * a + np.sin(u * theta) * b) / np.sin(theta)


def pair_to_ground_truth(estimate: list[PoseSample], ground_truth: list[PoseSample]) -> tuple[list[PoseSample], list[PoseSample]]:
    """Resample ``estimate`` onto ground-truth stamps that fall inside its span."""
    if len(estimate) < 2 or len(ground_truth) < 2:
        return [], []
    start = min(sample.t for sample in estimate)
    end = max(sample.t for sample in estimate)
    reference = [sample for sample in ground_truth if start <= sample.t <= end]
    if len(reference) < 2:
        return [], []
    times = np.array([sample.t for sample in reference], dtype=float)
    resampled = resample(estimate, times)
    if len(resampled) != len(reference):
        return [], []
    return resampled, reference


def resample(samples: list[PoseSample], times: np.ndarray) -> list[PoseSample]:
    """Interpolate samples onto ``times`` (seconds). Times outside the span are dropped."""
    if len(samples) < 2:
        return []
    stamps = np.array([sample.t for sample in samples], dtype=float)
    order = np.argsort(stamps)
    stamps = stamps[order]
    ordered = [samples[i] for i in order]
    output: list[PoseSample] = []
    for stamp in times:
        if stamp < stamps[0] or stamp > stamps[-1]:
            continue
        index = int(np.searchsorted(stamps, stamp, side="right") - 1)
        index = min(max(index, 0), len(ordered) - 2)
        span = stamps[index + 1] - stamps[index]
        u = 0.0 if span <= 0.0 else float((stamp - stamps[index]) / span)
        position = (1.0 - u) * ordered[index].position + u * ordered[index + 1].position
        quat = slerp(ordered[index].quat_wxyz, ordered[index + 1].quat_wxyz, u)
        output.append(PoseSample(float(stamp), position, quat))
    return output

