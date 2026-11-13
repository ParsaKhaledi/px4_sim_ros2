"""Read and write TUM trajectory files (evo compatible).

Each line is ``timestamp tx ty tz qx qy qz qw`` with timestamp in seconds.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from trajectory_eval.frames import quat_wxyz_to_xyzw, quat_xyzw_to_wxyz
from trajectory_eval.metrics import PoseSample


def write_tum(path: Path, samples: list[PoseSample]) -> None:
    """Write poses in TUM format."""
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for sample in samples:
        qx, qy, qz, qw = quat_wxyz_to_xyzw(sample.quat_wxyz)
        x, y, z = sample.position
        lines.append(
            f"{sample.t:.9f} {x:.9f} {y:.9f} {z:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}"
        )
    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def read_tum(path: Path) -> list[PoseSample]:
    """Load a TUM file. Blank lines and ``#`` comments are ignored."""
    samples: list[PoseSample] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = [float(value) for value in line.split()]
        if len(parts) != 8:
            raise ValueError(f"{path}: expected 8 columns, got {len(parts)}")
        stamp, x, y, z, qx, qy, qz, qw = parts
        samples.append(
            PoseSample(stamp, np.array([x, y, z]), quat_xyzw_to_wxyz(np.array([qx, qy, qz, qw])))
        )
    return samples
