"""Height rows line up on one clock. No ULog and no simulator."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "e2e"))

from height_summary import aligned_rows, fake_z_drift  # noqa: E402


def test_rows_share_one_clock_and_skip_a_far_sample():
    clock = np.array([0, 1_000_000, 2_000_000], dtype=np.int64)
    columns = {
        "ekf_height_m": (clock, np.array([0.0, 1.0, 2.0])),
        "vision_height_m": (np.array([0, 2_000_000]), np.array([0.1, 2.1])),
        "flags": (
            np.array([0, 2_000_000]),
            [{"cs_ev_hgt": False}, {"cs_ev_hgt": True}],
        ),
    }
    rows = aligned_rows(clock, columns, step_us=1_000_000, max_gap_us=100_000)
    assert [row["t_s"] for row in rows] == [0.0, 1.0, 2.0]
    assert rows[0]["ekf_height_m"] == 0.0
    assert rows[0]["vision_height_m"] == 0.1
    assert rows[1]["vision_height_m"] is None
    assert rows[2]["flags"]["cs_ev_hgt"] is True


def test_fake_drift_z_is_zero():
    drift = fake_z_drift()
    assert drift["z_mps"] == 0.0
    assert drift["north_mps"] == 0.0
