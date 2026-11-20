"""Height rows line up on one clock. No ULog and no simulator."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests" / "e2e"))

from height_summary import aligned_rows, fake_z_drift, prearm_level  # noqa: E402


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


def test_prearm_level_is_the_ground_before_the_climb():
    # Resting 0.24 m (base_link above an origin), then a 2 m climb.
    times = np.arange(0, 12_000_000, 100_000, dtype=np.int64)
    values = np.full(times.shape, 0.24)
    values[times >= 8_000_000] = 2.24
    assert abs(prearm_level(times, values) - 0.24) < 1e-9


def test_fake_drift_z_is_zero():
    drift = fake_z_drift()
    assert drift["z_mps"] == 0.0
    assert drift["north_mps"] == 0.0
