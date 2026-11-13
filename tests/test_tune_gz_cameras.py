from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "includes" / "gz"))

import tune_gz_cameras  # noqa: E402


SAMPLE = """
<sensor name="imu" type="imu">
  <update_rate>50</update_rate>
</sensor>
<sensor name="IMX214" type="camera">
  <image><width>640</width><height>480</height></image>
  <update_rate>30</update_rate>
</sensor>
<sensor name="StereoOV7251" type="depth_camera">
  <image><width>640</width><height>480</height></image>
  <update_rate>30</update_rate>
</sensor>
"""


def test_tune_changes_rate_and_leaves_size_and_imu():
    updated = tune_gz_cameras.tune_cameras(SAMPLE, "10")
    assert "<update_rate>50</update_rate>" in updated
    assert updated.count("<update_rate>10</update_rate>") == 2
    assert updated.count("<width>640</width>") == 2
    assert updated.count("<height>480</height>") == 2
    assert "<update_rate>30</update_rate>" not in updated
