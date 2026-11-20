# Frame tests

`pytest tests/frames` does not start Gazebo. It checks the vision-chain frames,
the fake RTAB-Map pose math, and the conversion tests that live next to the
bridge (`px4_control/test/test_frames.py` and `test_vision_and_modes.py`).

| File | What it covers |
|------|----------------|
| `test_package_frames.py` | Collects the package conversion tests: ENU/NED, FLU/FRD, yaw, quaternion order, tilted attitudes, covariance `R Σ Rᵀ`, and rejection of an odometry frame that is not `odom` → `base_link` |
| `test_timestamps.py` | `timestamp_sample` is the header plus the uXRCE-DDS offset. `timestamp - timestamp_sample` is the publish delay. A raw copy of the header fails. Negative and above 100 ms fail |
| `test_tree.py` | `map → odom → base_link → camera_link → optical`, one parent, optical rpy, sensor xyz, image and camera_info frame ids. The camera joint is compared to `geometry.Mount()` (the pose `render_oakd.py --patch-x500` writes), not the unpatched `x500_depth` overlay. Findings (not failures): the 0.24 m base_link comment and `RGBD/StartAtOrigin` |
| `test_fake_rtabmap.py` | Origin at the spawn with world-ENU yaw, `odom` → `base_link` only, delayed publish with the capture stamp kept, 12.5 Hz, noise, east-only drift, dropout, late stamp, pose jump, lost tracking |
| `test_vision_qos.py` | The control node subscribes to `/rtabmap/odom` best-effort, matching RTAB-Map `qos:=2` |
| `test_dropout_grade.py` | Brake / hold / land grading, and the yaw passing −90 deg ENU |
| `test_script.py` | `scripts/ci_fake_vision.sh` EKF2 values and the clean-only 8 cm / 12 cm / 10 deg limits |

The two SITL runs are `scripts/ci_fake_vision.sh clean` and
`scripts/ci_fake_vision.sh dropout`. They write
`logs/flights/fake_vision/<mode>/grade.json` and `summary.json`.
