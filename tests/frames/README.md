# Frame tests

`pytest tests/frames` does not start Gazebo. It checks the vision-chain frames
and the fake RTAB-Map pose math.

| File | What it covers |
|------|----------------|
| `test_conversions.py` | Real `frames.py` / `VisionOdometryBridge` conversions: ENU to NED, FLU to FRD, ENU yaw 90 deg to NED yaw 0, quaternion `xyzw` vs `wxyz`, round trips, tilted attitudes, covariance `R Σ Rᵀ`, camera-mount round trip |
| `test_timestamps.py` | `timestamp_sample` is the header plus the uXRCE-DDS offset. `timestamp - timestamp_sample` is the publish delay. A raw copy of the header fails. Negative and above 100 ms fail |
| `test_tree.py` | `map → odom → base_link → camera_link → optical`, one parent, optical rpy, sensor xyz, image and camera_info frame ids. Findings (not failures): the 0.24 m base_link comment, the committed SDF mount pitch of 0, and `RGBD/StartAtOrigin` |
| `test_fake_rtabmap.py` | Origin at the spawn with world-ENU yaw, delayed publish with the capture stamp kept, 12.5 Hz, noise, drift, camera child frame, dropout, late stamp, pose jump, lost tracking |
| `test_dropout_grade.py` | Brake / hold / land grading, and the yaw passing −90 deg ENU |
| `test_script.py` | `scripts/ci_fake_vision.sh` EKF2 values and the clean-only 8 cm / 12 cm / 10 deg limits |

The two SITL runs are `scripts/ci_fake_vision.sh clean` and
`scripts/ci_fake_vision.sh dropout`. They write
`logs/flights/fake_vision/<mode>/grade.json`.
