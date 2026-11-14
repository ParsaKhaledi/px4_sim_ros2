# HealthCheck

Per-service topic/process/TF gates used by compose healthchecks and CI.

| Path | Role |
|------|------|
| [healthcheck.py](healthcheck.py) | Measure rates, write `logs/health/<service>.jsonl`, `--report` table |
| [checks/px4.yaml](checks/px4.yaml) | PX4/Gazebo/XRCE, GT, odometry auto_detect, cameras |
| [checks/state_publisher.yaml](checks/state_publisher.yaml) | `robot_state_publisher` + camera optical TFs |
| [checks/rtabmap.yaml](checks/rtabmap.yaml) | RTAB-Map odom/info + camera topics |

## CameraType

Topics and TF frames use `when_env.CameraType`. Always export `CameraType`
explicitly in CI (`none` / `rgbd` / `stereo`); do not rely on compose defaults.

RGB-D (`CameraType=rgbd`) expects `/camera/rgb/image_raw`,
`/camera/depth/image_raw`, matching `camera_info`, and TF
`camera_rgb_optical_frame` (depth is RGB-aligned).

## Rate floors

- `HEALTH_CAMERA_MIN_HZ` overrides `group: camera` (soft-GL CI uses `1`)
- `HEALTH_SLAM_MIN_HZ` overrides `group: slam` (RTAB-Map odom on cpu)
- `HEALTH_TIMING=1` prints phase timing lines; unset/0 is quiet

## Report

```bash
./scripts/health_report.sh
python3 HealthCheck/healthcheck.py --report logs/health
```

Exits `1` when any latest row is `fail`. Auto-detect candidates share one
table slot with the gate (`pick_live_auto_detect_target`).
