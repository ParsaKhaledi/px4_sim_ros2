# Vision

Owner: Image Processing Researcher. Status on 2026-10-09: folded into the simulation plan. Vision work lives in `docs/plans/simulation.md` on `feature/sim-headless-groundtruth`. This file is the handoff, on `refactor/vision-modules` at `a06ef68`. Nothing merges unless Parsa asks.

## Already done, worth keeping

- One simulated camera, the Luxonis OAK-D S2, on `feature/oakd-s2-vision` at `8a27573`. Gazebo still loads it as `model://OakD-Lite`, because that is the name PX4's `x500_depth` includes. Stereo baseline is 0.075 m (`P[3] = -fx * 0.075`), with exact sync. The baseline relay that publishes `/camera/stereo/right/camera_info_baseline` stays with that camera. RTAB-Map uses `use_sim_time`, `frame_id:=base_link`, and `imu_topic:=/imu`.
- Cpu profile only, from `feature/vision-profiles` at `d5734d3`: `VISION_PROFILE=cpu`, 320x200 at 10 Hz, light RTAB-Map. Measured on that stereo setup: real-time factor 0.43–0.46, odometry 10.0 Hz in sim time. Health floors measured at 320x200: median features 40, inliers 15.
- One `/imu` source for the stack (`IMU_SOURCE=oak`). RTAB-Map odometry is `/rtabmap/odom` for `px4_control` on `feature/px4-control`.
- Golden snapshots (`refactor/vision-golden-tests` at `de0a33a`) and hygiene (`refactor/vision-hygiene` at `d78960e`) fold into one small cleanup on `feature/sim-headless-groundtruth`. `geometry.py` stays one file.

## Cut

- `full` and `hw` profiles (1280x800). They were never measured. GPU is off.
- The nine-module split of `geometry.py` on `refactor/vision-modules`.
- Separate landings of `refactor/vision-golden-tests`, `refactor/vision-hygiene`, and `refactor/vision-modules`.
- Demo GIF, vision observability, and the old follow-on (health log, extra relay, extra READMEs), except a line the cpu camera needs to run.
- A second `/imu` publisher.
- Flight control. `microxrce_offboard.py` is retired.

## What the simulation plan carries

- One camera, the OAK-D S2, cpu profile only, with the measured size, rate, and health floors above.
- One `/imu` source. `/rtabmap/odom` is the odometry `px4_control` consumes.
- `px4_control` on `feature/px4-control` is the only controller. Nav2 is the closed-loop goal, in `apt_world`.
- One small cleanup on `feature/sim-headless-groundtruth` for the golden snapshots and the hygiene pass.
