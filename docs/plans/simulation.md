# Simulation plan (`feature/sim-headless-groundtruth` @ `2abc7f9`)

Owner: Simulation. Vision is folded in here. Status on Oct 9, 2026: unmerged, replanned to a smaller indoor sim.

## Goal

Indoor flight in `apt_world`. Nav2 closed loop is the product. This branch provides the world, the OAK-D S2 camera, RTAB-Map, `/clock` sim time, and ground truth. `feature/px4-control` owns the controller and consumes `/rtabmap/odom`.

## Already done

- Headless Gazebo (EGL/Xvfb, software GL), real-time factor topic, `/clock` for Nav2, bridge, and state publisher.
- `/ground_truth/odom` (ENU, 50 Hz). Its covariance topic is off `/model/<model>/odometry_with_covariance`, so ground truth stays off EKF2.
- Static `world` → `spawn` transform, and geographic spawn at the Amirkabir Aerospace Department.
- Sensors loaded from the x500 model. Downward lidar on `x500_depth` (`LIDAR_DOWN`).
- `/sim/preflight_check`: stale topics fail, camera checked through `camera_info`, steady-clock discovery, IMU minimum 0.75 of the expected rate.
- On `feature/oakd-s2-vision` @ `8a27573`: OAK-D S2 model, stereo aligned with RTAB-Map.
- On `feature/vision-profiles` @ `d5734d3`: cpu profile (the `full` and `hw` profiles on that branch are dropped below).
- On `refactor/vision-golden-tests` @ `de0a33a` and `refactor/vision-hygiene` @ `d78960e`: golden snapshots and hygiene. One cleanup may keep both.

## What stays

- Worlds: `apt_world` and PX4 `default`.
- Wall map: `includes/gz/walls/apt_world.json` only.
- One camera: OAK-D S2, cpu profile only.
- The Gazebo magnetometer world plugin, so PX4 sees a compass. It is not a line in `sim.params`.
- One `/imu` publisher for the whole stack (the camera IMU, today's default). RTAB-Map subscribes to it.
- Ground truth on `/ground_truth/odom` only. It stays off the PX4 vision topic, so it never reaches EKF2.
- `.env.example` only.
- `spawn.json` from `trajectory_eval record`, with the agreed fields and nothing more: `spawn_xyz`, `spawn_yaw` (ENU, from the live `world` → `spawn` transform), `px4_offset_s` (t_ros − t_px4, lowest gap between `/fmu/out/vehicle_odometry` and `/clock` over at least 50 messages), `px4_offset_spread_s`, `px4_offset_samples`, `px4_offset_reason`. Flight analysis is parked.

## What is cut

This branch adds 191 files and about 275k lines, mostly assets. Those assets come out:

- Worlds `sonoma_raceway`, `husarion_office`, `husarion_world`, and `empty_with_plugins`.
- Mop-cart (`mopcart2`, `mopcart3`) and the other extra meshes (tables, chairs, fridge, sink, toilet, shelves, cabinets, bins). `apt_world` keeps its apartment mesh.
- Generated wall JSON other than `apt_world.json`.
- The classic `.pyc` files this branch added under `includes/gazebo_classic`. The base plan removes `includes/gazebo_classic`; this plan does not carry classic assets.
- The committed `.env`.

From the vision branches:

- `full` and `hw` profiles on `feature/vision-profiles`.
- The nine-module geometry split on `refactor/vision-modules` (`env`, `hardware`, `profiles`, `gpu`, `intrinsics`, `frames`, `sdf`, `urdf`, `x500_patch`). One small cleanup may keep the golden snapshots and the hygiene work. There are no three refactor branches.
- The oak-or-px4 switch (`IMU_SOURCE`, `px4_imu_relay`). That relay is a second IMU bridge and comes out.

Also cut: the mission benchmark, the demo GIF, the long cleanup-PR series, and any revival of `microxrce_offboard.py`.

## Next, in order

1. Delete the extra worlds, meshes, other wall JSON, classic `.pyc` files, and `.env`.
2. Take the OAK-D S2 and the cpu profile from `feature/oakd-s2-vision` and `feature/vision-profiles`.
3. Keep the single camera-IMU publisher on `/imu`. Load the magnetometer world plugin. Confirm ground truth is still off the PX4 vision topic.
4. One cleanup that may keep the golden snapshots from `refactor/vision-golden-tests` and the hygiene fixes from `refactor/vision-hygiene`.
5. `trajectory_eval record` writes `spawn.json` with the agreed fields.
6. After `feature/devops-compose-health-ci` merges, rebase this branch onto it and drop the duplicate start-script fix.

## Dependencies

Merge order: `feature/devops-compose-health-ci`, then `feature/sim-headless-groundtruth`, then `feature/px4-control`.

`feature/px4-control` closes the Nav2 loop on `/rtabmap/odom`. This branch does not own that controller. The base plan removes `includes/gazebo_classic`.

## Done when

- Headless `apt_world` runs with the OAK-D S2 cpu camera, RTAB-Map, `/clock`, one `/imu`, and `/ground_truth/odom` off EKF2. `default` still starts.
- The extra worlds, extra meshes, other wall JSON, and `.env` are gone. `.env.example` remains.
- `spawn.json` is written by `trajectory_eval record` and is not grown further.
- This branch is rebased on `feature/devops-compose-health-ci` and ready for `feature/px4-control`.
