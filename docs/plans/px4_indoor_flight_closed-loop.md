# PX4 indoor flight closed loop — superseded

This plan is superseded by `docs/plans/px4_control.md` on branch `feature/px4-control`.

`px4_control` is the only controller. `microxrce_offboard.py` is retired, including the uncommitted hold-pose changes. This branch plans no further work on that node.

Nav2 closed loop in `apt_world` stays the goal. Camera profile is `cpu` only. Worlds are `apt_world` and `default` only.

## What moved where

| Former work on this branch | Owner |
| --- | --- |
| Offboard control and the mission flight | `feature/px4-control` (`docs/plans/px4_control.md`). The mission compose profile launches `px4_control`. |
| One `/imu` source | Sim and vision. This branch does not add `px4_imu_bridge`. |
| Nav2 contract below | `feature/px4-control` and the sim plan on `feature/sim-headless-groundtruth` (`docs/plans/simulation.md`) |
| Magnetometer world plugin and `NAV_DLL_ACT` 0 | Base/sim parameter and world setup on `feature/devops-compose-health-ci` and `feature/sim-headless-groundtruth` |
| Vehicle-status topic name | `feature/px4-control`, if it reads vehicle status |

Dropped from this plan: `gazebo_classic`, m-explore, CycloneDDS activation, GPU Dockerfile work, and CI smoke growth.

## Nav2 contract

Facts for `feature/px4-control` and `feature/sim-headless-groundtruth`. They are notes for those plans, not tasks on `feature/px4-imu-bridge-indoor-flight`.

- `enable_stamped_cmd_vel: true` in `controller_server`, `behavior_server`, and `velocity_smoother`. Jazzy defaults it to false.
- Costmaps on `/rtabmap/map`.
- `use_sim_time: true`.
- `UXRCE_DDS_SYNCT` 0.
- The mission compose profile launches `px4_control`, not `microxrce_offboard.py`.

## Uncommitted findings

Notes for the owners above. Not tasks on this branch.

- The magnetometer world plugin and `NAV_DLL_ACT` 0 belong in the base/sim parameter and world setup on `feature/devops-compose-health-ci` and `feature/sim-headless-groundtruth`.
- This PX4 publishes vehicle status as `vehicle_status_v1`. `px4_control` subscribes to that name if it reads vehicle status.

## Stop

No further commits on `feature/px4-imu-bridge-indoor-flight` except this plan note.
