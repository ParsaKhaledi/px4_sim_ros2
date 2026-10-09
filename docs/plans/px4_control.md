# px4_control

Branch: `feature/px4-control` at `8668a11`.

## Goal

Nav2 closed-loop flight in apt_world (RGBD, cpu vision). Nav2 publishes `/cmd_vel`. `px4_control` is the only flight controller.

## What stays

- One controller: `px4_control`. `includes/gz/microxrce_offboard.py` and `includes/gazebo_classic/microxrce_offboard.py` are retired. `tests/e2e/px4_offboard.py` must not become a second controller.
- `/cmd_vel` as `geometry_msgs/TwistStamped` only. Read `msg.twist`. Use the header stamp for the timeout. Nav2 sets `enable_stamped_cmd_vel` true in `controller_server`, `behavior_server`, and `velocity_smoother`. Tests already expect a twist timestamp. No parallel `Twist` subscription.
- `/rtabmap/odom` for vision. One `/imu` source, owned by sim+vision. Do not add `px4_imu_bridge.py`.
- If this node reads vehicle status, subscribe to `/fmu/out/vehicle_status_v1`. This PX4 does not publish the unversioned `vehicle_status` topic.
- Safety, only these three: land on vision loss, lidar-vs-EKF2 height guard, geofence. Geofence is commit `cdbfcf3` on `cursor/geofence-json-walls-83ac` (one commit ahead of `7e7887f`, not in `8668a11`). Bring that commit onto `feature/px4-control`, then delete the cursor branch. No second geofence design.
- EKF2 settings in the single base parameter file owned with DevOps (`feature/devops-compose-health-ci`), applied before EKF2 starts. No post-boot MAVLink param set. Agreed values, and no others: `EKF2_GPS_CTRL` 0, `EKF2_EV_CTRL` 9, `EKF2_HGT_REF` 0 (baro), `EKF2_MAG_TYPE` 5 (compass present, not fused), `EKF2_RNG_CTRL` 1.
- Sim time, kept from the indoor work as a requirement, not a new subsystem: `use_sim_time` true and `UXRCE_DDS_SYNCT` 0. Gazebo `/clock` is the clock.

## What is cut

- Outdoor mode.
- 50 Hz vs 20 Hz benchmark, golden snapshot tests, per-action file split, mission benchmark, demo GIF.
- `flight_analysis` until a real `.ulg` exists. The first flight does not depend on it.
- A second controller, a second `/imu` bridge, a parallel `Twist` subscription, extra EKF2 parameters, and any geofence work other than bringing `cdbfcf3`.

## Already done

- `px4_control` flies: arm, mode, takeoff, land, goto, hold, and offboard setpoints. It consumes `/rtabmap/odom`, drops samples on tracking loss (null pose, covariance 9999, `odom_info`), and runs the lidar-vs-EKF2 height guard. Prearm waits for vision yaw.
- `/cmd_vel` is already subscribed. The type is still `geometry_msgs/Twist`, and the timeout uses the node clock, not the message stamp.
- The five EKF2 values above were set on this branch. They move into the DevOps base file and are not retuned here.

## Ordered next steps

1. Bring `cdbfcf3` onto `feature/px4-control`. Delete `cursor/geofence-json-walls-83ac`.
2. Switch `/cmd_vel` to `TwistStamped`. Read `msg.twist`. Time out on the header stamp. Remove the `Twist` subscription.
3. After the merge order below, one Nav2 flight in apt_world passes.

## Dependencies

Merge order: `feature/devops-compose-health-ci`, then `feature/sim-headless-groundtruth` (vision folded in), then `feature/px4-control`, then one Nav2 flight that passes.

- DevOps owns the base parameter file. The five EKF2 values are applied there before EKF2 starts.
- Sim+vision owns apt_world, RGBD cpu vision, `/rtabmap/odom`, the one `/imu`, and Gazebo `/clock`.
- Nav2 publishes `/cmd_vel` as `TwistStamped`.

## Done when

`px4_control` is the only flight controller, `/cmd_vel` is `TwistStamped` with the stamp used for timeout, geofence is `cdbfcf3` on this branch and the cursor branch is gone, the three safety behaviors are the only ones, and one Nav2 closed-loop flight in apt_world passes after the merge order above.
