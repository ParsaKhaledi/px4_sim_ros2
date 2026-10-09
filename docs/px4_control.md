# PX4 control

ROS 2 offboard control for PX4 v1.17.0 over uXRCE-DDS. One node owns the PX4 topics. Mission code talks to it through actions, the same way a MAVROS script would.

The old script `includes/gz/microxrce_offboard.py` is gone. `includes/gz/startFiles/start_mission.sh` starts this node instead.

## Packages

| Package | Type | Role |
|---------|------|------|
| `px4_control_interfaces` | ament_cmake | `Arm`, `SetMode`, `Takeoff`, `Land`, `GoTo`, `Hold`, `VehicleState` |
| `px4_control` | ament_python | node, frame math, trajectories, vision bridge, `Drone` client |

Build them on top of `px4_msgs` **v1.17.0**. The image workspace is `/home/px4/ws_px4`. An optional overlay is `/home/px4/ws_control`.

```bash
source /opt/ros/jazzy/setup.bash
source /home/px4/ws_px4/install/setup.bash
colcon build --base-paths ros2_ws/src
```

Unit tests do not need Gazebo:

```bash
cd ros2_ws/src/px4_control && PYTHONPATH=. python3 -m pytest -q test
```

## What the node does

`px4_control` publishes `OffboardControlMode` and `TrajectorySetpoint` together at 20 Hz. A mode switch to offboard waits until that stream has been alive at 2 Hz or more for 1 second. Duplicate timestamps do not count.

After takeoff the vehicle holds position. It then accepts a `GoTo` goal or `/cmd_vel`. If `/cmd_vel` is quiet for 0.5 s the vehicle brakes to a stop and latches position. It does not snap to a hold while it is still moving. `/cmd_vel` is ignored during takeoff and land.

`GoTo` and `move_forward` stream a trapezoidal position, velocity, and acceleration. They do not step the position setpoint, and the acceleration feedforward stays inside `max_accel`. `turn` ramps the signed yaw change with XY locked, so a +180 deg turn goes the long way counter-clockwise from above instead of picking either short path. The default yaw rate is 30 deg/s (`max_yaw_rate_deg_s`, env `PX4_MAX_YAW_RATE_DEG_S`). An action succeeds only after the vehicle has stayed inside the settle tolerance (0.02 m, 3 deg) for 1.0 s. A cancel or a timeout brakes from the current velocity instead of freezing the setpoint.

`land` descends to half a metre below the altitude recorded at takeoff, and it does not succeed when the setpoint merely reaches the ground. Success waits for `vehicle_land_detected.landed`. A position hold on the surface keeps hover thrust on, and PX4 then answers disarm with `TEMPORARILY_REJECTED` ("not landed"). That ack is retried until the detector latches or the disarm budget runs out.

Timers, settle windows, action timeouts, and the vision staleness check use the node clock. With `use_sim_time` that clock is `/clock`, so a real-time factor of 0.004 or 0.5 does not change the mission. The `Drone` client uses the same clock. `out_and_back` sets `use_sim_time` from `USE_SIM_TIME` (default true) and needs `/clock`. The 20 Hz setpoint timer is a ROS timer, so it is also sim time.

## Topics, services, actions

PX4 QoS is best-effort, volatile, keep-last 5. `/cmd_vel` and the vision odometry use the default reliable QoS.

Topic names are in `ros2_ws/src/px4_control/config/topics.yaml`, checked against px4_msgs v1.17.0. `VehicleStatus` and `VehicleLocalPosition` are `MESSAGE_VERSION` 1, so the live topics are `/fmu/out/vehicle_status_v1` and `/fmu/out/vehicle_local_position_v1`. `VehicleOdometry`, `VehicleAttitude`, `VehicleCommandAck`, `VehicleLandDetected`, `TrajectorySetpoint`, and `VehicleCommand` are version 0 and keep the bare name. `FailsafeFlags` and `OffboardControlMode` have no `MESSAGE_VERSION`. Subscribers and publishers on `/fmu` use best-effort QoS. The node also subscribes to the unversioned base name when the configured name has a `_vN` suffix, and logs which one delivers data. The pose source is `vehicle_odometry`, not `vehicle_local_position`.

| Direction | Name |
|-----------|------|
| publish | `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command` |
| publish (vision mode) | `/fmu/in/vehicle_visual_odometry` |
| subscribe | `/fmu/out/vehicle_status_v1` (and the base name), `/fmu/out/vehicle_odometry`, `/fmu/out/failsafe_flags`, `/fmu/out/vehicle_land_detected`, `/fmu/out/vehicle_command_ack` |
| subscribe | `/cmd_vel`, `/rtabmap/odom` (both configurable) |
| publish | `/px4_control/state` (`VehicleState`), `/px4_control/odom` (`nav_msgs/Odometry`, frame `odom`, child `base_link`, twist body FLU) |
| service | `/px4_control/arm`, `/px4_control/set_mode` |
| action | `/px4_control/takeoff`, `/px4_control/land`, `/px4_control/goto`, `/px4_control/hold` |

`use_sim_time` defaults to true. The node clock then needs `/clock`, which `includes/gz/config_gz_bridge.yaml` already bridges. Pass `use_sim_time:=false` only when no simulator clock is running. Once a PX4 timestamp has been seen, later stamps stay in the PX4 time domain by adding the node-clock delta.

## Drone API

```python
from px4_control.drone import Drone

drone = Drone(node)
drone.takeoff(2.0)
drone.hold(10.0)
drone.move_forward(0.3)
drone.turn(180.0)          # counter-clockwise from above
drone.move_forward(0.3)
drone.land()
```

The same mission is `ros2 run px4_control out_and_back`. It takes `CameraType` (`stereo` or `rgbd`), from `--camera-type` or the environment, the same name `scripts/up.sh` passes to Gazebo and RTAB-Map:

```bash
CameraType=rgbd ros2 run px4_control out_and_back
CameraType=stereo ros2 run px4_control out_and_back
```

A downward `/fmu/out/distance_sensor` is checked against EKF2 height above the floor (local `z` relative to the last landed `z`, not `dist_bottom`). The range is multiplied by the body-down cosine, then `lidar_mount_offset_m` (default 0.19 m) is subtracted. That offset is the sensor height above the floor at rest: base_link is 0.24 m above the model origin and the sensor is 5 cm below base_link. The check stays disarmed until the lidar reads above 0.5 m inside `min_distance`/`max_distance` with `signal_quality` not 0. Out-of-range samples are ignored. Once armed, a gap above `lidar_height_tolerance_m` (default 0.3 m) for more than `lidar_height_duration_s` (default 0.5 s) fails the active goal, brakes to a stop, and lands. The log line starts with `lidar height abort`. With no `distance_sensor` the guard does nothing and logs `no distance_sensor; lidar height guard disabled` once. This branch does not set `EKF2_EV_POS_Z`; the camera frame offset is fixed in the URDF.

`turn` is positive counter-clockwise in ROS, which is a negative NED yaw rate. `turn(180)` reverses heading by turning counter-clockwise, not by taking whichever short path `wrap_pi` would pick. `move_forward` is a body-frame `GoTo` along the current heading. `goto(x, y, z, yaw=None, frame='enu'|'body')` is the general call. In the ENU frame, `yaw` is an absolute ENU heading in radians and is converted to NED (`pi/2 - yaw`). In the body frame it is a relative counter-clockwise angle.

`hold(seconds)` blocks until the vehicle has settled, then for `seconds` on the control node's clock. With `use_sim_time` that is simulated seconds, not wall seconds.

## Frames

Vision odometry is ENU, body FLU, the usual ROS / RTAB-Map convention. PX4 external vision is NED, body FRD.

- Position `(E, N, U)` becomes `(N, E, -U)`.
- `R_NED_FROM_ENU = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]`.
- `R_FRD_FROM_FLU = diag(1, -1, -1)`.
- Attitude: `R_ned_frd = R_NED_FROM_ENU @ R_enu_flu @ R_FRD_FROM_FLU`.
- Covariance is rotated as `R Σ Rᵀ`. It is not multiplied element-wise by `diag(1, -1, -1)`.
- `/cmd_vel` is rotated by yaw only. Nav2 is level. `vy' = -vy`, then `vN = vx cosψ − vy' sinψ`, `vE = vx sinψ + vy' cosψ`, `yawrate = −ωz`. `ψ` is NED yaw, 0 at North, positive toward East.

An identity ENU quaternion faces East (NED yaw +90°). Facing north in ENU is yaw +90° (`xyzw` about 0, 0, 0.707, 0.707), which is the identity heading in NED.

`VehicleOdometry.q` is `wxyz`. `pose_frame` is `POSE_FRAME_NED`. `velocity_frame` is `VELOCITY_FRAME_BODY_FRD`. `quality` is 100 because 0 means invalid and `EKF2_EV_QMIN` defaults to 0.

## Vision bridge

Subscribes to `/rtabmap/odom` (parameter `vision_odom_topic`). Publishes `/fmu/in/vehicle_visual_odometry` only while the sample is valid, and only in `vision` mode.

A frame counts as lost tracking only from RTAB-Map's own signals:

- a null pose on `/rtabmap/odom` (the published null transform, not a real pose at the origin)
- pose covariance 9999, which is what `publish_null_when_lost` sends (the default)
- `rtabmap_msgs/OdomInfo.lost` when that package is installed

A null pose or a 9999 covariance is never forwarded to `/fmu/in/vehicle_visual_odometry`. Negative, non-finite, or huge variance (`vision_max_variance`, default 25) is also dropped, and a 9999 covariance is dropped even when `vision_max_variance` is raised above it.

`vision_timeout_s` (default 0.3 s) is only a stalled-publisher check. The node clock is sim time when `use_sim_time` is set, and the check is that clock minus the odometry header stamp. Past the limit the stream is stale and the vehicle holds. A slow but still valid frame is not lost tracking, and it does not increment `reset_counter`.

The bridge keeps the odometry header stamp and copies it, in microseconds, onto `VehicleOdometry.timestamp` and `timestamp_sample`. It does not re-stamp the last pose and it does not read the wall clock. `reset_counter` increments only when `/rtabmap/odom_info` reports a new map or a reset: a sample that is not lost and whose covariance is 9999, which is how RTAB-Map asks the mapper to start a new map after `Odom/ResetCountdown`. A pose jump does not bump the counter.

While vision is lost or the stream is stale the setpoint brakes to a stop, latches position, and takes yaw rate to zero. It does not yaw in place. `/cmd_vel` is ignored until the next valid sample. Takeoff and land keep climbing or descending and only freeze the yaw command. A `GoTo` or `Hold` that was running is aborted with `vision tracking lost`. After recovery the vehicle stays on that hold until a new command. The EKF re-anchors only when the reset counter was bumped by a new map.

`rtabmap_msgs` is optional. Without it, loss is detected from the null pose and the 9999 covariance on the odometry stream itself, and `reset_counter` stays put because the new-map signal is on `odom_info`.

## Estimation mode

ROS parameter `estimation_mode`. Environment key `ESTIMATION_MODE=vision|gps`. Default is `vision`.

`includes/gz/gz_modifications.bash` calls `includes/gz/params/install_px4_control_params.bash`, which rewrites `PX4-Autopilot/px4_control_params.env`. `gz_start_px4_gz_sim.sh` sources that file. Stock PX4 v1.17 `rcS` applies every `PX4_PARAM_<NAME>` before the airframe and before `ekf2 start` (`ROMFS/px4fmu_common/init.d-posix/rcS`). Appending `param set` to `px4-rc.params` does not: that file is not sourced, which is why a flight log can still show `NAV_RCL_ACT` 2 after the old append. Re-running the installer replaces the env file. `param set` inside rcS overrides a value stored from a previous run.

After PX4 connects, the node reads `EKF2_EV_CTRL`, `EKF2_EV_DELAY`, `EKF2_GPS_CTRL`, `EKF2_HGT_REF`, `EKF2_MAG_TYPE`, `EKF2_RNG_CTRL`, `SYS_HAS_MAG`, `GF_MAX_VER_DIST`, `GF_ACTION`, `NAV_RCL_ACT`, `NAV_DLL_ACT`, and `UXRCE_DDS_SYNCT` back over the SITL MAVLink port (18570) and logs each value. `EKF2_EV_DELAY`, `EKF2_RNG_CTRL`, `GF_MAX_VER_DIST`, and `GF_ACTION` are checked in vision mode. Arming is refused if any of them disagree with this profile. uXRCE in v1.17 has no parameter-by-name topic, and these EKF parameters are reboot-required, so a runtime set after `ekf2 start` would not change the running estimator.

Both modes also set `COM_OF_LOSS_T 1.0`, `COM_OBL_RC_ACT 4` (Land), `COM_RC_LOSS_T 35.0`, `NAV_RCL_ACT 1`, `NAV_DLL_ACT 0`, and `UXRCE_DDS_SYNCT 0`.

`NAV_DLL_ACT` 0 is sim-only. The x500 airframe default is 2, and then PX4 refuses to arm with "No connection to the GCS" when headless SITL has no QGroundControl. The prearm text names that cause. Do not put `NAV_DLL_ACT` 0 on a real vehicle.

`UXRCE_DDS_SYNCT` defaults to 1 in v1.17: the client measures the offset between the agent OS clock and PX4 time and rewrites message stamps. In this stack every clock is `/clock`. Syncing to the agent would turn those stamps into wall time, so the profile forces 0. The client starts after the `PX4_PARAM_*` loop, which is what the reboot-required flag needs.

### vision (indoor default)

External vision is the primary aid. GPS stays on the vehicle and keeps publishing `/fmu/out/vehicle_gps_position`.

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `EKF2_EV_CTRL` | 9 | bits 0 and 3: horizontal position and yaw. Vertical position (bit 1) and velocity (bit 2) stay off |
| `EKF2_MAG_TYPE` | 5 | None. Magnetometer fusion is off |
| `SYS_HAS_MAG` | 1 | firmware default, not exported. The compass stays required |
| `EKF2_HGT_REF` | 0 | barometric height |
| `EKF2_EV_DELAY` | 0 | ms, from `EKF2_EV_DELAY` if set. Read at EKF start |
| `EKF2_EV_NOISE_MD` | 0 | use the variances on the message |
| `EKF2_GPS_CTRL` | 0 | GNSS fusion off. The navsat sensor still publishes |
| `EKF2_RNG_CTRL` | 1 | conditional range aid. It fuses nothing until a downward lidar is on the model |
| `COM_ARM_WO_GPS` | 1 | arming is allowed if the GPS check fails |
| `GF_MAX_VER_DIST` | 3.0 | metres above home. The vertical fence is off while this is 0 |
| `GF_ACTION` | 5 | Land mode in PX4 v1.17. Value 3 is Return mode |
| `UXRCE_DDS_SYNCT` | 0 | do not sync PX4 time to the agent OS clock |

`EKF2_EV_CTRL` in v1.17 is a bitmask: bit 0 horizontal position, bit 1 vertical position, bit 2 3D velocity, bit 3 yaw. 9 is `0b1001`. 15 is `0b1111` and adds velocity and vision height.

Vision mode disables magnetometer fusion (`EKF2_MAG_TYPE` 5, None in `src/modules/ekf2/EKF/common.h`). Yaw comes from external vision (`EKF2_EV_CTRL` bit 3), and an indoor magnetometer would fight that heading. The [rtabmap_drone_example](https://github.com/matlabbe/rtabmap_drone_example) airframe does the same. GPS mode leaves `EKF2_MAG_TYPE` at the firmware default 0 (Automatic) and does not export it.

`EKF2_MAG_TYPE` stops EKF2 from fusing the magnetometer. It does not remove the sensor. `SYS_HAS_MAG` stays at the firmware default of 1 in both modes (`system_params.c`), and neither profile exports it, so the commander's compass presence check still runs (`magnetometerCheck.cpp`). The x500 keeps its magnetometer. Vision yaw comes from external vision (`EKF2_EV_CTRL` bit 3), so the vision profile sets `EKF2_MAG_TYPE` to 5 and the compass is present but not fused. GPS mode leaves `EKF2_MAG_TYPE` at 0 (Automatic).

RTAB-Map's twist is body velocity with a covariance that is often too small or not a real velocity uncertainty, so fusing it pulls the EKF off the pose. Velocity fusion stays off unless `EKF2_EV_CTRL=15` is set in the environment before the params installer runs. The bridge still fills the velocity fields; EKF2 ignores them while bit 2 is clear. Bit 1 stays clear so a vision pose cannot move the height estimate. Height is the barometer (`EKF2_HGT_REF` 0). `EKF2_RNG_CTRL` 1 is conditional range aid and starts fusing only after the x500 has a downward lidar.

`GF_MAX_VER_DIST` 3.0 m is a hard ceiling above home altitude. PX4 v1.17 `geofence_params.c` defines `GF_ACTION` 3 as Return mode and 5 as Land mode, so this profile exports 5. The check in `Geofence::isBelowMaxAltitude` runs when home altitude is valid.

### Tuning `EKF2_EV_DELAY`

The parameter is milliseconds of vision lag relative to the IMU, range 0 to 300, reboot-required, so it has to be in the params block before `ekf2 start`. Default 0. The bridge already writes `timestamp_sample` from the odometry header, and EKF2 subtracts `EKF2_EV_DELAY` from that stamp again, so 50 ms would count the same delay twice. Override with the environment variable of the same name.

It is not on the uXRCE topic list. In the PX4 shell, or in the log, read uORB `estimator_aid_src_ev_pos` (`EstimatorAidSource2d`, horizontal position). `innovation[0]` and `innovation[1]` are the residuals. During a real acceleration:

- innovation the same sign as the acceleration means vision is late: raise the delay
- the opposite sign means vision is early: lower the delay

Use the value that leaves the innovation near zero and uncorrelated with acceleration. One 30 Hz camera frame is about 33 ms. RTAB-Map's processing is often 50 to 100 ms. `estimator_aid_src_ev_hgt` stays empty while bit 1 is off. `estimator_aid_src_ev_vel` stays empty while bit 2 is off.

`vision_timeout_s` sits next to that delay. It is the age, node clock minus the odometry header stamp, after which a stalled publisher is stale and the vehicle holds. It is not itself tracking loss, and a frame that is merely slow does not bump `reset_counter`. Default 0.3 s. With `use_sim_time` that is 0.3 s of sim time (`/clock`), not wall time, so a low real-time factor does not by itself trip the hold. `VISION_PROFILE=cpu` runs the cameras at 10 Hz, and RTAB-Map odometry then arrives at about 7–10 Hz in sim time. 0.3 s is a few frame periods at 10 Hz, so a CPU run does not hold on every late frame. Raise the parameter if the odometry rate is closer to 7 Hz.

`EKF2_GPS_CTRL` is 0 in this mode so GNSS is not fused. In PX4 v1.17, an active `gnss_pos` aid sends external-vision horizontal position through the bias estimator and floors its variance at `GPS_P_NOISE` squared, which would leave GPS as the horizontal reference. The navsat sensor and plugin stay enabled. This profile does not override `EKF2_GPS_P_NOISE` or `EKF2_GPS_V_NOISE`. GPS mode still uses `EKF2_GPS_CTRL` 7 with `EKF2_GPS_P_NOISE` 0.5 and `EKF2_GPS_V_NOISE` 0.3.

### gps

Classic SITL. External vision is off. `EKF2_EV_CTRL 0`, `EKF2_HGT_REF 1`, `EKF2_GPS_CTRL 7`, `EKF2_GPS_P_NOISE 0.5`, `EKF2_GPS_V_NOISE 0.3`. Magnetometer fusion stays at the firmware default `EKF2_MAG_TYPE` 0 (Automatic). `SYS_HAS_MAG` stays at 1 in this mode as well. This profile exports neither. `UXRCE_DDS_SYNCT 0` still applies.

## Environment

These are read by the params installer and `gz_start_px4_control.sh`. They are not in `.env.example`; DevOps can pass them on the PX4 service without sharing that file with other PRs.

| Variable | Default | Used for |
|----------|---------|----------|
| `ESTIMATION_MODE` | `vision` | `vision` or `gps` |
| `EKF2_EV_DELAY` | `0` | vision delay in milliseconds, 0..300 |
| `EKF2_EV_CTRL` | `9` | vision bitmask, 0..15. `15` also fuses velocity and vision height |
| `E2E_HEIGHT_TOLERANCE_M` | unset | when set, abort and land if estimate or setpoint height leaves ground truth by more than this many metres |
| `PX4_MAX_YAW_RATE_DEG_S` | `30` | ROS `max_yaw_rate_deg_s` |
| `PX4_WALL_SEGMENTS_FILE` | empty | wall JSON, if the file exists |
| `USE_SIM_TIME` | `true` | launch `use_sim_time` |

## End-to-end height check

`E2E_HEIGHT_TOLERANCE_M` (ROS `e2e_height_tolerance_m`, default 0) turns on a flight check. The node subscribes to `/ground_truth/odom` and, on each setpoint, compares up-positive heights: estimator z is the negated NED down position, setpoint z is the negated trajectory z, and ground truth z is the Gazebo ENU z. The first sample where either absolute error exceeds the tolerance aborts the active goal and lands, including during takeoff. The log line is `height abort: ...; fault_to_abort_s=<seconds>` with the sim-time gap from that fault sample to the land command. After the land detector reports landed, the node disarms.

A vision hover is graded `suspect` when every EKF height error against ground truth is within 1 mm. That match means ground truth is being fused as vision. GPS mode is not failed for the same match. `grade_hover` in `px4_control/grading.py` returns that grade.

## Arming

`/px4_control/arm` calls `/sim/preflight_check` when that service exists and its type is `std_srvs/Trigger`. If the service is missing, the type does not match, or the call times out, the node logs a warning and continues with PX4 checks. A failed trigger returns that service's message unchanged.

PX4 must then report `pre_flight_checks_pass` and not be in failsafe. Informational bits such as `manual_control_signal_lost` do not block arming on their own. When the checks fail, the response lists the true `FailsafeFlags` boolean names plus `pre_flight_checks_pass is false`. If `gcs_connection_lost` is one of those bits, the text says arming is blocked because `NAV_DLL_ACT` defaults to 2 and no QGroundControl heartbeat is present, and that the sim profile sets `NAV_DLL_ACT` 0.

A missing or failed magnetometer is named from `estimator_status_flags`. `cs_mag_fault` says the compass is unhealthy. `fs_bad_mag_x`, `fs_bad_mag_y`, `fs_bad_mag_z`, or `fs_bad_mag_decl` says fusion failed. In GPS mode, if none of `cs_mag`, `cs_mag_hdg`, or `cs_mag_3d` is set and yaw is not aligned (`cs_yaw_align` and `cs_ev_yaw` both clear), the text says the magnetometer is missing. `SYS_HAS_MAG` stays at 1, so the commander still requires a compass. A world with no magnetometer plugin then denies arming with `Preflight Fail: Compass Sensor missing`, `No valid data from Compass`, or `Found 0 compass`.

In vision mode `EKF2_MAG_TYPE` 5 means `cs_mag`, `cs_mag_hdg`, and `cs_mag_3d` stay clear by design. Clear mag flags with `cs_yaw_align` and `cs_ev_yaw` also clear mean EKF2 has not fused vision yaw yet. The text says arming is waiting for vision yaw and points at `/fmu/in/vehicle_visual_odometry` rate and age. It does not call that a missing compass. A missing or failed compass in vision mode is still reported from `cs_mag_fault` or from the commander's own health text.

The parameter read-back has to match before arming. A mismatch is returned as the arm failure.

## Clocks

Control `dt` is the gap between node-clock stamps (`motion.py` `time_s - _last_t`). With `use_sim_time` that clock follows `/clock`, so the gap is sim time, not `1/setpoint_rate_hz` and not wall time. Takeoff, land, goto, hold, the arm command retry, and the offboard switch use that same clock.

Discovery and startup use the steady clock, so a stuck `/clock` still expires them: the preflight service wait and its call, the wait for the first `vehicle_status` and `vehicle_odometry`, the parameter read-back, and the Drone client's wait for an action or service server (`time.monotonic`). The MAVLink socket wait is also `time.monotonic`.

## Walls

Simulation writes `includes/gz/walls/<world>.json` (PR #21). The start script exports that path as `PX4_WALL_SEGMENTS_FILE` when the file exists. The text list is not a wall format.

The document matches that writer. `frame` is `world_enu` (Gazebo world ENU, x east, y north) or `local` (already the vehicle frame; real hardware). Each segment is `{"start": [x, y], "end": [x, y], "source": "...", "kind": "box"|"cylinder"|"mesh"}`. `units` is `m`. `segment_count` must match the list when it is present.

A `local` map is used as written. A `world_enu` map is not installed until two poses match. The spawn pose is only the static TF `world` -> `spawn` (x, y, and ENU yaw; z stays 0). The local pose is `vehicle_local_position` `x`, `y`, and `heading`, taken while the vehicle is landed and before arming, once `cs_yaw_align` is set and `xy_valid` is true. The rotation is the local heading minus the spawn heading, with the ENU yaw converted to NED in `frames.py`. The translation is the same pair of positions. GPS and mag keep local north on true north, so that rotation is about 0. Vision heading is 0 at the spawn, so the rotation is the negated world heading. The node does not switch on the estimation mode. If `world` -> `spawn` is not available, the geofence stays off and the node logs that once.

Arming freezes the transform and logs it. A change in `heading_reset_counter` while landed derives it again. The same change in flight brakes to a hold, logs the reset, and turns the geofence off until the vehicle is landed and the frame can be matched again.

Speed toward a wall is limited with the same `a_brake` used for holds. The cap along a commanded direction uses the clearance along that direction, so a diagonal approach is not given the head-on stopping speed:

```
v_max = sqrt(2 * a_brake * max(0, d - r_drone - margin))
```

`r_drone` defaults to 0.35 m and `margin` to 0.40 m. A goal or a path that would enter that margin is rejected. A missing file is an empty wall set: no extra limit, and a warning is logged. `config/walls_format_example.json` shows a `local` map and is not loaded by default.

## Launch

`includes/gz/startFiles/gz_start_px4_control.sh` sources ROS, `/home/px4/ws_px4`, and `/home/px4/ws_control` when that overlay exists, then:

```bash
ros2 launch px4_control px4_control.launch.py \
  use_sim_time:=true \
  estimation_mode:=${ESTIMATION_MODE:-vision}
```

Start it on the same DDS domain as the Micro-XRCE-DDS agent, after the clock bridge is up.

## Needed from DevOps

This change does not edit Compose files, Dockerfiles, or CI.

1. Colcon-build `ros2_ws/src/px4_control` and `px4_control_interfaces` in the image after `px4_msgs` v1.17.0. Image workspace `/home/px4/ws_px4`; optional overlay `/home/px4/ws_control`.
2. Pass `ESTIMATION_MODE` into the PX4 service environment so `gz_modifications.bash` writes `px4_control_params.env` before `gz_start_px4_gz_sim.sh`. Optional, listed in the Environment section above: `EKF2_EV_DELAY`, `EKF2_EV_CTRL`, `PX4_MAX_YAW_RATE_DEG_S`, `PX4_WALL_SEGMENTS_FILE`, and `USE_SIM_TIME`. Set `PX4_IMAGE=alienkh/px4_sim:1.17.0_121` for a live run. `px4_sim:1.17.0_01` is not published.
3. Start `includes/gz/startFiles/gz_start_px4_control.sh` on the same DDS domain as the XRCE agent, after the bridge so `/clock` exists. `use_sim_time` defaults to true.
4. Simulation: `/sim/preflight_check` as `std_srvs/Trigger`, wall files at `includes/gz/walls/<world>.json`, and `/ground_truth/odom` in ENU at about 50 Hz.
5. Leave the GPS sensor and `/fmu/out/vehicle_gps_position` in place.

GPU image builds are out of scope here.

## Headless flight

The default airframe in `gz_start_px4_gz_sim.sh` is still `x500_depth`. On PX4 v1.17 that model fails to spawn when the OakD-Lite link is no longer named `camera_link`. That spawn fix is a separate change. A plain `x500` GPS flight (`make px4_sitl gz_x500`, airframe 4001) is the first live proof when the depth camera does not spawn or CPU rendering drops the real-time factor to a few thousandths and camera frames reach ROS at 0–2 Hz. Do not commit a local copy of the camera model to work around the spawn failure.

Headless Gazebo should use Mesa llvmpipe (`LIBGL_ALWAYS_SOFTWARE=1`, `GALLIUM_DRIVER=llvmpipe`) under `HEADLESS=1`, with Xvfb if the render path needs an X server.

The image tag `px4_sim:1.17.0_01` is not on Docker Hub. Live runs use `PX4_IMAGE=alienkh/px4_sim:1.17.0_121`. That pin stays out of `.env` here so it does not collide with the DevOps change.
