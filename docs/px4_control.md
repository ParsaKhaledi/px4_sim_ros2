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

`GoTo` and `move_forward` stream a trapezoidal position, velocity, and acceleration. They do not step the position setpoint. `turn` ramps yaw with XY locked. The default yaw rate is 30 deg/s (`max_yaw_rate_deg_s`, env `PX4_MAX_YAW_RATE_DEG_S`). An action succeeds only after the vehicle has stayed inside the settle tolerance (0.20 m, 8 deg) for 0.4 s.

Timeouts in `config/px4_control.yaml` are on the node clock. With `use_sim_time` the action waiter allows up to 10× that budget in wall time (at least 120 s, at most 900 s) so a real-time factor around 0.3–0.6 can finish. The `Drone` client timeouts are wall-clock and already generous.

## Topics, services, actions

PX4 QoS is best-effort, volatile, keep-last 5. `/cmd_vel` and the vision odometry use the default reliable QoS.

uXRCE appends `_v<MESSAGE_VERSION>` when that constant is not zero. In px4_msgs v1.17.0, `VehicleStatus` is version 1, so the live topic is `/fmu/out/vehicle_status_v1`. The node subscribes and publishes both the versioned name and the base name, and logs which subscription is actually receiving data. `VehicleOdometry`, `TrajectorySetpoint`, `OffboardControlMode`, and `VehicleCommand` are unversioned.

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

The same mission is `ros2 run px4_control out_and_back`.

`turn` is positive counter-clockwise in ROS, which is a negative NED yaw rate. `turn(180)` reverses heading. `move_forward` is a body-frame `GoTo` along the current heading. `goto(x, y, z, yaw=None, frame='enu'|'body')` is the general call. In the ENU frame, `yaw` is absolute NED radians. In the body frame it is a relative counter-clockwise angle.

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

Publishing stops on any of:

- zero quaternion
- negative, non-finite, or huge variance (`vision_max_variance`, default 25)
- optional `rtabmap_msgs/OdomInfo.lost` when that package is installed
- no sample for `vision_timeout_s` (default 0.3 s)

The bridge keeps the odometry header stamp. It does not re-stamp the last pose. `reset_counter` increments when the pose jumps farther than the motion predicted since the previous sample (`vision_reset_jump_m`, default 0.75 m).

`rtabmap_msgs` is optional. Without it, loss is detected from the odometry stream itself.

## Estimation mode

ROS parameter `estimation_mode`. Environment key `ESTIMATION_MODE=vision|gps`. Default is `vision`.

`includes/gz/gz_modifications.bash` calls `includes/gz/params/install_px4_control_params.bash`, which rewrites one marked block in `ROMFS/px4fmu_common/init.d-posix/px4-rc.params`. PX4 v1.17 does not source that file on its own (the previous `>>` append never ran). The installer also inserts one source of it into `rcS`, after the airframe and immediately before `ekf2 start`, and adds `px4-rc.params` to the posix ROMFS file list so the SITL rootfs actually contains it. Re-running it does not stack lines. Values are `param set`, so a later start overrides a value stored from a previous run. The mode is expanded when the container starts, so the PX4 shell does not need the variable itself.

Both modes also set `COM_OF_LOSS_T 1.0`, `COM_OBL_RC_ACT 4` (Land), `COM_RC_LOSS_T 35.0`, and `NAV_RCL_ACT 1`.

### vision (indoor default)

External vision is the primary aid. GPS stays on the vehicle and keeps publishing `/fmu/out/vehicle_gps_position`.

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `EKF2_EV_CTRL` | 15 | horizontal position, vertical position, velocity, yaw |
| `EKF2_HGT_REF` | 3 | height from vision |
| `EKF2_EV_DELAY` | 50 | ms, read at EKF start |
| `EKF2_EV_NOISE_MD` | 0 | use the variances on the message |
| `EKF2_GPS_CTRL` | 5 | lon/lat and 3D velocity, not altitude |
| `EKF2_GPS_P_NOISE` | 5.0 | lower GPS position weight (max 10) |
| `EKF2_GPS_V_NOISE` | 1.0 | lower GPS velocity weight (max 5) |
| `COM_ARM_WO_GPS` | 1 | arming is allowed if the GPS check fails |

### gps

Classic SITL. External vision is off. `EKF2_EV_CTRL 0`, `EKF2_HGT_REF 1`, `EKF2_GPS_CTRL 7`, `EKF2_GPS_P_NOISE 0.5`, `EKF2_GPS_V_NOISE 0.3`.

## Arming

`/px4_control/arm` calls `/sim/preflight_check` when that service exists and its type is `std_srvs/Trigger`. If the service is missing, the type does not match, or the call times out, the node logs a warning and continues with PX4 checks. A failed trigger returns that service's message unchanged.

PX4 must then report `pre_flight_checks_pass` and not be in failsafe. Informational bits such as `manual_control_signal_lost` do not block arming on their own. When the checks fail, the response lists the true `FailsafeFlags` boolean names plus `pre_flight_checks_pass is false`.

## Walls

If Simulation drops a segment file at `includes/gz/worlds/walls/<World>.txt`, the start script exports `PX4_WALL_SEGMENTS_FILE`. Each line is `x1 y1 x2 y2` in local ENU metres (x East, y North). Blank lines and `#` comments are ignored.

Speed toward a wall is limited with the same `a_brake` used for holds:

```
v_max = sqrt(2 * a_brake * max(0, d - r_drone - margin))
```

`r_drone` defaults to 0.35 m and `margin` to 0.40 m. A goal or a path that would enter that margin is rejected. A missing file is an empty wall set: no extra limit, and a warning is logged. `config/walls_format_example.txt` shows the format and is not loaded by default.

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
2. Pass `ESTIMATION_MODE` into the PX4 service environment so `gz_modifications.bash` bakes it. Also optional `PX4_MAX_YAW_RATE_DEG_S`, `PX4_WALL_SEGMENTS_FILE`, and `USE_SIM_TIME`.
3. Start `includes/gz/startFiles/gz_start_px4_control.sh` on the same DDS domain as the XRCE agent, after the bridge so `/clock` exists. `use_sim_time` defaults to true.
4. Simulation: `/sim/preflight_check` as `std_srvs/Trigger`, wall files at `includes/gz/worlds/walls/<World>.txt`, and `/ground_truth/odom` in ENU at about 50 Hz.
5. Leave the GPS sensor and `/fmu/out/vehicle_gps_position` in place.

GPU image builds are out of scope here.

## Headless flight with the camera

The airframe is `x500_depth` (OakD-Lite, PX4 airframe 4002, make target `gz_x500_depth`). `gz_start_px4_gz_sim.sh` already sets `MODEL=x500_depth`. Do not drop the camera to save CPU unless software rendering fails.

Headless Gazebo should use Mesa llvmpipe (`LIBGL_ALWAYS_SOFTWARE=1`, `GALLIUM_DRIVER=llvmpipe`) under `HEADLESS=1`, with Xvfb if the render path needs an X server. A real-time factor around 0.3–0.6 is expected. The plain `x500` model is only a fallback when the depth camera fails to render. Confirm `/camera/rgb/image_raw` and `/camera/depth/image_raw` while the camera model is active (`CameraType=rgbd` overlays `includes/gz/models/OakD-Lite-rgbd`).
