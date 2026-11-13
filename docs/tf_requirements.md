# TF requirements (indoor vision → Nav2)

Requirements for the indoor stack: Gazebo GT frames, RTAB-Map SLAM TF, URDF camera tree, and Nav2 costmaps. Obstacle keep-out is owned by **SLAM occupancy → Nav2** (see [nav2_obstacle_limits.md](nav2_obstacle_limits.md)). World-frame JSON walls are not on the product path.

## Axis conventions

| Frame family | Axes | Where |
|--------------|------|--------|
| ENU world | x east, y north, z up | Gazebo `world`, `/ground_truth/odom`, static `spawn` |
| FLU body | x forward, y left, z up | ROS `base_link`, RTAB-Map odom child, Nav2 `robot_base_frame` |
| Optical | z forward, x right, y down | `*_optical_frame` under the Oak-D URDF |
| NED / FRD | PX4 only | `/fmu/in/vehicle_visual_odometry` after `px4_control` converts ENU/FLU → NED/FRD |

Do not mix NED into the ROS TF tree. Vision stays ENU/FLU until the control bridge.

## Required TF tree (two roots)

```
world ──(static)──► spawn                 # sim_monitor.spawn_frame
world ──(dynamic)─► base_link_gt          # same node, from /ground_truth/odom

map ──(RTAB-Map)──► odom ──(RTAB-Map)──► base_link
                                         │
                                         ├─ camera_link (or OakD-Lite/base_link) ─ camera_* / stereo_* / imu_link
                                         └─ (optional lidar_sensor_link from SDF patch)
```

`world` and `map` are **not** connected. That is intentional.

## Who publishes what

| Transform | When it must exist | Publisher | Notes |
|-----------|--------------------|-----------|--------|
| `world` → `spawn` | Before preflight / any world↔local wall math | `sim_monitor.spawn_frame` (static, from `PX4_GZ_MODEL_POSE`) | Uses x, y, **yaw**; z = 0 (floor). Roll/pitch dropped. |
| `world` → `base_link_gt` | While `/ground_truth/odom` streams | `sim_monitor.spawn_frame` | Child **must** be `base_link_gt` (`GT_CHILD_FRAME`), never `base_link`. |
| `map` → `odom` | Before Nav2 global costmap / planner | RTAB-Map (`publish_tf_map:=true`) | Present once SLAM is tracking. |
| `odom` → `base_link` | Before Nav2 local costmap / controller | RTAB-Map visual odometry (`publish_tf_odom:=true`) | `frame_id` launch arg is the robot base (`base_link`). |
| `base_link` → camera / IMU / optical | Before RTAB-Map and preflight optical check | `robot_state_publisher` + `x500_urdf.urdf` | No second static camera TF publisher. |
| `/ground_truth/odom` (topic) | GT eval / spawn TF | Gazebo `OdometryPublisher` (`odom_frame=world`, `robot_base_frame=base_link_gt`) | Covariance topic must **not** be `/model/.../odometry_with_covariance` (that feeds EKF2). |

## Nav2 frame contract

| Node / costmap | `global_frame` | `robot_base_frame` | Odometry |
|----------------|----------------|--------------------|----------|
| `bt_navigator` | `map` | `base_link` | `/rtabmap/odom` |
| `local_costmap` | `odom` | `base_link` | (TF + obstacle cloud) |
| `global_costmap` | `map` | `base_link` | `/rtabmap/cloud_obstacles` + static map |
| `behavior_server` | `odom` | `base_link` | recoveries in odom |
| `velocity_smoother` | — | — | `/rtabmap/odom` |

Obstacle cloud: `/rtabmap/cloud_obstacles`. Costmaps transform it using the cloud `header.frame_id` and the tree above. Wrong or missing `map`/`odom`/`base_link` makes inflation mark the wrong cells relative to the vehicle.

## Spawn yaw vs map yaw (alignment rule)

With `--RGBD/StartAtOrigin true`, RTAB-Map sets the **initial** `map`/`odom` pose to identity at the vehicle pose when SLAM starts. So at t₀:

* `map` +X is vehicle forward (FLU), not necessarily world east.
* `spawn` +X is also vehicle forward at spawn (`PX4_GZ_MODEL_POSE` yaw).

**For Nav2 keep-out from SLAM occupancy:** walls live in `map`/`odom`. They stay aligned with the vehicle as long as RTAB-Map owns `map`→`odom`→`base_link` and Nav2 uses those frame names. You do **not** need `world`→`map`.

**When spawn yaw still matters:** comparing GT / world-ENU geometry to the vehicle, or transforming any leftover `world_enu` prior into body/local. Then use `world`→`spawn` (same yaw as `PX4_GZ_MODEL_POSE`). Changing spawn yaw without restarting SLAM does not rotate an existing map; restart RTAB-Map after a pose change.

## Hard rules

1. **Never** publish `world` → `base_link` (or any second parent of `base_link`). Use `base_link_gt`.
2. **Never** feed ground truth into EKF2 (no default Gazebo covariance topic; leave RTAB-Map `ground_truth_frame_id` empty in flight).
3. RTAB-Map `frame_id` must be `base_link` (matches Nav2 `robot_base_frame` and the URDF root). Do not use `oak-d-base-frame` or `camera_rgb_frame` as the robot frame.
4. RTAB-Map, Nav2, bridge, and `robot_state_publisher` must share **sim time** (`use_sim_time:=true` / `USE_SIM_TIME`) so TF stamps line up with `/clock`.
5. Camera optical frames stay in the URDF; do not add a parallel static TF publisher for the same links.

## Failure modes

| Misalignment | Symptom |
|--------------|---------|
| `base_link` also under `world` | TF tree conflict; Nav2 / RTAB-Map lookup errors |
| Stereo `frame_id` = missing frame (`oak-d-base-frame`) | No `odom`→`base_link`; costmaps never update; planning fails |
| RTAB-Map `use_sim_time:=false` while Nav2/sim use `/clock` | Extrapolation errors; empty or stale costmaps |
| Spawn yaw ≠ SLAM start yaw (restarted pose, old DB) | World priors / GT overlays skew vs occupancy; Nav2 occupancy itself still self-consistent |
| GT covariance on PX4 vision topic | EKF2 tracks truth; invalidates vision flight tests |
| URDF not running | Preflight fails IMU→optical TF; RTAB-Map cannot place depth in `base_link` |
| Nav2 `robot_base_frame` ≠ RTAB-Map `frame_id` | Footprint and obstacles disagree; false lethal / missed walls |

## Launch checklist

* `gz_start_sim_helpers.sh` → spawn + GT TF
* `gz_start_state_publisher.sh` → URDF tree from `base_link`
* `gz_start_rtabmap.sh` (compose) → `frame_id:=base_link`, `map_frame_id:=map`, `vo_frame_id:=odom` (do **not** set `odom_frame_id`; that switches SLAM to TF-input odom), `use_sim_time` true, `RGBD/StartAtOrigin true`
* Nav2 params → `map` / `odom` / `base_link` as in the table above

Preflight default pairs: `world:spawn`, `world:base_link_gt`, `base_link:camera_rgb_frame` (or stereo left), `base_link:imu_link`.
