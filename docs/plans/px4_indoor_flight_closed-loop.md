# PX4 Indoor Flight Closed-Loop Integration

Goal: fly the x500_depth quadrotor in `apt_world` (RGBD default) with RTAB-Map SLAM feeding VIO into PX4 and Nav2 driving it via `/cmd_vel`. This revision is a full re-review for correctness and simplicity: it corrects a verified error from the previous revision (Nav2's actual Jazzy default), removes a design decision that added a race condition without benefit (bridge owning transforms), and keeps everything else: no image build, sim-time everywhere, CycloneDDS as an inactive capability, DDS agent kept at v2.4.3, an extended CI smoke test, a GPU pass marked untested, and m-explore bypassed.

## Read-only findings that shape this plan

Topics into/out of Nav2 and the bridges (verified in code):
- Nav2 consumes `/rtabmap/odom` + `/rtabmap/cloud_obstacles` + TF (from RTAB-Map, not the offboard node) and **publishes `/cmd_vel`**.
- **`/cmd_vel` type mismatch (blocker) — corrected after verification:** I initially assumed Nav2 Jazzy defaults `/cmd_vel` to `TwistStamped`. Verified against [Nav2 docs](https://docs.nav2.org/configuration/packages/configuring-controller-server.html): `enable_stamped_cmd_vel` actually **defaults to `false` in Jazzy** (only Kilted+ defaults `true`). So to "switch to Nav2 TwistStamped" (per user) it must be **explicitly set `true`** in `nav2_params.yaml`, in all three sections that touch `/cmd_vel`: `controller_server`, `behavior_server`, and `velocity_smoother` (all three are configured in this repo's `nav2_params.yaml`, so all three are active in the launch and must agree on type). [`microxrce_offboard.py`](../../includes/gz/microxrce_offboard.py) is updated to subscribe `TwistStamped` (reading `msg.twist`).
- **Dead code kept (per user):** `tf_broadcaster` (line 129) and the `/cmd_vel` `recovery_node_publisher` (line 119) are unused, but the user asked to **leave them in place for now** (they may back future recovery behavior). Not removed.
- **`Clock().now()` ignores `use_sim_time`:** the node uses a fresh `rclpy.clock.Clock()` rather than `self.get_clock()`, so sim time cannot take effect until this is changed to the node clock AND PX4 stops double-syncing time.
- **No `/imu` today:** the new bridge publishes it on `base_link`. The **rgbd** branch of [`gz_start_rtabmap.sh`](../../includes/gz/startFiles/gz_start_rtabmap.sh) uses `imu_topic:=/imu approx_sync:=true` with no `wait_imu_to_init` (rgbd has depth, so it doesn't need IMU to initialize). The **stereo** branch correctly uses `wait_imu_to_init:=true` + `approx_sync_max_interval:=0.001` (stereo VIO needs IMU for gravity/scale init) — these are good and are now viable because `/imu` is reliably published.
- **Stereo frame bug (the only real defect):** stereo path uses `frame_id:=oak-d-base-frame` (in standalone [`gz_start_rtabmap_stereo.sh`](../../includes/gz/startFiles/gz_start_rtabmap_stereo.sh)), absent from [`x500_urdf.urdf`](../../includes/gz/x500_tf_publisher/x500_urdf.urdf) (root `base_link`). Fix = set `frame_id:=base_link` and add `use_sim_time:=true`, while **keeping** stereo's `wait_imu_to_init`/`approx_sync_max_interval` (better than degrading stereo to the rgbd profile).

Transform work already inside `microxrce_offboard.py` (kept there, see Step 2 for why):
- First transformation matrix: on first `/fmu/out/vehicle_odometry`, build `T_RpBp_zero` (initial PX4 attitude) and `T_RpRv = T_RpBp_zero · T_BvBp^T` (VIO/ROS -> PX4-NED alignment).
- Per-frame: `T_RvBv -> T_RpBp`, converting VIO position/velocity/angular-rate/variances ENU/FLU -> NED/FRD, then publishing `/fmu/in/vehicle_visual_odometry` (feeds EKF2 indoors).
- Body convention `T_BvBp = diag(1,-1,-1)` (FLU<->FRD).

DDS agent: PX4 v1.17 + ROS 2 Jazzy require Micro-XRCE-DDS-Agent **v2.4.3**; PX4 docs warn the v2.x client is incompatible with v3.x agents. Therefore v2.4.3 is kept (documented), **not** bumped.

## Step 0 — Branch (first action)
- `git checkout -b feature/px4-imu-bridge-indoor-flight` from current HEAD before any edits.

## Step 1 — New bridge node `includes/gz/px4_imu_bridge.py` (kept intentionally simple — single responsibility)
Re-reviewed for complexity: having the bridge also own the VIO transform pipeline would require it to independently subscribe `/fmu/out/vehicle_odometry` and compute its own "first transformation matrix" zero-reference — a **second, independent** copy of the same zeroing logic already running inside `microxrce_offboard.py`. Two processes each latching onto whichever message they happen to receive first as their "zero" is a race condition with no guarantee they agree, and the bridge broadcasting TF would duplicate what RTAB-Map's `rtabmap.launch.py` already publishes (`publish_tf` is on by default). Both would add real complexity/risk for no benefit. **Decision: keep this node to one job.**
- Subscribes (PX4 DDS, `rmw_qos_profile_sensor_data`): `/fmu/out/sensor_combined` (`gyro_rad`, `accelerometer_m_s2`) and `/fmu/out/vehicle_attitude` (quaternion).
- Converts FRD (PX4 body) -> FLU (ROS body), same fixed pi-rotation convention as `microxrce_offboard.py` (negate Y/Z).
- Publishes `/imu` (`sensor_msgs/msg/Imu`, `frame_id: base_link`) — matches the rgbd branch's default `frame_id`, so both rgbd and (fixed) stereo consume `/imu` the same way.
- No odometry subscription, no TF broadcast, no shared state with any other node.
- `use_sim_time:=true`, uses `self.get_clock().now()` for the message header stamp.

This still directly delivers "IMU from PX4's own DDS" — it's just scoped to IMU only, not also the VIO/odometry transform (see Step 2 for why that stays where it is).

## Step 2 — Fix `microxrce_offboard.py` (both `includes/gz/` and `includes/gazebo_classic/`) — logic unchanged, only bugs fixed
- **Keep unchanged:** the entire existing VIO ingestion / first-transformation-matrix (`T_RpBp_zero`, `T_RpRv`) / per-frame `T_RvBv -> T_RpBp` / `/fmu/in/vehicle_visual_odometry` publish pipeline. It already works correctly today and is self-contained (depends only on `vehicle_odometry` + the VIO odom topic, not on anything the new bridge does) — moving it would add risk, not remove it. Also keep `tf_broadcaster` and `recovery_node_publisher` as-is (dead but harmless, per user).
- Fix only: VIO odometry subscription `/odom` -> `/rtabmap/odom`; `/cmd_vel` subscription type `Twist` -> `geometry_msgs/msg/TwistStamped` (read `msg.twist`); every `Clock().now()` -> `self.get_clock().now()`; drop the dead `/camera/camera_info` subscription (`self.timestamp` is never read elsewhere).
- Run with `use_sim_time:=true`.

## Step 3 — Nav2 cmd_vel -> TwistStamped — [`nav2_params.yaml`](../../includes/gz/Params/nav2/nav2_params.yaml)
Verified against [Nav2 docs](https://docs.nav2.org/configuration/packages/configuring-controller-server.html): `enable_stamped_cmd_vel` **defaults to `false` on Jazzy** (Kilted+ only). To actually get TwistStamped (per user) it must be set explicitly. Add `enable_stamped_cmd_vel: True` to all three `/cmd_vel`-touching sections already present in this file (must be consistent across all of them or the topic ends up with mismatched publisher/subscriber types):
- `controller_server` (line ~113, next to `use_sim_time: True`)
- `behavior_server` (line ~307, next to `use_sim_time: True`)
- `velocity_smoother` (line ~330, next to `use_sim_time: True`)

## Step 4 — Sim time everywhere + single clock source (PX4-documented Gazebo-clock scenario)
Evaluated the user's "use `UXRCE_DDS_SYNCT=true` together with `use_sim_time=true` if possible": this combination is **invalid**. PX4's time sync assumes the **agent wall clock**, so with `SYNCT=true` and ROS on sim time the bridge applies a wrong offset and produces `Time jump detected. Resetting time synchroniser` (PX4 PR [#21757](https://github.com/PX4/PX4-Autopilot/pull/21757), issue [#22522](https://github.com/PX4/PX4-Autopilot/issues/22522)). So we fall back to PX4's preferred plan for a Gazebo perception stack. Evidence: PX4 [ROS 2 User Guide](https://docs.px4.io/v1.17/en/ros2/user_guide) states the OS-clock default is only valid at RTF ~= 1; for Gazebo sims where ROS consumes Gazebo sensors or RTF < 1 (our RTAB-Map/Nav2 case) it prescribes the Gazebo-clock scenario, confirmed by the Beniamino Pozzan PX4 talk ([HtdU9DwmjFg](https://www.youtube.com/watch?v=HtdU9DwmjFg)). Main course:
- Set/confirm `use_sim_time:=true`: bridge, offboard, RTAB-Map (rgbd already true; add to stereo), [`start_nav2.sh`](../../includes/gz/startFiles/start_nav2.sh) and [`start_nav2_rviz.sh`](../../includes/gz/startFiles/start_nav2_rviz.sh) (`False`->`true`), robot_state_publisher (already true), Nav2 params (already true).
- Add `param set-default UXRCE_DDS_SYNCT 0` in [`gz_modifications.bash`](../../includes/gz/gz_modifications.bash) so PX4 stops its own timestamp sync and Gazebo `/clock` (already bridged in [`config_gz_bridge.yaml`](../../includes/gz/config_gz_bridge.yaml)) is the single time source for PX4 and every ROS node.

## Step 5 — Rename `start_mission.sh` -> `start_offboard_flight.sh`
Better, meaningful name (options considered: `start_offboard_flight.sh`, `start_flight_bridge.sh`, `run_offboard_control.sh`; chosen: **`start_offboard_flight.sh`** — it clearly says it starts offboard-mode flight). `git mv includes/gz/startFiles/start_mission.sh includes/gz/startFiles/start_offboard_flight.sh` and:
- `cd ${HOME}/volume/includes/gz` (real location) instead of the missing `startFiles/` path.
- Launch `px4_imu_bridge.py` (background) then `microxrce_offboard.py --OffboardControllEnable True --TakeoffHeight $FLIGHT_HEIGHT` (foreground).

## Step 6 — Compose `Offboard` service — [`docker-compose-px4.yml`](../../docker-compose-px4.yml)
- New `Offboard` service, profile `mission` (opt-in; it actively arms/flies): `depends_on` PX4 healthy + Rtabmap started; same `includes/gz` + `startFiles` + `Params` binds; `environment: FLIGHT_HEIGHT=${FlightHeight:-2}`; `command` runs `start_offboard_flight.sh`.
- Add a **commented** `RMW_IMPLEMENTATION: rmw_cyclonedds_cpp` line to service environments (CycloneDDS capability present but not activated).
- Full stack now: `COMPOSE_PROFILES=gcs,slam,nav,mission`.

## Step 7 — Dockerfiles
- NO-GPU [`Dockerfile_px4_sim_NO_GPU`](../../dockerFile/Dockerfile_px4_sim_NO_GPU): add `python3-numpy python3-scipy` (bridge + offboard need `scipy.spatial.transform.Rotation`); add a **commented** `# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (not active); keep XRCE agent `v2.4.3` with a comment on why it is not bumped.
- GPU [`Dockerfile_px4_sim_with_GPU`](../../dockerFile/Dockerfile_px4_sim_with_GPU): add `python3-scipy` (numpy already pip-installed) so the bridge/offboard can run; add a prominent header comment `# NOT FULLY TESTED`. No deeper rework.

## Step 8 — World + RTAB-Map stereo fix (better plan) + m-explore
- [`apt_world.sdf`](../../includes/gz/worlds/apt_world.sdf): `<start_paused>1</start_paused>` -> `0` (sim must advance headless).
- **Stereo fix (chosen better plan):** fix only the real defect and keep stereo's good IMU-init settings. In the [`gz_start_rtabmap.sh`](../../includes/gz/startFiles/gz_start_rtabmap.sh) stereo branch and standalone [`gz_start_rtabmap_stereo.sh`](../../includes/gz/startFiles/gz_start_rtabmap_stereo.sh): set `frame_id:=base_link` (replacing the nonexistent `oak-d-base-frame`) and add `use_sim_time:=true`, while **keeping** `imu_topic:=/imu`, `approx_sync:=true`, `wait_imu_to_init:=true`, and `approx_sync_max_interval:=0.001`. Rationale: stereo VIO genuinely benefits from IMU-gated initialization (rgbd skips it only because depth gives scale); now that `/imu` is reliably published from PX4 DDS, the robust stereo profile works — better than dumbing stereo down to rgbd's profile. Both consume the bridge's `/imu` on `base_link`.
- m-explore: **bypass** — leave [`start_explore.sh`](../../includes/gz/startFiles/start_explore.sh) unwired and out of Compose (not installed; not needed for flight).

## Step 9 — CI smoke test — [`scripts/smoke_test.sh`](../../scripts/smoke_test.sh) + [`.github/workflows/docker-image.yml`](../../.github/workflows/docker-image.yml)
- A smoke test already exists (headless SITL, checks `/clock` + `/fmu/out/vehicle_odometry`). Extend it: also start `px4_imu_bridge.py` and assert `/imu` and `/fmu/out/sensor_combined` publish (rgbd, `default` world, headless). Keeps CI bounded; does not add full Nav2/flight to CI.

## Step 10 — Docs — [`.env.example`](../../.env.example) / [`README.md`](../../README.md)
- Document `mission` profile, `FlightHeight`, the PX4-DDS IMU source (`sensor_combined` + `vehicle_attitude` -> `/imu`; OakD `imu/data` sensor left defined but unused), CycloneDDS as an opt-in (uncomment `RMW_IMPLEMENTATION`), sim-time-everywhere note, and a GPU "not fully tested" caveat.

## Step 11 — Validation (NO image build)
Per the user: do **not** build; use the already-present/published image. Because `includes/gz`, `startFiles`, and `Params` are bind-mounted, all script/param/node changes are picked up by the existing image at runtime.
- Pull/run existing `docker.io/alienkh/px4_sim:${px4TAG}` headless (`QT_QPA_PLATFORM=offscreen`). If `scipy` is missing from the current published image, `pip install --break-system-packages scipy` inside the container for this validation only (the Dockerfile change covers future builds).
- Bring up PX4 (`CameraType=rgbd World=apt_world`) + StatePublisher + Rtabmap + mission bridge.
- Check topics: `/clock`, `/fmu/out/{vehicle_odometry,sensor_combined,vehicle_attitude}`, `/imu`, `/rtabmap/odom`, `/fmu/in/{vehicle_visual_odometry,trajectory_setpoint}`, and that Nav2 `/cmd_vel` (TwistStamped) reaches the offboard node.
- Confirm arm -> offboard -> climb to `FlightHeight` (via `/fmu/out/vehicle_local_position.z`), report measured altitude, then tear down. If the current image blocks a full flight test, report exactly which runtime piece is missing and stop short of building.

## Out of scope / explicitly deferred
- Building any image (per user).
- GPU compose file `docker-compose-px4-GPU.yml` beyond the Dockerfile dep add + untested note.
- m-explore exploration.
- Bumping the DDS agent past v2.4.3 (would break the Jazzy client).
