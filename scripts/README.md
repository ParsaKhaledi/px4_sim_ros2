# scripts/

Helpers for launching the stack, checking health, and running the headless tests.

| Script | Purpose |
|--------|---------|
| [up.sh](up.sh) | Start Compose with profiles, camera, world, and spawn pose |
| [smoke_test.sh](smoke_test.sh) | Headless compose smoke test. Camera rates run first |
| [ci_camera_flight.sh](ci_camera_flight.sh) | CI RGB-D/stereo: health + hover + out-and-back |
| [ci_rgbd_flight.sh](ci_rgbd_flight.sh) / [ci_stereo_flight.sh](ci_stereo_flight.sh) | Thin wrappers around `ci_camera_flight.sh` |
| [run_e2e.sh](run_e2e.sh) | Out-and-back (or hover-only) flight, with crash restart |
| [record_flight.sh](record_flight.sh) | Rosbag of the grading topics plus the newest PX4 ULog |
| [health_report.sh](health_report.sh) | Latest JSONL status per service |
| [compose_stack.sh](compose_stack.sh) | `up` / `down` / `recreate` / `logs` for the headless override |
| [image_assert.sh](image_assert.sh) | Check binaries inside an image without starting Gazebo |
| [check_versions.sh](check_versions.sh) | Dockerfile pins match `versions.env` |
| [check_fuel_refs.sh](check_fuel_refs.sh) | Warn when worlds reference fuel.gazebosim.org |

`versions.env` is the component pin (PX4, px4_msgs, XRCE agent, ROS distro). `.env` is the image tag and runtime knobs, including `PX4_GZ_MODEL_POSE`.

## up.sh

```bash
CameraType=rgbd World=default ./scripts/up.sh
COMPOSE_PROFILES= ./scripts/up.sh

# Sim + QGroundControl in the indoor world
COMPOSE_PROFILES=gcs CameraType=rgbd World=apt_world ./scripts/up.sh
```

`up.sh` starts [docker-compose-px4.yml](../docker-compose-px4.yml).

## smoke_test.sh

Brings the stack up with [docker-compose-px4.yml](../docker-compose-px4.yml) and the headless [compose.ci.yml](../compose.ci.yml) override: `HEADLESS=1`, `RTABMAPVIZ=false`, no X11 socket, no `/dev` bind, no host ports. Waits until healthchecks pass, checks camera rates, then the rest of the PX4 graph. The CI `flight` matrix runs this for `CameraType=none` (plain `x500`). It does not start an offboard controller. Always set `CameraType` explicitly.

```bash
CameraType=none PX4_GZ_MODEL=x500 COMPOSE_SERVICES=PX4 \
  ./scripts/smoke_test.sh docker.io/alienkh/px4_sim:1.17.0_121
SMOKE_KEEP_UP=1 SMOKE_TEST_TIMEOUT=900 ./scripts/smoke_test.sh px4_sim:ci
```

## ci_camera_flight.sh (rgbd / stereo)

Shared CI entry for `CameraType=rgbd` and `CameraType=stereo` on a CPU runner
(`VISION_PROFILE=cpu`, no GPU). Sets `PX4_GZ_MODEL=x500_depth`, starts
`PX4 StatePublisher Rtabmap` with the `slam` profile, then calls
[run_e2e.sh](run_e2e.sh) → [test_out_and_back.py](../tests/e2e/test_out_and_back.py)
with the **same** mission and `E2E_*` pass criteria for both cameras:

- takeoff → hover (`E2E_HOVER_S`, default 10 s at `E2E_TAKEOFF_HEIGHT_M` 1.5 m)
  → forward (`E2E_LEG_LENGTH_M`) → yaw 180° → return → land
- `E2E_HOVER_ONLY` is unset (full out-and-back, not hover-only)
- `E2E_HEALTH_SERVICES="PX4 StatePublisher Rtabmap"` — camera rates/TF + RTAB-Map;
  final table must have `failing=0`
- soft floors `HEALTH_CAMERA_MIN_HZ=1` / `HEALTH_SLAM_MIN_HZ=1` under software GL

```bash
./scripts/ci_rgbd_flight.sh docker.io/alienkh/px4_sim:1.17.0_121
./scripts/ci_stereo_flight.sh docker.io/alienkh/px4_sim:1.17.0_121
# or: ./scripts/ci_camera_flight.sh rgbd|stereo [PX4_IMAGE]
```

The `CameraType=none` smoke job still uses [smoke_test.sh](smoke_test.sh) only
(no offboard hover/legs).

## run_e2e.sh and record_flight.sh

```bash
# Fast flight: plain quad, no cameras.
CameraType=none PX4_GZ_MODEL=x500 COMPOSE_SERVICES=PX4 ./scripts/run_e2e.sh

# CI camera shape (hover + out-and-back) with multi-service health before takeoff.
CameraType=rgbd PX4_GZ_MODEL=x500_depth VISION_PROFILE=cpu \
  COMPOSE_PROFILES=slam COMPOSE_SERVICES="PX4 StatePublisher Rtabmap" \
  GZ_HEADLESS_RENDERING=1 E2E_TAKEOFF_HEIGHT_M=1.5 E2E_HOVER_S=10 \
  E2E_LEG_LENGTH_M=0.3 E2E_HEALTH_SERVICES="PX4 StatePublisher Rtabmap" \
  E2E_CHECK_CAMERAS=1 ./scripts/run_e2e.sh

# Same mission for stereo (left/right topics via CameraType).
CameraType=stereo PX4_GZ_MODEL=x500_depth VISION_PROFILE=cpu \
  COMPOSE_PROFILES=slam COMPOSE_SERVICES="PX4 StatePublisher Rtabmap" \
  GZ_HEADLESS_RENDERING=1 E2E_CHECK_CAMERAS=1 ./scripts/run_e2e.sh

# Start on the virtual framebuffer instead of EGL.
GZ_USE_XVFB=1 CameraType=rgbd PX4_GZ_MODEL=x500_depth \
  E2E_CHECK_CAMERAS=1 E2E_WALL_SCALE=3 COMPOSE_SERVICES=PX4 ./scripts/run_e2e.sh
```

`GZ_HEADLESS_RENDERING=1` puts a `gz` wrapper on `PATH` that turns PX4's `gz sim -s` into `gz sim --headless-rendering`, with `LIBGL_ALWAYS_SOFTWARE=1` and `GALLIUM_DRIVER=llvmpipe`. `GZ_USE_XVFB=1` starts `Xvfb` on `DISPLAY` (default `:99`) and does not add the EGL flag. The CPU image installs `libgl1-mesa-dri`, `libegl-mesa0`, and `xvfb` for this. Software rendering runs at a real-time factor around 0.3–0.6. Lockstep keeps the flight valid; `E2E_WALL_SCALE` multiplies the wall-clock timeouts (use 3 for the camera flight).

With `E2E_HOVER_ONLY=1` the mission is preflight, arm, takeoff, hover
`E2E_HOVER_S`, land, disarm (no legs/yaw). Otherwise it is takeoff to
`E2E_TAKEOFF_HEIGHT_M`, hover, forward `E2E_LEG_LENGTH_M`, yaw 180°, forward
again, land, disarm. The hover clock starts after height has held
`E2E_HOVER_HEIGHT_BAND_M` (5 cm) for `E2E_HOVER_HEIGHT_HOLD_S` (2 s).
`px4_control.Drone` is used when it imports; otherwise the script talks to PX4
with `OffboardControlMode`, `TrajectorySetpoint`, and `VehicleCommand`.

`E2E_HEALTH_SERVICES` (space-separated) runs those `healthcheck.py --service`
gates after param readback and before the first attempt. The final health table
must show `failing=0`.

Grading uses `/ground_truth/odom` when that topic is publishing. Otherwise it
uses the Gazebo model pose on `/world/<world>/pose/info`. Pass limits are the
`E2E_*` keys in `.env`. Hover-only uses `grade_hover` (drift / height / duration).

`E2E_CHECK_CAMERAS=1` subscribes to `/camera/rgb/image_raw` and
`/camera/depth/image_raw`. Blank or missing frames are a render failure (not a
crash restart) and trigger one Xvfb retry. `E2E_FLIGHT_DIR` must stay under
`logs/`. CI writes `logs/flights/rgbd` for the RGB-D matrix job. Soft-GL CI uses
`VISION_PROFILE=cpu` and `HEALTH_CAMERA_MIN_HZ=1`. `E2E_ATTEMPT_TIMEOUT`
(default 480s) and `E2E_TOTAL_TIMEOUT` bound hung runs so they reset instead of
stalling.

While the vehicle should be airborne, the attempt ends early on any of these:

- tilt over `E2E_CRASH_TILT_DEG` (60°)
- height under `E2E_CRASH_MIN_HEIGHT_M`, or a fast impact near the ground
- an unexpected disarm, failsafe, or land detection
- ground truth farther than `E2E_CRASH_DIVERGENCE_M` from the setpoint
- no odometry for `E2E_ODOM_TIMEOUT_S` seconds

The reason, container log, health JSONL, rosbag, and ULog are saved under `logs/flights/attempt-<n>/`. The script then recreates the PX4 container (`./scripts/compose_stack.sh recreate`) and waits until the health check passes. `E2E_MAX_RETRIES` defaults to 2, so the first try plus two restarts is the budget. `logs/flights/trajectory.json` lists every attempt. The run fails if the last attempt does not pass.

`record_flight.sh start|stop` stores a rosbag and `flight.ulg`. `run_e2e.sh` calls it on every attempt.

## health_report.sh

Prints the latest JSONL row per check. Auto-detect topic groups share the
same selection path as `healthcheck.py` (live odometry wins over a silent
`_v1` alias). Exits `1` when any row is `fail`.

```bash
./scripts/health_report.sh
./scripts/health_report.sh logs/health
```
