# scripts/

Helpers for launching the stack, checking health, and running the headless tests.

| Script | Purpose |
|--------|---------|
| [up.sh](up.sh) | Start Compose with profiles, camera, world, and spawn pose |
| [smoke_test.sh](smoke_test.sh) | Headless compose smoke test. Camera rates run first |
| [run_e2e.sh](run_e2e.sh) | Out-and-back flight, with a sim restart after a crash |
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
GPU=1 ./scripts/up.sh
```

`GPU=1` selects [docker-compose-px4-GPU.yml](../docker-compose-px4-GPU.yml). The same stack is `docker compose -f compose.yml -f compose.gpu.yml -f compose.gui.yml up`.

## smoke_test.sh

Brings the stack up with [compose.ci.yml](../compose.ci.yml): `HEADLESS=1`, `RTABMAPVIZ=false`, no X11 socket, no `/dev` bind, no host ports. Waits until healthchecks pass, checks camera rates, then the rest of the PX4 graph.

```bash
./scripts/smoke_test.sh docker.io/alienkh/px4_sim:1.17.0_01
SMOKE_KEEP_UP=1 SMOKE_TEST_TIMEOUT=900 ./scripts/smoke_test.sh px4_sim:ci
```

GitHub-hosted runners are a poor place for this. They have no GPU, and the camera topics need a GL renderer. Run it on a machine that can already launch the sim, or dispatch the nightly workflow onto a self-hosted runner.

## run_e2e.sh and record_flight.sh

```bash
# Pull-request shape: plain quad, no cameras, CPU only.
CameraType=none PX4_GZ_MODEL=x500 COMPOSE_SERVICES=PX4 ./scripts/run_e2e.sh

# Nightly shape: depth camera and a 1 m leg. Needs a renderer.
CameraType=rgbd PX4_GZ_MODEL=x500_depth E2E_LEG_LENGTH_M=1.0 ./scripts/run_e2e.sh
```

The mission is preflight and arm, takeoff to 2 m, hover 10 s, forward `E2E_LEG_LENGTH_M`, yaw 180°, forward the same distance, land, disarm. `px4_control.Drone` is used when it imports. Otherwise the script talks to PX4 with `OffboardControlMode`, `TrajectorySetpoint`, and `VehicleCommand`.

Grading uses `/ground_truth/odom` when that topic is publishing. Otherwise it uses the Gazebo model pose on `/world/<world>/pose/info`. Each sample also stores how far PX4 `vehicle_local_position` is from that pose. Pass limits are the `E2E_*` keys in `.env`: hover drift 5 cm, legs 30 cm ± 5 cm, yaw ± 5°, return 5 cm, height ± 10 cm.

While the vehicle should be airborne, the attempt ends early on any of these:

- tilt over `E2E_CRASH_TILT_DEG` (60°)
- height under `E2E_CRASH_MIN_HEIGHT_M`, or a fast impact near the ground
- an unexpected disarm, failsafe, or land detection
- ground truth farther than `E2E_CRASH_DIVERGENCE_M` from the setpoint
- no odometry for `E2E_ODOM_TIMEOUT_S` seconds

The reason, container log, health JSONL, rosbag, and ULog are saved under `logs/flights/attempt-<n>/`. The script then recreates the PX4 container (`./scripts/compose_stack.sh recreate`) and waits until the health check passes. `E2E_MAX_RETRIES` defaults to 2, so the first try plus two restarts is the budget. `logs/flights/trajectory.json` lists every attempt. The run fails if the last attempt does not pass.

`record_flight.sh start|stop` stores a rosbag and `flight.ulg`. `run_e2e.sh` calls it on every attempt.

## health_report.sh

```bash
./scripts/health_report.sh
./scripts/health_report.sh logs/health
```
