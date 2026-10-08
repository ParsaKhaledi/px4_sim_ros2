# scripts/

Helpers for launching the stack, checking health, and running the headless tests.

| Script | Purpose |
|--------|---------|
| [up.sh](up.sh) | Start Compose with profiles, camera, world, and spawn pose |
| [smoke_test.sh](smoke_test.sh) | Headless compose smoke test. Camera rates run first |
| [run_e2e.sh](run_e2e.sh) | Out-and-back mission. Skips if `px4_control.Drone` is missing |
| [record_flight.sh](record_flight.sh) | Rosbag of the grading topics plus the newest PX4 ULog |
| [health_report.sh](health_report.sh) | Latest JSONL status per service |
| [compose_stack.sh](compose_stack.sh) | `up` / `down` / `logs` for the headless override |
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
E2E_LEG_LENGTH_M=0.3 ./scripts/run_e2e.sh
# nightly distance
E2E_LEG_LENGTH_M=1.0 ./scripts/run_e2e.sh
```

The mission is takeoff to 2 m, hold 10 s, forward `E2E_LEG_LENGTH_M`, yaw 180 deg, forward the same distance, land, disarm. Thresholds are the `E2E_*` keys in `.env`. Grading uses `/ground_truth/odom` relative to the takeoff point. If the Drone class or that topic is missing, the script exits 0 and writes `logs/flights/trajectory.json` with `"status": "skipped"`.

`record_flight.sh start|stop` stores a rosbag and `flight.ulg` under `logs/flights/`. `run_e2e.sh` calls it around the mission.

## health_report.sh

```bash
./scripts/health_report.sh
./scripts/health_report.sh logs/health
```
