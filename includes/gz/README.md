# gz/

Gazebo Harmonic worlds, models, and the simulation helpers that run inside the PX4 container. The full description of headless mode, sim time, ground truth, offline models, wall maps, `trajectory_eval`, and `/sim/preflight_check` is in [docs/simulation.md](../../docs/simulation.md).

## Commands

```bash
# GUI (default) or headless server with camera rendering
HEADLESS=1 CameraType=rgbd World=default ./scripts/up.sh

# Fail if a world still points at Gazebo Fuel
python3 includes/gz/scripts/check_offline_worlds.py

# Rebuild wall maps and re-vendor Fuel models
python3 includes/gz/scripts/wall_geometry.py
python3 includes/gz/scripts/fuel_assets.py

# Trajectory report (math tests do not need a running sim)
export PYTHONPATH=includes/gz/sim_ws/src/trajectory_eval
python3 -m trajectory_eval offline --ground-truth gt.tum --output /tmp/traj
```

Spawn pose `PX4_GZ_MODEL_POSE` defaults to `-3,-1.6,0.15,0,0,3.14` (ENU, radians). Pose z is drop clearance above the floor. Static TF `world` -> `spawn` uses that x, y, and yaw with z = 0.

| Path | Notes |
|------|--------|
| [sim_ws/src/sim_monitor/](sim_ws/src/sim_monitor/README.md) | Real-time factor, spawn TF, preflight, PX4 IMU relay |
| [sim_ws/src/trajectory_eval/](sim_ws/src/trajectory_eval/README.md) | Trajectory recording and scores |
| [scripts/](scripts/README.md) | Origin, x500 patches, wall export, offline models |
| [startFiles/](startFiles/README.md) | Headless SITL, bridge, sim helpers |
| [walls/](walls/README.md) | 2D obstacle JSON |

## Sensor systems

IMU, air pressure, magnetometer, and NavSat systems are added to PX4's `x500_base` model at startup. Worlds stay world-only: physics, scene, user commands, the scene broadcaster, and the rendering Sensors system. The same startup step removes those four sensor systems from the launched world and from `server.config` so each is loaded once. A second x500 in that world would load them again; multi-vehicle is out of scope.
