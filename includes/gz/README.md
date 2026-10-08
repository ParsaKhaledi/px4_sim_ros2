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

Spawn pose `PX4_GZ_MODEL_POSE` defaults to `-3,-1.6,0,0,0,3.14` (ENU, radians) and is published as static TF `world` -> `spawn`.
