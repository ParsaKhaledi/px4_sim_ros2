# px4_control

Offboard control node and `Drone` client for PX4 v1.17 over uXRCE-DDS. Architecture, frames, `ESTIMATION_MODE`, and the wall-file format are in [docs/px4_control.md](../../../docs/px4_control.md).

## Run

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch px4_control px4_control.launch.py use_sim_time:=true estimation_mode:=vision
```

`estimation_mode` is `vision` (default) or `gps`. The container start path is `includes/gz/startFiles/gz_start_px4_control.sh`.

## Example mission

With the node already running:

```bash
ros2 run px4_control out_and_back
```

That calls `takeoff(2)`, `hold(10)`, `move_forward(0.3)`, `turn(180)`, `move_forward(0.3)`, `land()`.

## Tests

```bash
cd ros2_ws/src/px4_control
PYTHONPATH=. python3 -m pytest -q test
```

Frame, covariance, trajectory, and geofence tests do not start Gazebo. `fake_vision_source` republishes a pose topic (default `/ground_truth/odom`) onto `/rtabmap/odom` and logs that it is a test substitute, not RTAB-Map. The default launch does not start it.
