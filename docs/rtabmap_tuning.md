# RTAB-Map parameters for this sim

The three scripts under `includes/gz/startFiles/gz_start_rtabmap*.sh` pass these
on purpose. Names were checked against `corelib/include/rtabmap/core/Parameters.h`
on the current rtabmap master (the same keys Jazzy and Humble ship) and against
`rtabmap_launch/launch/rtabmap.launch.py` on the `ros2` branch. This machine
does not have the image installed, so the launch-argument name was not read
out of a container.

`Parameters::parseArguments` looks the key up in the default map. A key that
is not there, and is not an old name in `getRemovedParameters()`, is dropped
with no warning. (`deserialize()` does warn. The argument parser does not.)
The odometry node then keeps only keys that belong to odometry and logs
`Ignored parameter` for the rest. That is why `--NormalsSegmentation` and
`--MaxFeatures` never did anything: the real keys are `Grid/NormalsSegmentation`
and `Vis/MaxFeatures`. `Odom/MaxFeatures` is an old name that migrates to
`Vis/MaxFeatures`; bare `MaxFeatures` is not.

`MaxFeatures:=75` is not an RTAB-Map key at all. It is an undeclared launch
argument. ROS 2 rejects those, so that token could fail the RGB-D launch
rather than set a feature cap. It is gone.

The viewer argument in that launch file is `rtabmap_viz` and it defaults to
true. `rtabmapviz` is not declared, so `RTABMAPVIZ=false` never reached the
node. The scripts now pass `rtabmap_viz:=${RTABMAPVIZ}`.

## What each flag does

| Argument | Value | Why |
|---|---|---|
| `Optimizer/GravitySigma` | `0.1` | Graph optimization keeps poses consistent with gravity. `0` turns that off. The default in current master is `0.3`; older builds default to `0`. Safe to leave at `0.1` only because the IMU orientation reference is ENU (see below). |
| `Vis/FeatureType` | `10` | ORB-OCTREE for odometry features. Same code as `Kp/DetectorStrategy` 10, which is the loop-closure detector. |
| `Kp/DetectorStrategy` | `10` | ORB-OCTREE for the vocabulary. Needs a build with the ORB octree; without it RTAB-Map will not extract those features. |
| `Vis/MaxFeatures` | `1000` | Cap on features per frame. This is the default. The old `--MaxFeatures` was not this key. |
| `Vis/MinInliers` | `20` | Accept a motion only with at least this many matches. Default. The health log uses the same number. |
| `Grid/MapFrameProjection` | `true` | Project the occupancy cloud in the map frame. |
| `Grid/NormalsSegmentation` | `false` | Do not label ground from point normals. The grid then uses the height passthrough (`Grid/MaxGroundHeight`, `Grid/MaxObstacleHeight`). **Nav2's costmap is built from this grid, so this changes which cells are ground.** The old `--NormalsSegmentation` was ignored, so the default (`true`) was what actually ran. |
| `Grid/MaxGroundHeight` | stereo `1.0`, RGB-D `0.5` | Metres. Used because normals segmentation is off. The description says to set this in that case. |
| `Grid/MaxObstacleHeight` | stereo `2.0`, RGB-D `2.2` | Metres. Points above this are not obstacles. |
| `Grid/RayTracing`, `Grid/3D`, `Grid/FlatObstacleDetected` | wrapper RGB-D branch only | That branch already asked for a 3D grid with ray tracing. `Grid/3D` must be true or 3D ray tracing is ignored. They were kept. The dedicated RGB-D script does not set them. |
| `RGBD/StartAtOrigin` | `true` | In localization mode, start at the map origin instead of the last saved pose. |
| `Odom/Strategy` | `0` | Frame-to-map visual odometry. Default. Passed in `odom_args` so it reaches the odometry node. Putting `Odom/*` only in `args` has crashed the SLAM node when that parameter was not declared there (`rtabmap_ros` issue 1304). |
| `Odom/ResetCountdown` | `1` | After this many frames where odometry cannot be computed, reset and continue from the last good pose with a large covariance. `0` disables the reset, which is the default, so a lost odometry stays lost. The wiki's Kinect mapping note describes the same switch. |
| `OdomF2M/MaxSize` | `2000` | Local map word cap for frame-to-map. Default. |
| `Vis/CorGuessWinSize` | `40` | Pixel window for matching when a motion guess exists. Default. The guess here is the IMU. |
| `Vis/EstimationType` | `1` | 3D to 2D (PnP). Default. `0` is 3D-3D, `2` is epipolar. |
| `Vis/DepthAsMask` | `true`, RGB-D `odom_args` only | Ignore features where the depth image has no return. Default, and only meaningful for RGB-D. |

`-d` deletes the previous database on start.

## Sync and the IMU

Stereo uses `approx_sync:=false`. Both OV9282s and their `camera_info` are
stamped with one simulation step, and the baseline relay copies that stamp.
See [frames.md](frames.md).

RGB-D stays on `approx_sync:=true` with no max interval (the launch default
`0` means no finite window). Color is the IMX378 camera sensor and depth is
a second sensor, `rgb_aligned_depth`. They are updated in the same render
pass when both are due, but they are not one sensor update, and this tree
has not been flown to compare the stamps. A `0.001` s window is shorter than
a 30 Hz frame, so it is not used.

`wait_imu_to_init:=true` is on every script. The IMU is not inside the image
synchronizer. The odometry node subscribes to `imu` on its own and waits for
a first sample. At 200 Hz it does not share the 30 Hz stamp.

`Optimizer/GravitySigma 0.1` is only useful if that orientation is in the
world frame. gz-sim8 (`src/systems/imu/Imu.cc`) calls
`SetOrientationReference` with the sensor's spawn rotation, then calls
`SetWorldFrameOrientation` only when `<orientation_reference_frame>` is
present. gz-sensors8 reads the `<localization>` child (`ENU`, `NED`, `NWU`,
or `CUSTOM`) and, for `ENU`, replaces the spawn reference with the world
frame. Gazebo's world and ROS REP-103 are ENU, which is what RTAB-Map
consumes. `NED` would be the PX4 body convention, not this IMU message.
The SDF sets `<localization>ENU</localization>` on the BNO086.

## Rectified stereo

Both cameras render the left calibration (fx 406.6239). The right yaml is
about one pixel off. RTAB-Map expects a rectified pair (`rtabmap_ros` issue
1375): one K, and the baseline only in the right `P[3]`. The relay writes

- `K` = left calibration
- `P` = that K, with `P[3] = -fx * 0.075` on the right topic and `0` on the left image's own `camera_info`

The 7.5 cm baseline is unchanged. The left topic is still Gazebo's
`/camera/stereo/left/camera_info`. The right topic RTAB-Map subscribes to is
`/camera/stereo/right/camera_info_baseline`.

## Checking a run

Odometry health is `/rtabmap/odom_info` (`rtabmap_msgs/msg/OdomInfo`; the
launch file's default namespace is `rtabmap`). A flight is in good shape when:

- `lost` is not true for more than 3 frames in a row
- after a loss, `lost` is false again within 2 frames (`Odom/ResetCountdown 1` is what makes that possible)
- the median of `features` is at least 500
- `inliers` stays at least 20 (`Vis/MinInliers`)

```bash
ros2 topic echo /rtabmap/odom_info --field lost
ros2 topic echo /rtabmap/odom_info --field inliers
ros2 topic echo /rtabmap/odom_info --field features
```

The same keys are declared on the nodes when the installed `rtabmap_ros`
includes the parameter-declaration fix (issue 525). Read them back with:

```bash
ros2 param get /rtabmap/stereo_odometry Odom/ResetCountdown
ros2 param get /rtabmap/rgbd_odometry Odom/ResetCountdown
ros2 param get /rtabmap/rtabmap Grid/NormalsSegmentation
ros2 param get /rtabmap/rtabmap Optimizer/GravitySigma
ros2 param get /rtabmap/rtabmap Vis/MaxFeatures
```

`ResetCountdown` should be `1`, `Grid/NormalsSegmentation` `false`,
`Optimizer/GravitySigma` `0.1`, `Vis/MaxFeatures` `1000`. If `ros2 param get`
says the parameter was not declared, the log line
`Update parameter "Odom/ResetCountdown"="1"` from the odometry node is the
check instead. Values passed only in `odom_args` will not appear on the
SLAM node, and grid keys passed only in `args` will not appear on the
odometry node.
