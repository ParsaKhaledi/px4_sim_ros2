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
| `Vis/MinInliers` | `20` | Accept a motion only with at least this many matches. Default. `VISION_MIN_INLIERS` is the same floor for frames that are not lost. |
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
launch file's default namespace is `rtabmap`). `HealthCheck/rtabmap_health_log.py`
subscribes to that topic and scores the run from `.env`. The defaults are:

- `VISION_MAX_LOST_STREAK=3`: `lost` is not true for more than this many frames in a row
- `VISION_MAX_RECOVERY_FRAMES=2`: after each loss, the first frame with `lost` false arrives within this many frames (`Odom/ResetCountdown 1` is what makes that possible). A loss that is still open at the end of the log fails this gate.
- `VISION_MIN_MEDIAN_FEATURES`: median of `features`. Default 120. `VISION_PROFILE=cpu` uses 40 unless the variable is set.
- `VISION_MIN_INLIERS`: `inliers` on frames that are not lost (`Vis/MinInliers`). Default 20. `VISION_PROFILE=cpu` uses 15 unless the variable is set. Lost frames are in the distribution, and the streak and recovery gates cover them. The first frame after an automatic odometry reset is not lost and reports 0 inliers, because the local map was just cleared. That frame stays in the distribution and is left out of this gate.
- `VISION_MIN_ODOM_HZ=7`: rate from `odom_info` header stamps (simulation time, which PX4's EKF sees). The summary also reports the wall-clock rate and `ratio` = wall Hz / sim Hz. That ratio is not gated.

The last JSONL line is `event=summary`. Each metric has `value`, `threshold`,
and `pass`, and the line has an overall `pass`. `--fail-on-loss` or
`VISION_FAIL_ON_LOSS` exits 1 when `pass` is false. Change the numbers in
`.env` to retune CI without editing the script.

Until the first `odom_info`, tracking falls back to `/rtabmap/info` inlier
stats. Loop closures still come from `/rtabmap/info`.

```bash
ros2 topic echo /rtabmap/odom_info --field lost
ros2 topic echo /rtabmap/odom_info --field inliers
ros2 topic echo /rtabmap/odom_info --field features
python3 HealthCheck/rtabmap_health_log.py --fail-on-loss --output /tmp/rtabmap_health.jsonl
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
`Optimizer/GravitySigma` `0.1`, `Vis/MaxFeatures` `1000` on `full` and `400`
on `cpu`. If `ros2 param get`
says the parameter was not declared, the log line
`Update parameter "Odom/ResetCountdown"="1"` from the odometry node is the
check instead. Values passed only in `odom_args` will not appear on the
SLAM node, and grid keys passed only in `args` will not appear on the
odometry node.

## CPU profile

`VISION_PROFILE=full` is the OAK-D S2 spec and the default. `VISION_PROFILE=cpu`
is for Gazebo on llvmpipe (EGL headless, no GPU) with RTAB-Map in the same
box. Measured on an 8-core CPU-only machine in `apt_world` with stereo:

- full (640x400 at 30 Hz): RTF 0.125, odometry 30.3 Hz in sim time and 3.8 Hz on the wall, 64 ms per frame.
- cpu (320x200 at 10 Hz): RTF 0.43-0.46, odometry 10.0 Hz in sim time and 4.3-4.6 Hz on the wall, 25-35 ms per frame.
- cpu with `CAM_RATE_HZ=15`: RTF 0.31, odometry 15.2 Hz in sim time and 4.7 Hz on the wall.

Gazebo renders in lockstep, so the sim-time rate equals the camera rate as long as the per-frame odometry time stays below the wall-time gap between frames. On the cpu profile that gap is set by rendering: 25-35 ms of odometry fits inside it, so every camera frame is tracked and the sim-time odometry rate is the camera rate. Wall-clock 7 Hz is not reachable with CPU rendering, because Gazebo rendering alone takes about 200% CPU.

Mapping-side tweaks do not help. The SLAM node uses 5-8% CPU. In rtabmap 0.22.1 `Grid/DepthDecimation` already defaults to 4 and `Grid/RayTracing` to false.

`CAM_RATE_HZ=15` is an option for steadier tracking in motion. In one hover trial it had 0 lost frames, against 1 lost frame at 10 Hz. It is not the cpu default. The cpu default stays 10 Hz cameras and 320x200 stereo.

Set it in `.env` and recreate the PX4, StatePublisher, and Rtabmap containers.
`CAM_RATE_HZ`, `CAM_STEREO_WIDTH`, `CAM_STEREO_HEIGHT`, `CAM_COLOR_WIDTH`,
`CAM_COLOR_HEIGHT`, and `IMU_RATE_HZ` override one field and leave the rest
of the profile. The mount (`CAM_PITCH_DEG`, `CAM_X`, `CAM_Y`, `CAM_Z`) is
unchanged. `geometry.py` is still the only copy of the intrinsics: fx, fy,
cx, and cy scale with the resolution ratio, so the field of view stays, and
the right `P[3]` is `-fx_scaled * 0.075`. Both stereo cameras still share the
left K. The checked-in SDF and URDF are the full profile. Container start
re-renders them the same way it applies the mount.

What the cpu numbers cost and save:

| Setting | full | cpu | Effect |
|---|---|---|---|
| Stereo size | 640x400 | 320x200 | Quarter of the pixels. Render and bridge cost drop with the pixel count. |
| Color and depth | 640x480 | 320x240 | Same, quarter of the pixels. Depth uses the color size. |
| `CAM_RATE_HZ` | 30 | 10 | A third of the frames Gazebo has to render. |
| `IMU_RATE_HZ` | 200 | 100 | Half the IMU samples. Noise bandwidth follows the rate. |
| `Vis/MaxFeatures` | 1000 | 400 | Fewer features to extract and match. Default is 1000 (`Parameters.h`, "0 no limits"). |
| `OdomF2M/MaxSize` | 2000 | 1000 | Smaller frame-to-map local map. Default is 2000. |
| `Vis/CorGuessWinSize` | 40 | 20 | Matching window in pixels when a motion guess exists. Half the window matches half the resolution. Default on current master is 40. |
| `Vis/MinInliers` | 20 | 15 | Accept a pose with fewer matches. Small CPU effect; the health floor follows it. |
| `Odom/VisKeyFrameThr` | 150 (default, unset) | 30 | Create a keyframe when inliers drop under this count. The default 150 is above typical inliers, so every frame becomes a keyframe and frame time grows over a run (23 to 36 ms on cpu). Not yet measured live. Passed in `odom_args`. |
| `Kp/MaxFeatures` | 500 (default, unset) | 300 | Fewer words for the loop-closure vocabulary. Default is 500. |
| `Stereo/MaxDisparity` | unset | 64, stereo only | Disparity search limit in pixels. Current master defaults to 128 (float); older headers default to 64 (int). Disparity is `fx * baseline / depth`, and fx scales with width, so 64 at half resolution is the same minimum depth as 128 at full resolution. |
| `Grid/CellSize` | 0.05 (default, unset) | 0.1 | Four times fewer cells in the occupancy grid. **Nav2's costmap uses this grid, so the cells get coarser.** |
| `Grid/RangeMax` | 5 (default) | 5 | Already the default ("Maximum range from sensor. 0=inf."). Pinned so a later default cannot grow the grid. No saving against today's full profile. |
| `Rtabmap/DetectionRate` | 1 (default) | 1 | How often the SLAM node accepts an image, in Hz (`Parameters.h`). This is not the odometry rate. Stereo odometry is a separate node and is not capped by it. Pinned at the default. |

Nothing from that list was dropped. Each name is a `RTABMAP_PARAM` in
`corelib/include/rtabmap/core/Parameters.h` on current master, and the same
names are in the 0.21-era header. They are passed inside `args:=` and
`odom_args:=`. `Parameters::parseArguments` would silently ignore a key that
is not in that map. `Odom/ResetCountdown 1`, `Odom/Strategy 0` (F2M),
`Vis/FeatureType 10` and `Kp/DetectorStrategy 10` (ORB-OCTREE), and
`Vis/EstimationType 1` (PnP) stay on both profiles. Stereo stays on exact
sync. RGB-D stays on approximate sync. `wait_imu_to_init` stays.

`HealthCheck/vision_rate_probe.py` counts the image topics, `/imu`, and
`/rtabmap/odom` for N wall seconds and reads `/clock` against that window
for the real-time factor. It prints one line and writes JSONL. It does not
apply a threshold. The 7 Hz gate is `VISION_MIN_ODOM_HZ` on the health log,
using sim time. The measured wall-clock rate on CPU rendering is about 4-5 Hz.
That rate is reported beside the sim-time rate and is not gated.

The GPU compose file does not pass `VISION_PROFILE` or the `CAM_*` overrides
(it also does not pass the mount). The no-GPU compose does.
