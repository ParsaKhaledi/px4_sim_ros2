# Coordinate frames in Gazebo, ROS 2 and PX4

This is the frame guide for the x500 plus OAK-D S2 simulation. It replaces the
notes that described the mismatches in the old OAK-D Lite model. The geometry
those notes asked for now comes from one module,
[`includes/gz/oakd_s2/geometry.py`](../includes/gz/oakd_s2/geometry.py).

## 1. The three conventions

| World | Body | Who uses it |
|---|---|---|
| **ENU**: x East, y North, z Up | **FLU**: x Forward, y Left, z Up | ROS 2 (REP-103/105), Gazebo, RTAB-Map, Nav2 |
| **NED**: x North, y East, z Down | **FRD**: x Forward, y Right, z Down | PX4 (EKF2, `/fmu/in/*`, `/fmu/out/*`) |
| — | **Optical**: z Forward (into the scene), x Right, y Down | Every camera image and `camera_info` |

All three are right-handed, so a conversion is always a rotation.

### Conversions you actually need

- **Position or velocity, world:** ENU to NED is `(x, y, z)` to `(y, x, -z)`.
- **Vector, body:** FLU to FRD is `(x, y, z)` to `(x, -y, -z)` (180 degrees about x).
- **Orientation:** `q_NED_FRD = q_NED_ENU ⊗ q_ENU_FLU ⊗ q_FLU_FRD`. Do not flip quaternion components by hand.
- **Covariance:** a 3x3 covariance rotates as `R Σ Rᵀ`.
- **Body link to optical frame:** `rpy = (-π/2, 0, -π/2)`. Gazebo looks along the sensor's +x, with +y left and +z up. That rotation makes optical +z the viewing direction, optical +x image-right, and optical +y image-down.

Positive pitch about y points the glass down. `Ry(+pitch)` sends +x toward -z, and z is up. `CAM_PITCH_DEG=17` is `+0.296706` rad in both the URDF and PX4's include pose.

## 2. How the frames chain together

```
Gazebo world (ENU)
  └─ x500_depth (FLU)
       └─ camera_link   pose from x500_depth's OakD-Lite include
            ├─ OV9282_left / OV9282_right     (stereo model)
            ├─ IMX378 and rgb_aligned_depth   (RGB-D model, same pose)
            └─ BNO086

ROS 2 TF (robot_state_publisher reads the generated x500_urdf.urdf)
  map → odom (RTAB-Map) → base_link
        └─ camera_link
             ├─ stereo_left_camera_frame  → stereo_left_camera_optical_frame
             ├─ stereo_right_camera_frame → stereo_right_camera_optical_frame
             ├─ camera_rgb_frame          → camera_rgb_optical_frame
             └─ imu_link
```

The Gazebo link is named `camera_link` because that is the child of `CameraJoint` in PX4's `x500_depth` model. The folder is still called `OakD-Lite` after startup so `model://OakD-Lite` keeps resolving. Image `frame_id` values are the optical frames, set both as `gz_frame_id` and as `<optical_frame_id>` (Harmonic stamps the optical id on the image and on `camera_info`).

RTAB-Map's `frame_id` is `base_link`. The old stereo launch used `oak-d-base-frame`, which was never in the URDF.

Stereo uses exact sync (`approx_sync:=false`). Gazebo gives both OV9282 sensors, and the `camera_info` published with each image, the simulation time of the step that rendered them. The baseline relay copies that stamp onto `/camera/stereo/right/camera_info_baseline`. The four stereo topics therefore match. The IMU is not in that set. `rtabmap_launch` remaps `imu` on its own and `wait_imu_to_init:=true` waits for the first sample before odometry starts. At 200 Hz (`full` and `hw`) the IMU does not land on the image stamps, so it is not put through `approx_sync`. `VISION_PROFILE=cpu` lowers those to 100 Hz and 10 Hz; the stamps still do not match, and the IMU stays out of the synchronizer. `hw` images are 15 Hz.

Depth is aligned to color the way a real OAK-D publishes it: the depth sensor uses the color camera's pose, intrinsics, and `camera_rgb_optical_frame`. There is no separate depth frame.

## 3. What was wrong, and what the sim does now

| # | What | Before | Now |
|---|---|---|---|
| 1 | Stereo `frame_id` | no `gz_frame_id` | `stereo_*_camera_optical_frame`, and those links are in the URDF |
| 2 | Baseline | SDF y = ±0.037, URDF y = ±0.05 | both y = ±0.0375 (7.5 cm) |
| 3 | Right `camera_info` Tx | two independent cameras, Tx usually 0, and the two Ks differed | both cameras render the left K. Tx = `-fx * 0.075` with that fx. The relay writes the same K and P |
| 4 | Pitch | 0.3 rad on the sensors only; URDF sensors had no pitch | pitch is the mount (`CAM_PITCH_DEG`, default 17). Sensors stay fixed in the housing |
| 5 | Mount on the drone | URDF `camera_joint` rpy `-1.85 0 0` | translation `0.12 0.03 0.242` and pitch only, matching PX4's include. The roll was not in the SDF |
| 6 | Stereo base frame | `oak-d-base-frame` | `base_link` |
| 7 | IMU | topic `imu/data`, not bridged, URDF rpy `1.57 3.14 1.57`, orientation relative to the spawn pose | absolute topic `/imu`, frame `imu_link`, orientation reference ENU so a pitched mount is not reported as level |
| 8 | Sim time | RGB-D launch set `use_sim_time:=false` | both RTAB-Map launches set `use_sim_time:=true` |

Items 2, 4 and 5 do not crash anything. They bend the map, and that bent pose is what EKF2 was integrating.

The mount is not stored inside the camera SDF. PX4 includes the camera at a pose, and `gz_modifications.bash` rewrites that include (and `CameraJoint`) from the same `CAM_*` values that render the URDF. Sensor xyz in the SDF is the pose in `camera_link`, and the matching URDF joint is the same xyz with zero rotation. The optical joint is only in the URDF, because Gazebo does not have a separate optical link. It only stamps the optical frame name.

## 4. How to check a live run

```bash
ros2 topic echo --once /camera/stereo/left/image_raw --field header
ros2 topic echo --once /camera/stereo/right/camera_info_baseline --field p
# p[3] / p[0] is -0.075. At the 640x400 calibration, p[3] is about -30.497
# (fx 406.624 * 0.075). full and hw at 1280x800 double fx, so p[3] is about
# -60.994. Right K matches left K.

ros2 run tf2_ros tf2_echo base_link stereo_left_camera_optical_frame
ros2 run tf2_ros tf2_echo stereo_left_camera_optical_frame stereo_right_camera_optical_frame
# translation x is +0.075

gz topic -l | grep -E 'imu|camera'
```

`HealthCheck/check_vision_pipeline.py` checks those same things and writes PASS/FAIL JSONL. It was not run here; this environment does not have the simulator.

A tilted or curved wall in the RTAB-Map cloud is a rotation error. A wall at the wrong distance is the baseline or the intrinsics.

## 5. OAK-D S2 numbers used in the sim

| Item | Value | Source |
|---|---|---|
| Housing | 97 × 29.5 × 22.9 mm, 91 g | Luxonis datasheet and shop page |
| Stereo | 2× OV9282, 1280×800 native, global shutter. `full` and `hw` run at 1280×800. `cpu` runs at 320×200. The calibration file is 640×400 | Luxonis OAK-D S2 docs |
| Baseline | 7.5 cm, not configurable | Shop page, older manual, `depthai-ros` xacro (`baseline = 0.075`). The current docs page says 75 cm; that is a typo |
| Stereo intrinsics | left fx 406.6239, fy 406.0800, cx 305.7769, cy 204.5298, used for both cameras | `oak-s2/left.yaml`. The right yaml differs by about a pixel and is not rendered |
| Stereo HFOV | about 76.4 deg, from that fx | Datasheet nominal HFOV is 80 deg. The sim follows the calibration, with `scale_to_hfov` off |
| Color | IMX378 auto-focus, HFOV 66 deg, 640×480, square pixels | Datasheet "center color camera" table. Fixed-focus would be 69 deg and was not used |
| Color VFOV / DFOV | about 52 deg / 78 deg implied | Datasheet lists 54 / 78. 66 and 54 deg are not one pinhole on 4:3; HFOV is what the model is pinned to |
| Depth | near 0.2 m, far 12 m, aligned to color | Ideal range about 0.8–12 m, MinZ about 0.2 m at 400p with extended disparity |
| IMU | BNO086, 200 Hz, orientation reference ENU | Luxonis docs. gz-sim8 otherwise uses the spawn pose as the reference. Noise is a BMI270-class stand-in (see `geometry.py`) |
| Rates | `full` cameras 30 Hz and IMU 200 Hz. `hw` cameras 15 Hz and IMU 200 Hz. `cpu` cameras 10 Hz and IMU 100 Hz | `full` keeps the previous image rate at the native stereo size. `hw` is the onboard rate. `cpu` is the CPU-only sim rate |
| Image noise | Gaussian stddev 0.007 | Normalized intensity, about two counts on an 8-bit image |

Real distortion coefficients are in the calibration yaml and are **not** applied. The render is a pinhole and `camera_info` D is zero, so the image and the calibration agree. Putting the real k1/k2 on a pinhole render would make RTAB-Map undistort a picture that was never distorted.

Sensor origins in `camera_link` (x forward, y left, z up, origin at the housing center):

- left mono: `(0.01145, +0.0375, 0)`
- right mono: `(0.01145, -0.0375, 0)`
- color and aligned depth: `(0.01145, 0, 0)`
- IMU: `(0, 0, 0)`, axes parallel to the housing

`0.01145` is half the 22.9 mm depth, so the lenses sit on the front glass. `depthai_descriptions` puts all three cameras on the base origin and splits them only in y; the extra x is the glass. The public URDF does not enable an IMU for the S2, so there is no PCB offset to copy. The IMU frame is the housing center.

Right-camera Tx uses the shared left fx: `-406.6239117362143 * 0.075 ≈ -30.497`.

Why the launches pass the parameters they pass is in [rtabmap_tuning.md](rtabmap_tuning.md).

## 6. Changing the mount

In `.env`:

```bash
CAM_PITCH_DEG=17
CAM_X=0.12
CAM_Y=0.03
CAM_Z=0.242
```

`scripts/up.sh` exports these into Compose. The PX4 container renders both SDF variants and patches `x500_depth`. The state publisher container renders the URDF again from the same variables. Recreate both containers after a change. The baseline does not have an environment variable; it is fixed hardware.

`RTABMAPVIZ` defaults to `false`. Set it to `true` only when a display is available.

## 7. Still open

- Stereo and RGB-D are separate models. A running stack has one of them, selected by `CameraType`.
- The GPU compose file was not given these environment lines. The active file is `docker-compose-px4.yml`.
- This tree was not flown. `HealthCheck/check_vision_pipeline.py` is the check to run once Gazebo is up.
