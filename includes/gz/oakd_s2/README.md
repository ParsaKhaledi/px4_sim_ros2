# OAK-D S2 model

`geometry.py` holds the camera. `render_oakd.py` writes:

- `includes/gz/models/OakD-Lite-stereo/model.sdf`
- `includes/gz/models/OakD-Lite-rgbd/model.sdf`
- `includes/gz/x500_tf_publisher/x500_urdf.urdf`

and, with `--patch-x500`, the include pose in PX4's `x500_depth` model. Edit the Python module, not the generated XML. The checked-in XML is the default mount (`CAM_PITCH_DEG=17`, `0.12 0.03 0.242`) so the files are readable without running the script.

```bash
# From the repo, using CAM_* in the environment (defaults if unset)
python3 includes/gz/oakd_s2/render_oakd.py
python3 includes/gz/oakd_s2/render_oakd.py --urdf-only
python3 includes/gz/oakd_s2/test_oakd_s2.py
```

At container start, `gz_modifications.bash` renders both models, copies the one selected by `CameraType`, renames that folder to `OakD-Lite`, and patches `x500_depth`. The state publisher container renders the URDF again. Both need `CAM_PITCH_DEG`, `CAM_X`, `CAM_Y`, and `CAM_Z`.

`stereo_info_relay.py` republishes `/camera/stereo/right/camera_info` as `/camera/stereo/right/camera_info_baseline`. Both cameras use the left K at the active profile resolution. The relay writes that K and `P[3] = -fx * 0.075`, and it does not change the stamp. `VISION_PROFILE` is `cpu` (default, 320x200 at 10 Hz), `full`, or `hw`. `full` and `hw` are opt-in. The PX4 container warns once when it renders the SDFs and no GPU is visible. Frames: [docs/frames.md](../../../docs/frames.md). Launch parameters: [docs/rtabmap_tuning.md](../../../docs/rtabmap_tuning.md).
