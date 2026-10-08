# walls

2D obstacle maps, one JSON file per world. `includes/gz/scripts/wall_geometry.py` slices collision geometry with a horizontal plane and writes these files. Regenerate them after a world change. Do not edit the JSON by hand.

## Contents

| File | World |
|------|--------|
| `apt_world.json` | apartment |
| `husarion_office.json` | office |
| `husarion_world.json` | empty husarion shell |
| `sonoma_raceway.json` | raceway |
| `empty_with_plugins.json` | empty world, no segments |

## Run

```bash
python3 includes/gz/scripts/wall_geometry.py
```

The script reads `includes/gz/worlds/*.sdf` and the models next to them. It takes no environment variables. The plane is `z = 0.5` m.

## Test

```bash
python3 -m pytest includes/gz/scripts/test_wall_geometry.py
```

## Environment

None. Spawn pose is not applied here.

## Topics

None. This is a file export.

## Outputs

Each file is one document:

```json
{
  "world": "apt_world",
  "frame": "world_enu",
  "units": "m",
  "slice_z_m": 0.5,
  "min_height_m": 0.25,
  "min_segment_m": 0.05,
  "description": "2D line segments where collision geometry crosses slice_z_m.",
  "segment_count": 0,
  "unresolved_uris": [],
  "segments": [
    {"start": [0.0, 0.0], "end": [1.0, 0.0], "source": "model/link/collision", "kind": "box"}
  ]
}
```

`start` and `end` are `[x, y]` in metres. `kind` is `box`, `cylinder`, or `mesh`. Shapes shorter than `min_height_m` are skipped (floors). Segments shorter than `min_segment_m` are dropped. `unresolved_uris` lists meshes that were not on disk.

`frame` is `world_enu` or `local`.

* `world_enu` is the Gazebo world: x east, y north. Every file this script writes uses `world_enu`.
* `local` is the spawn frame: the same plane, shifted and yawed so the spawn is the origin.

A consumer that wants obstacles relative to the vehicle should transform `world_enu` points with the static TF `world` → `spawn` from `sim_monitor.spawn_frame`. That frame is x, y, and yaw of the spawn, with z = 0. Use that TF. Parsing `PX4_GZ_MODEL_POSE` drops the rule that pose z is drop clearance and that roll and pitch are not part of the spawn frame.
