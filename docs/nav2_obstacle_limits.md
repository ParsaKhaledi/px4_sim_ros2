# Nav2 obstacle limits (SLAM occupancy)

Wall keep-out and near-wall slowdown are owned by **Nav2**, using the RTAB-Map occupancy / obstacle cloud. They are not owned by the offboard JSON geofence in `px4_control`.

The `px4_control` JSON wall geofence loader is removed on this tip; do not reintroduce `wall_segments_file`.

Costmaps and keep-out only line up with the vehicle when the TF tree matches [tf_requirements.md](tf_requirements.md) (`map`→`odom`→`base_link` from RTAB-Map; no `world`→`base_link`).

## What replaces the old geofence behaviours

| Old `px4_control` behaviour | Nav2 owner | Parameters in `includes/gz/Params/nav2/nav2_params.yaml` |
|-----------------------------|------------|----------------------------------------------------------|
| A. Reject goals / straight paths near walls | Global costmap inflation + NavFn planner (+ BT `IsPathValid`) | `robot_radius: 0.35`, `inflation_radius: 0.75`, `cost_scaling_factor: 10.0`, planner `tolerance: 0.15` |
| B. Cap speed near walls | Local costmap inflation + DWB `BaseObstacle` critic | Same inflation on the local map; `BaseObstacle.scale: 1.0`, `BaseObstacle.sum_scores: True` |

Numbers match the former control keep-out: `drone_radius_m` 0.35 + `wall_margin_m` 0.40 → `inflation_radius` 0.75.

## How it works

1. RTAB-Map publishes obstacles (`/rtabmap/cloud_obstacles`) and the map.
2. Costmap **inflation** marks cells within `robot_radius` as lethal/inscribed and raises cost out to `inflation_radius`.
3. **Planner** fails (or only accepts a nearby free goal within `tolerance`) when the goal sits in that keep-out — same job as `path_rejection` / `goal_rejection`.
4. **DWB** scores each sample trajectory with `BaseObstacle` over the local costmap. High cost near walls loses; lower-speed / farther trajectories win — same job as `_speed_cap` / `max_speed_along`.

## Docs used

- Inflation layer: [Nav2 Rolling inflation](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/inflation/)
- DWB controller: [Nav2 Rolling DWB](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/dwb_controller/)
- BaseObstacle critic: [Nav2 Rolling BaseObstacle](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/dwb_controller/trajectory_critics/base_obstacle/)

For ground robots that use Regulated Pure Pursuit, Nav2 also offers `use_cost_regulated_linear_velocity_scaling`. This stack keeps DWB because the drone commands lateral `vy`.

## Tune

- Closer keep-out: lower `inflation_radius` or `robot_radius`.
- Stronger slowdown: raise `BaseObstacle.scale` or `cost_scaling_factor`.
- Stricter goal reject: lower planner `tolerance`.

Recreate the Nav2 container (or restart the launch) after editing the YAML.
