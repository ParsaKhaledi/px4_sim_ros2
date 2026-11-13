# Nav2 obstacle limits (SLAM occupancy)

Wall keep-out and near-wall slowdown are owned by **Nav2**, using the RTAB-Map occupancy / obstacle cloud. They are not owned by an offboard wall JSON geofence in `px4_control`. There is no `includes/gz/walls/` product path.

## What Nav2 owns

| Behaviour | Nav2 owner | Parameters in `includes/gz/Params/nav2/nav2_params.yaml` |
|-----------|------------|----------------------------------------------------------|
| Reject goals / paths through walls | Global costmap inflation + planner (+ BT `IsPathValid`) | `robot_radius`, `inflation_radius`, `cost_scaling_factor`, planner `tolerance` |
| Cap speed near walls | Local costmap inflation + DWB `BaseObstacle` critic | Same inflation on the local map; `BaseObstacle.scale`, `BaseObstacle.sum_scores` |

Tune keep-out with `robot_radius` and `inflation_radius`. Stronger slowdown: raise `BaseObstacle.scale` or `cost_scaling_factor`. Stricter goal reject: lower planner `tolerance`.

## How it works

1. RTAB-Map publishes obstacles (`/rtabmap/cloud_obstacles`) and the map.
2. Costmap **inflation** marks cells within `robot_radius` as lethal/inscribed and raises cost out to `inflation_radius`.
3. The **planner** fails (or only accepts a nearby free goal within `tolerance`) when the goal sits in that keep-out.
4. **DWB** scores each sample trajectory with `BaseObstacle` over the local costmap. High cost near walls loses; lower-speed / farther trajectories win.

## Docs used

- Inflation layer: [Nav2 Rolling inflation](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/inflation/)
- DWB controller: [Nav2 Rolling DWB](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/dwb_controller/)
- BaseObstacle critic: [Nav2 Rolling BaseObstacle](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/dwb_controller/trajectory_critics/base_obstacle/)

Recreate the Nav2 container (or restart the launch) after editing the YAML.
