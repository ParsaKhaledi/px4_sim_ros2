#!/bin/bash

rm -rf /PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris
rm /PX4-Autopilot/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake

cp -r /px4_sim_volume/Params/gazebo-classic/models/iris/ /PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
cp -r /px4_sim_volume/Params/gazebo-classic/models/apt /PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
cp -r /px4_sim_volume/Params/gazebo-classic/worlds/apt.world /PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/
cp -r /px4_sim_volume/Params/gazebo-classic/sitl_targets_gazebo-classic.cmake /PX4-Autopilot/src/modules/simulation/simulator_mavlink/
