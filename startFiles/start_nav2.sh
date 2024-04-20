#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /ws_px4_ros2/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True params_file:=/px4_sim_volume/Params/nav2/nav2_params.yaml
