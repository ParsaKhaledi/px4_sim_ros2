#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /ws_px4_sim/install/setup.bash
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=True #rviz_config:=/px4_sim_volume/Params/nav2/nav2.rviz
