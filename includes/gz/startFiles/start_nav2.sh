#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4

source /opt/ros/$ROS_DISTRO/setup.bash
# source /ws_px4_ros2/install/setup.bash

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False --ros-args -p \
     params_file:=${HOME}/volume/Params/nav2/nav2_params.yaml
