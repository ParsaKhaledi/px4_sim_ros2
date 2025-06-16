#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4

source /opt/ros/$ROS_DISTRO/setup.bash
source ${WORKDIR}/install/setup.bash
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=False --ros-args -p \
     rviz_config:=${HOME}/volume/Params/nav2/nav2.rviz
