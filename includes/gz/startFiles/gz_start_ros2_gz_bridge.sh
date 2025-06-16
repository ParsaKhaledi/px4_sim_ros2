#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4
source /opt/ros/$ROS_DISTRO/setup.bash

ros2 run ros_gz_bridge parameter_bridge --ros-args -p \
     config_file:=${HOME}/includes/gz/config_gz_bridge.yaml