#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4

source /opt/ros/$ROS_DISTRO/setup.bash

cd ${HOME}/volume/includes/gz/
ros2 launch robot_state_publisher.launch.py

# Update addresses