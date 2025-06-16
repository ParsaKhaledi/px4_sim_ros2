#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4

source /opt/ros/$ROS_DISTRO/setup.bash
source /ws_px4_ros2/install/setup.bash

cd ${HOME}/volume/Params/owl_tf_publisher && \
ros2 launch robot_state_publisher.launch.py
