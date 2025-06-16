#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4

source /opt/ros/$ROS_DISTRO/setup.bash
source ${WORKDIR}/install/setup.bash

ros2 run explore_lite explore --ros-args \
     --params-file ${HOME}/volume/includes/gz/m-explore_params.yaml
# m-explore_params.yaml not included here