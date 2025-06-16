#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4

source /opt/ros/$ROS_DISTRO/setup.bash
source ${WORKDIR}/install/setup.bash

cd ${HOME}/volume/startFiles
export FLIGHT_HEIGHT=2
python3 microxrce_offboard.py --OffboardControllEnable True  --TakeoffHeight  $FLIGHT_HEIGHT
