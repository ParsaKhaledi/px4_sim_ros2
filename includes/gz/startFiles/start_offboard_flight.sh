#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4

source /opt/ros/$ROS_DISTRO/setup.bash
source ${WORKDIR}/install/setup.bash

# microxrce_offboard.py and px4_imu_bridge.py live in includes/gz (bind-mounted
# here), not in startFiles.
cd ${HOME}/volume/includes/gz

export FLIGHT_HEIGHT=${FLIGHT_HEIGHT:-2}

python3 px4_imu_bridge.py &

python3 microxrce_offboard.py --OffboardControllEnable True --TakeoffHeight "${FLIGHT_HEIGHT}"
