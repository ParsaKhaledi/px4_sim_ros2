#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4

source /opt/ros/$ROS_DISTRO/setup.bash


cd ${HOME}/PX4-Autopilot && \
    PX4_GZ_WORLD=$1 \
    PX4_GZ_MODEL_POSE="-3,-1.6,0,0,0,3.14" \
    MAVLINK_UDP_URI=udp://qgc:14550 \
        make px4_sitl gz_x500_depth 
# PX4_GZ_WORLD=${World} 