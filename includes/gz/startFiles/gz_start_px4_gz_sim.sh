#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
cd  /px4/PX4-Autopilot && \
    PX4_GZ_WORLD=$2 \
    PX4_GZ_MODEL_POSE="-3,-1.6,0,0,0,3.14" \
    make px4_sitl gz_x500_depth

# PX4_GZ_WORLD=${World} 