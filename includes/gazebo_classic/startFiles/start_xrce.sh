#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /ws_px4_ros2/install/setup.bash
MicroXRCEAgent udp4 -p 8888
