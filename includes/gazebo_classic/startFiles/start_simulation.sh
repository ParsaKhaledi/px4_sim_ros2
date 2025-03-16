#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /ws_px4_ros2/install/setup.bash
cd /PX4-Autopilot
make px4_sitl gazebo-classic_iris__apt
