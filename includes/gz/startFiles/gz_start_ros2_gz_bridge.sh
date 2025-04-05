#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/px4/volume/includes/gz/config_gz_bridge.yaml