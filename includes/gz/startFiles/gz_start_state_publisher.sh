#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
cd /px4/volume/includes/gz/x500_tf_publisher
ros2 launch robot_state_publisher.launch.py