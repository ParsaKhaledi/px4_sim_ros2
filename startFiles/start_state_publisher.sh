#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /ws_px4_ros2/install/setup.bash
cd /px4_sim_volume/Params/owl_tf_publisher
ros2 launch robot_state_publisher.launch.py
