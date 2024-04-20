#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /ws_px4_ros2/install/setup.bash
ros2 run explore_lite explore --ros-args --params-file /px4_sim_volume/Params/m-explore_params.yaml

