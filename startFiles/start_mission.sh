#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /ws_px4_ros2/install/setup.bash
cd /px4_sim_volume/
export FLIGHT_HEIGHT=2
python3 microxrce_offboard.py --OffboardControllEnable True  --TakeoffHeight  $FLIGHT_HEIGHT
