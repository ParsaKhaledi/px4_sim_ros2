#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /ws_px4_ros2/install/setup.bash
ros2 launch rtabmap_launch rtabmap.launch.py   \
     args:='-d  --Optimizer/GravitySigma 0.1 --Vis/FeatureType 10 --Kp/DetectorStrategy 10 --Grid/MapFrameProjection true  \
     --NormalsSegmentation false --Grid/MaxGroundHeight 0.5  --Grid/MaxObstacleHeight 2.2 --RGBD/StartAtOrigin true   '    \
     rgb_topic:=/camera/rgb/image_raw   depth_topic:=/camera/depth/image_raw    camera_info_topic:=/camera/rgb/camera_info \
     imu_topic:=/imu approx_sync:=true    \
     use_sim_time:=true  qos:=2    rtabmapviz:=true     rviz:=false   subscribe_rgbd:=false
