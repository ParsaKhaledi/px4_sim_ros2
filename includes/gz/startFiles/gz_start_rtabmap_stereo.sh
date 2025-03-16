#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
source /ws_px4_ros2/install/setup.bash
ros2 launch rtabmap_launch rtabmap.launch.py \
     args:="-d --Optimizer/GravitySigma 0.1 --Vis/FeatureType 10  --Kp/DetectorStrategy 10  \
     --Grid/MapFrameProjection true  --NormalsSegmentation false --Grid/MaxGroundHeight 1.0 \
     --Grid/MaxObstacleHeight 2.0 --RGBD/StartAtOrigin true" \
     stereo:=true  \
    left_image_topic:=/camera/stereo/left/image_raw    left_camera_info_topic:=/camera/stereo/left/camera_info    \
    right_image_topic:=/camera/stereo/right/image_raw  right_camera_info_topic:=/camera/stereo/right/camera_info   \
    imu_topic:=/imu  frame_id:=oak-d-base-frame  \
    approx_sync:=true  wait_imu_to_init:=true  approx_sync_max_interval:=0.001  \
    qos:=2  rtabmapviz:=true  rviz:=false
