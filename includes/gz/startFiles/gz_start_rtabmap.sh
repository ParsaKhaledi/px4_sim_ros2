#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash
export CamerType=$1

if [ "$CamerType" = Stereo ] || [ "$CamerType" = stereo ]; then
     echo "Run Rtabmap with $CamerType camera"
     ros2 launch rtabmap_launch rtabmap.launch.py \
          args:="-d --Optimizer/GravitySigma 0.1 --Vis/FeatureType 10  --Kp/DetectorStrategy 10  \
          --Grid/MapFrameProjection true  --NormalsSegmentation false --Grid/MaxGroundHeight 1.0 \
          --Grid/MaxObstacleHeight 2.0 --RGBD/StartAtOrigin true --MaxFeatures 200" \
          stereo:=true  \
          left_image_topic:=/camera/stereo/left/image_raw    left_camera_info_topic:=/camera/stereo/left/camera_info    \
          right_image_topic:=/camera/stereo/right/image_raw  right_camera_info_topic:=/camera/stereo/right/camera_info   \
          imu_topic:=/imu   \
          approx_sync:=true  wait_imu_to_init:=true  approx_sync_max_interval:=0.001  \
          qos:=2  rtabmapviz:=true  rviz:=false #  frame_id:=camera_rgb_frame
elif [ "$CamerType" = rgbd ] || [ "$CamerType" = RGBD ] ; then
     echo "Run Rtabmap with $CamerType camera"
     ros2 launch rtabmap_launch rtabmap.launch.py   \
          args:='-d  --Optimizer/GravitySigma 0.1 --Vis/FeatureType 10 --Kp/DetectorStrategy 10 --Grid/MapFrameProjection true  \
          --NormalsSegmentation false --Grid/MaxGroundHeight 0.5  --Grid/MaxObstacleHeight 2.2 --RGBD/StartAtOrigin true        \
          --MaxFeatures 100 '    \
          rgb_topic:=/camera/rgb/image_raw   depth_topic:=/camera/depth/image_raw    camera_info_topic:=/camera/rgb/camera_info \
          imu_topic:=/imu approx_sync:=false    \
          use_sim_time:=true  qos:=2    rtabmapviz:=true     rviz:=false   subscribe_rgbd:=false    MaxFeatures:=75
else
    echo "Invalid CamerType"
    exit 1
fi
