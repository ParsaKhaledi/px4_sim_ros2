#!/bin/bash
# RTAB-Map entry used by compose (CameraType=rgbd|stereo).
# frame_id must be base_link (Nav2 robot_base_frame + URDF root).
# use_sim_time must match the bridge /clock and Nav2 (USE_SIM_TIME).

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4
source /opt/ros/$ROS_DISTRO/setup.bash

USE_SIM_TIME="${USE_SIM_TIME:-true}"
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
          frame_id:=base_link map_frame_id:=map vo_frame_id:=odom \
          approx_sync:=true  wait_imu_to_init:=true  approx_sync_max_interval:=0.001  \
          use_sim_time:=${USE_SIM_TIME} \
          qos:=2  rtabmapviz:=true  rviz:=false
elif [ "$CamerType" = rgbd ] || [ "$CamerType" = RGBD ] ; then
     echo "Run Rtabmap with $CamerType camera"
     ros2 launch rtabmap_launch rtabmap.launch.py   \
          args:='-d  --Optimizer/GravitySigma 0.1 --Vis/FeatureType 10 --Kp/DetectorStrategy 10 --Grid/MapFrameProjection true   \
          --NormalsSegmentation false --Grid/MaxGroundHeight 0.0  --Grid/MaxObstacleHeight 3.0 --RGBD/StartAtOrigin true         \
          --MaxFeatures 100 --Grid/RayTracing true --Grid/3D true  --Grid/FlatObstacleDetected true                            ' \
          rgb_topic:=/camera/rgb/image_raw   depth_topic:=/camera/depth/image_raw    camera_info_topic:=/camera/rgb/camera_info  \
          imu_topic:=/imu approx_sync:=true    \
          frame_id:=base_link map_frame_id:=map vo_frame_id:=odom \
          use_sim_time:=${USE_SIM_TIME} qos:=2    rtabmapviz:=true     rviz:=false   subscribe_rgbd:=false    MaxFeatures:=75
else
    echo "Invalid CameraType"
    exit 1
fi
