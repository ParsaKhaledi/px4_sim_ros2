#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4
# source /opt/ros/$ROS_DISTRO/setup.bash
source ${WORKDIR}/install/setup.bash

RTABMAPVIZ="${RTABMAPVIZ:-false}"
case "${RTABMAPVIZ}" in
  true|TRUE|True|1|yes|YES) RTABMAPVIZ=true ;;
  *) RTABMAPVIZ=false ;;
esac

# Color (IMX378) and depth (rgb_aligned_depth) are separate sensors, so exact
# sync is not used here. rtabmap_viz is the ROS 2 launch argument.
ros2 launch rtabmap_launch rtabmap.launch.py   \
     args:="-d --Optimizer/GravitySigma 0.1 --Vis/FeatureType 10 --Kp/DetectorStrategy 10 --Vis/MaxFeatures 1000 --Vis/MinInliers 20 --Grid/MapFrameProjection true --Grid/NormalsSegmentation false --Grid/MaxGroundHeight 0.5 --Grid/MaxObstacleHeight 2.2 --RGBD/StartAtOrigin true" \
     odom_args:="--Odom/Strategy 0 --Odom/ResetCountdown 1 --OdomF2M/MaxSize 2000 --Vis/CorGuessWinSize 40 --Vis/EstimationType 1 --Vis/DepthAsMask true" \
     rgb_topic:=/camera/rgb/image_raw   depth_topic:=/camera/depth/image_raw    camera_info_topic:=/camera/rgb/camera_info \
     imu_topic:=/imu  frame_id:=base_link  approx_sync:=true  wait_imu_to_init:=true  \
     use_sim_time:=true qos:=2 rtabmap_viz:=${RTABMAPVIZ} rviz:=false subscribe_rgbd:=false
