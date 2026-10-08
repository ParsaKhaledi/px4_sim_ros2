#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4
source /opt/ros/$ROS_DISTRO/setup.bash

RTABMAPVIZ="${RTABMAPVIZ:-false}"
case "${RTABMAPVIZ}" in
  true|TRUE|True|1|yes|YES) RTABMAPVIZ=true ;;
  *) RTABMAPVIZ=false ;;
esac

export CamerType=$1
if [ "$CamerType" = Stereo ] || [ "$CamerType" = stereo ]; then
     echo "Run Rtabmap with $CamerType camera"
     python3 "${HOME}/volume/includes/gz/oakd_s2/stereo_info_relay.py" &
     RELAY_PID=$!
     trap 'kill ${RELAY_PID} 2>/dev/null || true' EXIT
     # Exact sync: both cameras and the relayed camera_info share one Gazebo stamp.
     # IMU is a separate subscription (wait_imu_to_init), not part of that set.
     ros2 launch rtabmap_launch rtabmap.launch.py \
          args:="-d --Optimizer/GravitySigma 0.1 --Vis/FeatureType 10 --Kp/DetectorStrategy 10 --Vis/MaxFeatures 1000 --Vis/MinInliers 20 --Grid/MapFrameProjection true --Grid/NormalsSegmentation false --Grid/MaxGroundHeight 1.0 --Grid/MaxObstacleHeight 2.0 --RGBD/StartAtOrigin true" \
          odom_args:="--Odom/Strategy 0 --Odom/ResetCountdown 1 --OdomF2M/MaxSize 2000 --Vis/CorGuessWinSize 40 --Vis/EstimationType 1" \
          stereo:=true  \
          left_image_topic:=/camera/stereo/left/image_raw    left_camera_info_topic:=/camera/stereo/left/camera_info    \
          right_image_topic:=/camera/stereo/right/image_raw  right_camera_info_topic:=/camera/stereo/right/camera_info_baseline   \
          imu_topic:=/imu  frame_id:=base_link  \
          approx_sync:=false  wait_imu_to_init:=true  \
          use_sim_time:=true \
          qos:=2  rtabmap_viz:=${RTABMAPVIZ}  rviz:=false
elif [ "$CamerType" = rgbd ] || [ "$CamerType" = RGBD ] ; then
     echo "Run Rtabmap with $CamerType camera"
     # Color and depth are two sensors, so this branch stays on approx sync.
     # The 3D grid flags stay: this branch already asked for an OctoMap-style grid.
     ros2 launch rtabmap_launch rtabmap.launch.py   \
          args:="-d --Optimizer/GravitySigma 0.1 --Vis/FeatureType 10 --Kp/DetectorStrategy 10 --Vis/MaxFeatures 1000 --Vis/MinInliers 20 --Grid/MapFrameProjection true --Grid/NormalsSegmentation false --Grid/MaxGroundHeight 0.5 --Grid/MaxObstacleHeight 2.2 --RGBD/StartAtOrigin true --Grid/RayTracing true --Grid/3D true --Grid/FlatObstacleDetected true" \
          odom_args:="--Odom/Strategy 0 --Odom/ResetCountdown 1 --OdomF2M/MaxSize 2000 --Vis/CorGuessWinSize 40 --Vis/EstimationType 1 --Vis/DepthAsMask true" \
          rgb_topic:=/camera/rgb/image_raw   depth_topic:=/camera/depth/image_raw    camera_info_topic:=/camera/rgb/camera_info  \
          imu_topic:=/imu  frame_id:=base_link  approx_sync:=true  wait_imu_to_init:=true  \
          use_sim_time:=true  qos:=2    rtabmap_viz:=${RTABMAPVIZ}     rviz:=false   subscribe_rgbd:=false
else
    echo "Invalid CameraType"
    exit 1
fi
