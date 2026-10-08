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
     # shellcheck disable=SC1091
     source "${HOME}/volume/startFiles/rtabmap_profile.sh"
     rtabmap_profile_args stereo
     ros2 launch rtabmap_launch rtabmap.launch.py \
          args:="${RTAB_ARGS}" \
          odom_args:="${RTAB_ODOM}" \
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
     # shellcheck disable=SC1091
     source "${HOME}/volume/startFiles/rtabmap_profile.sh"
     rtabmap_profile_args rgbd-wrapper
     ros2 launch rtabmap_launch rtabmap.launch.py   \
          args:="${RTAB_ARGS}" \
          odom_args:="${RTAB_ODOM}" \
          rgb_topic:=/camera/rgb/image_raw   depth_topic:=/camera/depth/image_raw    camera_info_topic:=/camera/rgb/camera_info  \
          imu_topic:=/imu  frame_id:=base_link  approx_sync:=true  wait_imu_to_init:=true  \
          use_sim_time:=true  qos:=2    rtabmap_viz:=${RTABMAPVIZ}     rviz:=false   subscribe_rgbd:=false
else
    echo "Invalid CameraType"
    exit 1
fi
