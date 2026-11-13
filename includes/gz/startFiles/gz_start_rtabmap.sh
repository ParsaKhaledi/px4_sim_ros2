#!/bin/bash
# RTAB-Map entry used by compose (CameraType=rgbd|stereo).
# frame_id must be base_link (Nav2 robot_base_frame + URDF root).
# use_sim_time must match the bridge /clock and Nav2 (USE_SIM_TIME).
# map_frame_id / vo_frame_id must match Nav2 global_frame / local odom (see docs/tf_requirements.md).

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4
source /opt/ros/$ROS_DISTRO/setup.bash

RTABMAPVIZ="${RTABMAPVIZ:-false}"
case "${RTABMAPVIZ}" in
  true|TRUE|True|1|yes|YES) RTABMAPVIZ=true ;;
  *) RTABMAPVIZ=false ;;
esac

USE_SIM_TIME="${USE_SIM_TIME:-true}"
export CamerType=$1
if [ "$CamerType" = Stereo ] || [ "$CamerType" = stereo ]; then
     echo "Run Rtabmap with $CamerType camera"
     python3 "${HOME}/volume/includes/gz/oakd_s2/stereo_info_relay.py" &
     RELAY_PID=$!
     trap 'kill ${RELAY_PID} 2>/dev/null || true' EXIT
     # Exact sync: both cameras and the relayed camera_info share one Gazebo stamp.
     # IMU is a separate subscription (wait_imu_to_init), not part of that set.
     # VISION_PROFILE selects the ini. Stereo and RGB-D both use it.
     # shellcheck disable=SC1091
     source "${HOME}/volume/startFiles/rtabmap_profile.sh"
     rtabmap_profile_args stereo || {
          echo "refusing to launch RTAB-Map without a profile" >&2
          exit 1
     }
     ros2 launch rtabmap_launch rtabmap.launch.py \
          cfg:="${RTAB_CFG}" \
          args:="${RTAB_ARGS}" \
          odom_args:="${RTAB_ODOM}" \
          stereo:=true  \
          left_image_topic:=/camera/stereo/left/image_raw    left_camera_info_topic:=/camera/stereo/left/camera_info    \
          right_image_topic:=/camera/stereo/right/image_raw  right_camera_info_topic:=/camera/stereo/right/camera_info_baseline   \
          imu_topic:=/imu  frame_id:=base_link  \
          map_frame_id:=map vo_frame_id:=odom \
          approx_sync:=false  wait_imu_to_init:=true  \
          use_sim_time:=${USE_SIM_TIME} \
          qos:=2  rtabmap_viz:=${RTABMAPVIZ}  rviz:=false
elif [ "$CamerType" = rgbd ] || [ "$CamerType" = RGBD ] ; then
     echo "Run Rtabmap with $CamerType camera"
     # Color and depth are two sensors, so this branch stays on approx sync.
     # The 3D grid flags stay: this branch already asked for an OctoMap-style grid.
     # shellcheck disable=SC1091
     source "${HOME}/volume/startFiles/rtabmap_profile.sh"
     rtabmap_profile_args rgbd-wrapper || {
          echo "refusing to launch RTAB-Map without a profile" >&2
          exit 1
     }
     ros2 launch rtabmap_launch rtabmap.launch.py   \
          cfg:="${RTAB_CFG}" \
          args:="${RTAB_ARGS}" \
          odom_args:="${RTAB_ODOM}" \
          rgb_topic:=/camera/rgb/image_raw   depth_topic:=/camera/depth/image_raw    camera_info_topic:=/camera/rgb/camera_info  \
          imu_topic:=/imu  frame_id:=base_link  approx_sync:=true  wait_imu_to_init:=true  \
          map_frame_id:=map vo_frame_id:=odom \
          use_sim_time:=${USE_SIM_TIME}  qos:=2    rtabmap_viz:=${RTABMAPVIZ}     rviz:=false   subscribe_rgbd:=false
else
    echo "Invalid CameraType"
    exit 1
fi
