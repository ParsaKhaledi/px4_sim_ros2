#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4
# source /opt/ros/$ROS_DISTRO/setup.bash
source ${WORKDIR}/install/setup.bash

# Headless CI leaves the viewer off. Set RTABMAPVIZ=true when a display is up.
RTABMAPVIZ="${RTABMAPVIZ:-false}"
case "${RTABMAPVIZ}" in
  true|TRUE|True|1|yes|YES) RTABMAPVIZ=true ;;
  *) RTABMAPVIZ=false ;;
esac

# Right camera_info from Gazebo has Tx = 0 unless <projection><tx> is honored.
# The relay publishes P[3] = -fx * 0.075 either way, and it keeps header.stamp.
# Both cameras are stamped from the same Gazebo step, so exact sync matches
# the pair plus both camera_info topics. The IMU is not in that synchronizer:
# rtabmap_launch subscribes to it on its own, and wait_imu_to_init holds
# odometry until the first sample. A 200 Hz IMU does not share the 30 Hz stamp.
python3 "${HOME}/volume/includes/gz/oakd_s2/stereo_info_relay.py" &
RELAY_PID=$!
trap 'kill ${RELAY_PID} 2>/dev/null || true' EXIT

# rtabmap.launch.py (ROS 2) names the viewer rtabmap_viz. rtabmapviz is not
# a declared argument. Odom/ResetCountdown 0 never restarts a lost odometry.
# VISION_PROFILE=cpu selects the lighter set in rtabmap_profile.sh.
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
