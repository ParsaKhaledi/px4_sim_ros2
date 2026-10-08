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
# VISION_PROFILE selects rtabmap_profiles/<profile>.ini via cfg:=.
# shellcheck disable=SC1091
source "${HOME}/volume/startFiles/rtabmap_profile.sh"
rtabmap_profile_args rgbd || {
     echo "refusing to launch RTAB-Map without a profile" >&2
     exit 1
}
ros2 launch rtabmap_launch rtabmap.launch.py   \
     cfg:="${RTAB_CFG}" \
     args:="${RTAB_ARGS}" \
     odom_args:="${RTAB_ODOM}" \
     rgb_topic:=/camera/rgb/image_raw   depth_topic:=/camera/depth/image_raw    camera_info_topic:=/camera/rgb/camera_info \
     imu_topic:=/imu  frame_id:=base_link  approx_sync:=true  wait_imu_to_init:=true  \
     use_sim_time:=true qos:=2 rtabmap_viz:=${RTABMAPVIZ} rviz:=false subscribe_rgbd:=false
