#!/bin/bash
# Publish the x500 camera frames from the statePublisher container.
# ROS_DISTRO selects the ROS setup. render_oakd.py --urdf-only also
# reads CAM_X, CAM_Y, CAM_Z, CAM_PITCH_DEG, and VISION_PROFILE.

USER_NAME=px4
HOME=/home/${USER_NAME}

source /opt/ros/$ROS_DISTRO/setup.bash

# startFiles is mounted on its own, so the renderer is the copy under includes/gz.
python3 "${HOME}/volume/includes/gz/oakd_s2/render_oakd.py" --urdf-only || exit 1

cd "${HOME}/volume/includes/gz/x500_tf_publisher" || exit 1
ros2 launch robot_state_publisher.launch.py
