#!/bin/bash

USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR=/home/${USER_NAME}/ws_px4

source /opt/ros/$ROS_DISTRO/setup.bash

# startFiles is mounted on its own, so the renderer is the copy under includes/gz.
python3 "${HOME}/volume/includes/gz/oakd_s2/render_oakd.py" --urdf-only || exit 1

cd ${HOME}/volume/includes/gz/x500_tf_publisher/
ros2 launch robot_state_publisher.launch.py

# Update addresses