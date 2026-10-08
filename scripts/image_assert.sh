#!/usr/bin/env bash
# L1 checks against an already built image. Does not start Gazebo.
set -euo pipefail

IMAGE="${1:?usage: $0 <image>}"

docker run --rm --entrypoint bash "${IMAGE}" -lc '
  set -euo pipefail
  test -x /usr/local/bin/MicroXRCEAgent || command -v MicroXRCEAgent
  command -v gz
  test -d /opt/ros/jazzy
  test -d /home/px4/PX4-Autopilot
  test "$(id -u px4)" = "1000"
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  command -v ros2
  test "${ROS_DISTRO}" = "jazzy"
'
echo "Image assert passed for ${IMAGE}"
