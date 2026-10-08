#!/usr/bin/env bash
# Record a rosbag of the grading topics and copy the newest PX4 ULog.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ACTION="${1:-start}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="${2:-${ROOT}/logs/flights/${STAMP}}"
CONTAINER="${PX4_CONTAINER:-px4_sim}"
BAG_DIR="/home/px4/volume/logs/flights/current"
TOPICS_RE='^/clock$|^/fmu/out/vehicle_odometry$|^/fmu/out/vehicle_status$|^/fmu/out/vehicle_status_v1$|^/ground_truth/odom$|^/rtabmap/odom$|^/tf$|^/tf_static$|^/camera/rgb/image_raw$|^/camera/depth/image_raw$|^/camera/stereo/left/image_raw$|^/camera/stereo/right/image_raw$'

if ! docker inspect "${CONTAINER}" >/dev/null 2>&1; then
  echo "Container ${CONTAINER} is not running. Start the stack before recording." >&2
  exit 1
fi

case "${ACTION}" in
  start)
    mkdir -p "${ROOT}/logs/flights"
    docker exec "${CONTAINER}" bash -lc "rm -rf '${BAG_DIR}' && mkdir -p /home/px4/volume/logs/flights"
    docker exec -d "${CONTAINER}" bash -lc \
      "source /opt/ros/\${ROS_DISTRO}/setup.bash && source /home/px4/ws_px4/install/setup.bash && exec ros2 bag record -o '${BAG_DIR}' -e '${TOPICS_RE}'"
    echo "${OUT}" > "${ROOT}/logs/flights/active_record_path"
    echo "Recording rosbag in ${CONTAINER}:${BAG_DIR}"
    ;;
  stop)
    docker exec "${CONTAINER}" bash -lc "pkill -INT -f 'ros2 bag record' || true"
    sleep 2
    if [ -f "${ROOT}/logs/flights/active_record_path" ]; then
      OUT="$(cat "${ROOT}/logs/flights/active_record_path")"
    fi
    mkdir -p "${OUT}"
    if docker exec "${CONTAINER}" bash -lc "test -d '${BAG_DIR}'"; then
      docker cp "${CONTAINER}:${BAG_DIR}" "${OUT}/rosbag"
    else
      echo "No rosbag directory in the container." >&2
    fi
    ULOG="$(docker exec "${CONTAINER}" bash -lc \
      "ls -1t /home/px4/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg 2>/dev/null | head -1" || true)"
    if [ -n "${ULOG}" ]; then
      docker cp "${CONTAINER}:${ULOG}" "${OUT}/flight.ulg"
      echo "Copied ULog ${ULOG}"
    else
      echo "No ULog found under the SITL rootfs log directory." >&2
    fi
    echo "Flight artifacts: ${OUT}"
    ;;
  *)
    echo "usage: $0 start|stop [output_dir]" >&2
    exit 2
    ;;
esac
