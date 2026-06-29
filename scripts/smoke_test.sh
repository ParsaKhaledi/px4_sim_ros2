#!/bin/bash
set -euo pipefail

IMAGE="${1:?Usage: $0 <image>}"
TIMEOUT="${SMOKE_TEST_TIMEOUT:-300}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

docker rm -f px4_smoke_test >/dev/null 2>&1 || true

docker run -d \
  --name px4_smoke_test \
  --privileged \
  -e CameraType=rgbd \
  -e World=default \
  -e QT_QPA_PLATFORM=offscreen \
  -v "${REPO_ROOT}/HealthCheck:/home/px4/volume/HealthCheck" \
  -v "${REPO_ROOT}/includes/gz:/home/px4/volume/includes/gz" \
  -v "${REPO_ROOT}/includes/gz/startFiles:/home/px4/volume/startFiles" \
  "${IMAGE}" \
  /bin/bash -c "
    . /home/px4/volume/includes/gz/gz_modifications.bash rgbd && \
    MicroXRCEAgent udp4 -p 8888 & \
    /home/px4/volume/startFiles/gz_start_px4_gz_sim.sh default & \
    /home/px4/volume/startFiles/gz_start_ros2_gz_bridge.sh & \
    sleep infinity
  "

cleanup() {
  docker rm -f px4_smoke_test >/dev/null 2>&1 || true
}
trap cleanup EXIT

elapsed=0
while [ "${elapsed}" -lt "${TIMEOUT}" ]; do
  if docker exec px4_smoke_test /home/px4/volume/HealthCheck/check_multi_topic_pub.bash 2 \
      /clock /fmu/out/vehicle_odometry; then
    echo "Smoke test passed."
    exit 0
  fi
  sleep 10
  elapsed=$((elapsed + 10))
  echo "Waiting for PX4 topics... (${elapsed}s)"
done

echo "Smoke test failed: topics not healthy within ${TIMEOUT}s"
docker logs px4_smoke_test 2>&1 | tail -80
exit 1
