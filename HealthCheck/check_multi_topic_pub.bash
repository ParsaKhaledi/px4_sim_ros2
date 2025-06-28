#!/bin/bash
#
# combined_topics_health.sh
#
# This script uses a loop to check the health of a specified number
# of ROS 2 topics. The first argument is the number of topics to check,
# followed by one or more topic names.
#
# Only the first N topics (where N is the number provided as the first argument)
# will be evaluated. The script declares the overall health as healthy (exit code 0)
# only if all N topics are healthy.
#
USER_NAME=px4
HOME=/home/${USER_NAME}
WORKDIR_HEALTHCHECK=/home/${USER_NAME}/volume/HealthCheck
source /opt/ros/$ROS_DISTRO/setup.bash


# Usage:
#   ./combined_topics_health.sh <num_topics> <topic1> <topic2> [<topic3> ...]
# For example, to check three topics:
#   ./combined_topics_health.sh 3 /scan /rtabmap/odom /another/topic

# Validate that at least 2 arguments are provided.
if [ "$#" -lt 2 ]; then
  echo "Usage: $0 <num_topics> <topic1> <topic2> [<topic3> ...]"
  exit 1
fi

# Get the number of topics to check and shift the arguments.
NUM_TOPICS="$1"
shift

# Now $# represents the number of topic parameters provided.
if [ "$#" -lt "$NUM_TOPICS" ]; then
  echo "Error: Expected at least $NUM_TOPICS topics, but received $# topics."
  exit 1
fi

# Source the ROS2 environment so the ros2 command is available.
source /opt/ros/${ROS_DISTRO}/setup.bash

# Initialize overall health indicator to "healthy" (0).
all_healthy=0

# Place the provided topics into an array.
topics=("$@")

# Loop over the first NUM_TOPICS topics.
for (( i=0; i<NUM_TOPICS; i++ )); do
  topic="${topics[$i]}"
  echo "Checking health for topic: $topic"
  
  # Call your existing health check script.
  ${WORKDIR_HEALTHCHECK}/check_topic_pub.bash "$topic"
  ret=$?
  
  if [ "$ret" -ne 0 ]; then
    echo "Topic '$topic' is unhealthy."
    all_healthy=1
  else
    echo "Topic '$topic' is healthy."
  fi
done

# Final decision using AND logic: only healthy if every topic is healthy.
if [ "$all_healthy" -eq 0 ]; then
  echo "All specified topics are healthy."
  exit 0
else
  echo "One or more specified topics are unhealthy."
  exit 1
fi
