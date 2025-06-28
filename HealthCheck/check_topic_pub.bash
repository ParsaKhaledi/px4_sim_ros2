#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
# Function to check the health of a ROS 2 topic
check_ros2_topic_health() {
  topic_name="$1"

  # Use ros2 topic info to get topic information
  output=$(ros2 topic info "$topic_name" 2>/dev/null) # Redirect stderr to /dev/null

  # Check if the command was successful
  if [ $? -ne 0 ]; then
    echo "Error: Failed to get topic information for '$topic_name'"
    return 1 # Return 1 to indicate failure
  fi

  # Extract the publisher count using grep and awk
  publisher_count=$(echo "$output" | grep "Publisher count:" | awk '{print $3}')

  # Debugging output: Print the raw output of ros2 topic info
  echo "DEBUG: Output of ros2 topic info:"
  echo "$output"

  # Check if publisher_count is empty or not a number
  if [[ -z "$publisher_count" || ! "$publisher_count" =~ ^[0-9]+$ ]]; then
    echo "Error: Could not parse publisher count for topic '$topic_name'"
    echo "Full output was: $output"
    return 1 # Return 1 to indicate failure
  fi

  # Check the publisher count
  if [ "$publisher_count" -gt 0 ]; then
    echo "Topic '$topic_name' has $publisher_count publishers. Healthy."
    return 0 # Return 0 to indicate healthy
  else
    echo "Topic '$topic_name' has $publisher_count publishers. Unhealthy."
    return 1 # Return 1 to indicate unhealthy
  fi
}

# Main script
if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <topic_name>"
  exit 1
fi

topic_name="$1"
if check_ros2_topic_health "$topic_name"; then
  exit 0 # Exit with healthy status
else
  exit 1 # Exit with unhealthy status
fi