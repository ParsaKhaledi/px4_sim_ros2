# Estimated vehicle state in a local ENU world, plus the PX4 commander flags
# a mission script needs. Pose and twist follow ROS REP-103 (ENU, body FLU).

std_msgs/Header header

uint8 arming_state
uint8 nav_state
string nav_state_name
bool armed
bool offboard
bool pre_flight_checks_pass
bool failsafe
string failsafe_reason

geometry_msgs/Pose pose_enu
geometry_msgs/Twist twist_enu
float32 yaw_ned

bool vision_valid
uint8 vision_reset_counter
string estimation_mode
