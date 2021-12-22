# Telemetry: localization status message for ros2
# Simplified version of LocalizationStatus.msg
# Author : Andrea Finazzi @ KAIST
# Date : 2021.10.11


builtin_interfaces/Time stamp

geometry_msgs/PoseStamped odometry

float64 uncertainty

uint8 localization_status_code 0

# Distance from left object
float32 detected_inner_distance

# Distance from right object
float32 detected_outer_distance

# novatel_oem7_msgs/BESTPOS pos_type
uint8 pos_type_0 0
uint8 pos_type_1 0

float32 heading