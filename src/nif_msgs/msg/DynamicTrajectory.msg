# DynamicTrajectory message for ros2
# Author : chanyoung jung, hyunki seong @ KAIST
# Date : 2021.12.11, 2021.12.12

std_msgs/Header header
nav_msgs/Path trajectory_path
float32[] trajectory_velocity
float32[] trajectory_timestamp_array
float32[] trajectory_global_progress

# Trajectory types
int32 trajectory_type
int32 TRAJECTORY_TYPE_PREDICTION = 0
int32 TRAJECTORY_TYPE_PLANNING = 1
int32 TRAJECTORY_TYPE_PREDICTION_BEYOND_LINE_OF_SIGHT = 16

# Collision
bool has_collision

# Longitudinal planning type
int32 longi_planning_type
int32 LONGITUDINAL_PLANNING_TYPE_STRAIGHT = 2 # Drive as fast as possible without ACC
int32 LONGITUDINAL_PLANNING_TYPE_FOLLOW = 3   # ACC activated
int32 LONGITUDINAL_PLANNING_TYPE_ESTOP = 4    # ESTOP

# Lateral planning type
int32 lat_planning_type
int32 LATERAL_PLANNING_TYPE_KEEP = 5
int32 LATERAL_PLANNING_TYPE_MERGE = 6
int32 LATERAL_PLANNING_TYPE_CHANGE_PATH = 7

# Planning_target_path
# Planning target path of the "nif_dynamic_planning_nodes"
# Added by Chanyoung (21.12.24)
int32 planning_target_path_type
int32 PLANNING_TARGET_PATH_RACELINE = 8
int32 PLANNING_TARGET_PATH_CENTER = 9
int32 PLANNING_TARGET_PATH_RIGHT = 10
int32 PLANNING_TARGET_PATH_LEFT = 11
int32 PLANNING_TARGET_PATH_CENTER_RIGHT = 12
int32 PLANNING_TARGET_PATH_CENTER_LEFT = 13
int32 PLANNING_TARGET_PATH_RACE_READY = 14
int32 PLANNING_TARGET_PATH_DEFENDER = 15
