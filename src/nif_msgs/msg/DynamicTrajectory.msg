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

# Planning_decision_type
# NOTE : should match with the "dynamic_planning_node.h"
int32 planning_decision_type
int32 PLANNING_DECISION_TYPE_STRAIGHT = 2
int32 PLANNING_DECISION_TYPE_FOLLOW = 3
int32 PLANNING_DECISION_TYPE_RIGHT = 4
int32 PLANNING_DECISION_TYPE_LEFT = 5
int32 PLANNING_DECISION_TYPE_ESTOP = 6

# Planning_stage_type
# NOTE : should match with the "dynamic_planning_node.h"
int32 planning_action_type
int32 PLANNING_ACTION_DRIVING = 7
int32 PLANNING_ACTION_START_OVERTAKING = 8
int32 PLANNING_ACTION_SIDE_BY_SIDE = 9
int32 PLANNING_ACTION_FINISH_OVERTAKING = 10
int32 PLANNING_ACTION_ABORT_OVERTAKING = 11
