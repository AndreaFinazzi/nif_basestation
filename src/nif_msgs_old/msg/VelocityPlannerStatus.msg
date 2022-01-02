# Velocity planner status message for ros2
# Author : Hyunki Seong @ KAIST
# Date : 2021.10.03

builtin_interfaces/Time stamp

# Velocity planner status
bool valid_path
bool valid_odom

float64 max_vel_mps
float64 vel_curv_mps
float64 vel_dyn_mps

float64 curvature
float64 a_lat_max
float64 a_lat_kf

float64 error_gain
float64 safe_dist_gain
float64 safe_dist

float64 a_lon_kf
float64 yaw_rate_kf