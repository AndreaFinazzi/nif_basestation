# Acceleration control status message for ros2
# Author : Hyunki Seong @ KAIST
# Date : 2021.10.03

builtin_interfaces/Time stamp

# Acceleration control status
bool is_engine_based
bool traction_activated
bool abs_activated
float64 max_a_lon

float64 tire_slip_ratio
float64 desired_engine_torque
float64 max_engine_torque

