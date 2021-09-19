# Powertrain status message for ros2
# Author : chanyoung jung @ KAIST
# Date : 2021.06.27

std_msgs/Header header

# engine
bool is_engine_on
bool is_engine_idle
float32 engine_rpm
float32 engine_torque

# gear box
int32 gear_type
int32 current_gear
bool is_cluch_engaged
bool is_gear_in_reverse
bool is_gear_in_drive
bool is_gear_in_park
int32 GEAR_TYPE_AUTO = 0
int32 GEAR_TYPE_MANUAL = 1

# wheel speed
float32 vehicle_speed_mps
float32 vehicle_speed_kph
float32 fl_speed_kph
float32 fl_speed_mps
float32 fr_speed_kph
float32 fr_speed_mps
float32 rl_speed_kph
float32 rl_speed_mps
float32 rr_speed_kph
float32 rr_speed_mps

# steering
float32 current_steer_wheel_angle_deg
float32 current_steer_wheel_angle_rad
float32 current_wheel_angle_deg
float32 current_wheel_angle_rad
float32 steering_wheel_angle_velocity_rps
float32 wheel_angle_velocity_rps

# vehicle acceleration
float32 current_acceleration_mpss

# pedal
bool is_accel_pedal_pressed
bool is_brake_pedal_pressed
float32 current_accel_pedal_value
float32 current_brake_pedal_value

# body
int32 lamp
int32 LAMP_LEFT = 0
int32 LAMP_RIGHT = 1
int32 LAMP_HAZRD = 2






