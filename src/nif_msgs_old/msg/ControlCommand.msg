std_msgs/Header header

#
# Priority level of the control command. The lower the number, the higher the priority.
#
uint8 order

#
# Brake pressure in Pascal [Pa] from 0 to 3447379 (res: 55158)
#
std_msgs/Float32 braking_control_cmd

#
# Accelerator percentage [%] from 0 to 100 (res: 0.1)
#
std_msgs/Float32 accelerator_control_cmd

#
# Steering wheel's angle in Degrees from -600 to 600 (res: 1)
#
std_msgs/Float32 steering_control_cmd

#
# Desired gear from -1 to 6
#
std_msgs/UInt8 gear_control_cmd

#
# Desired steer(tyre) angle in Deg
#
std_msgs/Float32 desired_steer_cmd

#
# Desired velocity in m/s
#
std_msgs/Float32 desired_velocity_cmd

#
# Desired acceleration in m/s^2
#
std_msgs/Float32 desired_accel_cmd
