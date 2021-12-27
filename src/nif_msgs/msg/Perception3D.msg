# Perception3D message for ros2
# Author : chanyoung jung @ KAIST
# Date : 2021.06.27

std_msgs/Header header

# object classification result
uint32 label
uint32 LABEL_PEDESTRIAN = 0
uint32 LABEL_VEHICLE    = 1
uint32 LABEL_TRUCK      = 2
uint32 LABEL_BUS        = 3
uint32 LABEL_BIKE       = 4

# tracking result
uint32 id

# detection score
float64 score

# detection result in 3d
vision_msgs/BoundingBox3D detection_result_3d

# object velocity
geometry_msgs/Twist obj_velocity_in_local # in local coordinate

# object velocity
geometry_msgs/Twist obj_velocity_in_global # in global coordinate
