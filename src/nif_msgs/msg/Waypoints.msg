# Waypoint status message for ros2
# Author : chanyoung jung @ KAIST
# Date : 2021.06.27

std_msgs/Header header

int32 current_wpt_idx

# same as local map track
nav_msgs/Path truncated_track_in_body   # length and sampling distance should be predefined in the yaml file
nav_msgs/Path truncated_track_in_global # length and sampling distance should be predefined in the yaml file