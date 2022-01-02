# Autonomy status message for ros2
# Author : chanyoung jung @ KAIST
# Date : 2021.06.27

std_msgs/Header header

# driving scenario
int32 driving_scenario
int32 DRIVING_SCENARIO_URBAN = 0
int32 DRIVING_SCENARIO_RACING = 1
int32 DRIVING_SCENARIO_PARKING = 2
int32 DRIVING_SCENARIO_DEMO = 3

# overall autonomy status
bool is_full_autonomy_on # when is_longi_accuation_power_on and is_lateral_accuation_power_on are true

# accuation system status
bool is_longi_accuation_power_on
bool is_lateral_accuation_power_on
