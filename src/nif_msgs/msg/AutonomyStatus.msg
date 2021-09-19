# Autonomy status message for ros2
# Author : chanyoung jung @ KAIST
# Date : 2021.06.27

# driving scenario
int32 driving_scenario
int32 DRIVING_SCENARIO_URBAN = 0
int32 DRIVING_SCENARIO_RACING = 1
int32 DRIVING_SCENARIO_PARKING = 2
int32 DRIVING_SCENARIO_DEMO = 3

# actuation system status
bool longitudinal_autonomy_enabled false
bool lateral_autonomy_enabled false

bool emergency_mode_enabled false