# System health status message for ros2
# Author : chanyoung jung @ KAIST
# Date : 2021.06.27

# nodes name list
string[] node_list

# nodes status list
NodeStatusShort[] node_statuses_list

# system failure flag
bool system_failure true

# Communications status flag
bool communication_failure true

# Localization status flag
bool localization_failure true

#
bool commanded_stop false

# system status code
uint8 SYSTEM_OK = 0
uint8 SYSTEM_INITIALIZED = 64
uint8 SYSTEM_NOT_INITIALIZED = 128
uint8 SYSTEM_ERROR = 200
uint8 SYSTEM_FATAL_ERROR = 250
uint8 SYSTEM_DEAD = 255

uint8 system_status_code 0