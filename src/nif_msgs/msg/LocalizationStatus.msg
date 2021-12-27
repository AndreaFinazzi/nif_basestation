# Localization status message for ros2
# Author : Daegyu Lee @ KAIST
# Date : 2021.10.01

builtin_interfaces/Time stamp

# Localization_status
float64 bottom_error
float64 top_error
float64 diff
float64 bottom_weight
float64 top_weight
string status

float64 uncertainty

# Sensor init
bool top_initialized
bool bottom_initialized

# Localization_code
uint8 BEST_STATUS = 71
uint8 SENSOR_FUSION = 72
uint8 ONLY_TOP = 73
uint8 ONLY_BOTTOM = 74
uint8 GPS_HIGH_ERROR = 150
uint8 HEADING_ERROR = 215
uint8 UNCERTAINTY_TOO_HIGH = 225
uint8 NO_CONVERGED = 235
uint8 NO_SENSOR_INITIALIZED = 255

uint8 localization_status_code 0

uint8 pos_type_0 0
uint8 pos_type_1 0

float64 heading