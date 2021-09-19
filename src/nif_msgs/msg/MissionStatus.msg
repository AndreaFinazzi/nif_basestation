# Two-integer timestamp that is expressed as seconds and nanoseconds.
# Stamp of the last mission status update.
builtin_interfaces/Time stamp_last_update

# mission status code
uint16 MISSION_RACE = 0
uint16 MISSION_PIT_IN = 60
uint16 MISSION_PIT_OUT = 70
uint16 MISSION_SLOW_DRIVE = 128
uint16 MISSION_COMMANDED_STOP = 200
uint16 MISSION_EMERGENCY_STOP = 250
uint16 MISSION_DEFAULT = 65535

uint16 mission_status_code 65535
