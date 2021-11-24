# Two-integer timestamp that is expressed as seconds and nanoseconds.
# Stamp of the last mission status update.
builtin_interfaces/Time stamp_last_update

uint8 track_flag 0
uint8 veh_flag 0

uint8 lap_count 0
uint16 lap_distance 0

# mission status code
uint16 MISSION_RACE = 0
uint16 MISSION_STANDBY = 50
uint16 MISSION_PIT_IN = 60
uint16 MISSION_PIT_STANDBY = 65
uint16 MISSION_PIT_OUT = 70
uint16 MISSION_PIT_TO_TRACK = 75
uint16 MISSION_SLOW_DRIVE = 128
uint16 MISSION_COMMANDED_STOP = 200
uint16 MISSION_EMERGENCY_STOP = 250
uint16 MISSION_COLLISION_AVOIDNACE = 400
uint16 MISSION_TIRE_WARMUP = 500
uint16 MISSION_TEST = 777
uint16 MISSION_INIT = 50000
uint16 MISSION_PIT_INIT = 65000
uint16 MISSION_DEFAULT = 65535

uint16 mission_status_code 65535

# Maximum allowed linear velocity in meter per seconds
float64 max_velocity_mps 0.0
