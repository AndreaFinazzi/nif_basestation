# Node status message for ros2
# Author : Andrea Finazzi @ KAIST
# Date : 2021.09.02


uint16 node_id 0
string node_name

# node_status
uint8 NODE_OK = 0
uint8 NODE_INITIALIZED = 64
uint8 NODE_INACTIVE = 100
uint8 NODE_NOT_INITIALIZED = 128
uint8 NODE_ERROR = 200
uint8 NODE_FATAL_ERROR = 250
uint8 NODE_DEAD = 255

uint8 node_status_code 0