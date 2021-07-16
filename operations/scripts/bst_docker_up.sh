#!/usr/bin/env bash
__dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
__file_path="${__dir}/$(basename "${BASH_SOURCE[0]}")"
__file_name="$(basename "${BASH_SOURCE[0]}")"
__call_dir=$(pwd)

export IAC_BASESTATION_ROOT=$__dir/../../
docker-compose -f $IAC_BASESTATION_ROOT/operations/docker-compose.yml up --force-recreate -d iac_basestation
docker exec --privileged -w "/workspace" -it iac_basestation /bin/bash -i -c 'colcon build --packages-select deep_orange_msgs raptor_dbw_msgs joystick_vehicle interface joystick_vehicle_interface_nodes userinput'