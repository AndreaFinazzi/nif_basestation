#!/usr/bin/env bash
__dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
__file_path="${__dir}/$(basename "${BASH_SOURCE[0]}")"
__file_name="$(basename "${BASH_SOURCE[0]}")"
__call_dir=$(pwd)

export IAC_BASESTATION_ROOT=$__dir/../../
docker-compose -f $IAC_BASESTATION_ROOT/operations/docker-compose.yml up --force-recreate -d nifbasestation
docker exec --privileged -w "/workspace" -it nifbasestation \
    /bin/bash -i -c 'colcon build --symlink-install --packages-select nif_msgs deep_orange_msgs bvs_msgs raptor_dbw_msgs joystick_vehicle_interface joystick_vehicle_interface_nodes userinput py_pubsub'
