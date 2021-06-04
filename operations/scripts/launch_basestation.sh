#!/usr/bin/env bash
__dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
__file_path="${__dir}/$(basename "${BASH_SOURCE[0]}")"
__file_name="$(basename "${BASH_SOURCE[0]}")"
__call_dir=$(pwd)

export IAC_BASESTATION_ROOT=$__dir/../../

docker exec --privileged -e DISPLAY=${DISPLAY} -e LINES=`tput lines` \
        -w "/workspace" -it iac_basestation /bin/tmuxp load /workspace/operations/config/launch_basestation.yaml