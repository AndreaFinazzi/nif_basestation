#!/usr/bin/env bash
__dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
__file_path="${__dir}/$(basename "${BASH_SOURCE[0]}")"
__file_name="$(basename "${BASH_SOURCE[0]}")"
__call_dir=$(pwd)

export IAC_BASESTATION_ROOT=$__dir/../../
docker-compose -f $IAC_BASESTATION_ROOT/operations/docker-compose.yml build --force-rm nifbasestation
