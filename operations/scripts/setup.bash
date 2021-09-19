#!/usr/bin/env bash
__dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
__file_path="${__dir}/$(basename "${BASH_SOURCE[0]}")"
__file_name="$(basename "${BASH_SOURCE[0]}")"
__call_dir=$(pwd)

export IAC_BASESTATION_ROOT=$__dir
if [ ! `echo :$PATH: | grep -F :$IAC_BASESTATION_ROOT:` ]; then
   export PATH=$IAC_BASESTATION_ROOT:$PATH
fi