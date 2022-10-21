#!/usr/bin/env bash

# MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
B2_SCRIPTS_DIR="/home/b2/r2b2_project/r2b2/hosts/b2-nuc8/scripts"

ssh -Tq -o ConnectTimeout=1 b2-nuc8 "${B2_SCRIPTS_DIR}/container-d.sh \
ros2 launch /ros2/src/r2b2/launch/stub.launch.py"