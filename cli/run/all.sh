#!/usr/bin/env bash

# MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJ_DIR="/home/b2/r2b2_project"

ssh -Tq -o ConnectTimeout=1 b2-nuc8 <<EOF 
    echo "Starting R2B2 robot"
    ${PROJ_DIR}/r2b2/run/b2-nuc8/container.sh ros2 launch /ros2/
EOF