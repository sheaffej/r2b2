#!/usr/bin/env bash

# MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJ_DIR="/home/b2/r2b2_project"

ssh -Tq -o ConnectTimeout=1 b2-nuc8 <<EOF 
    echo "Stopping R2B2 robot"
    docker kill r2b2-robot
EOF