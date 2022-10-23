#!/usr/bin/env bash

# MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
B2_SCRIPTS_DIR="/home/b2/r2b2_project/r2b2/hosts/b2-nuc8/scripts"

# ${MYDIR}/../b2-nuc8/build_docker.sh
ssh -Tq -o ConnectTimeout=1 b2-nuc8 "${B2_SCRIPTS_DIR}/build_docker.sh"
