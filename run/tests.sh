#!/bin/bash
set -e

# MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${ROS_WS}/install/setup.bash

echo
echo "============================"
echo "     Running Unit Tests     "
echo "============================"
echo
/usr/bin/env python3 -m pytest -v --cache-clear \
    /ros2/src/r2b2-base/test \
    /ros2/src/roboclaw_driver2/test
