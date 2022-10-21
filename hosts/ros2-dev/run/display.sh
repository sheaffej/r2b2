#!/usr/bin/env bash

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

ros2 launch ${MYDIR}/../launch/display.launch.yaml

