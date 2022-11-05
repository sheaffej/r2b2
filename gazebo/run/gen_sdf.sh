#!/usr/bin/env bash

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
export GZ_SIM_RESOURCE_PATH="${MYDIR}/../models"

cd $MYDIR/../worlds

xacro ${MYDIR}/../../config/urdf/r2b2.xacro > ${MYDIR}/../../config/urdf/r2b2.urdf && \
xacro ${MYDIR}/../models/walls.xacro > ${MYDIR}/../models/walls.sdf && \
gz sdf -p downstairs.sdf