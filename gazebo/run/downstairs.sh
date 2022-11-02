#!/usr/bin/env bash

WORLD_SDF="downstairs.sdf"

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
GZ_DIR="${MYDIR}/.."

export GZ_SIM_RESOURCE_PATH="${MYDIR}/../models"

xacro ${MYDIR}/../../config/urdf/r2b2.xacro > ${MYDIR}/../../config/urdf/r2b2.urdf
xacro ${MYDIR}/../models/walls.xacro > ${MYDIR}/../models/walls.sdf

gz sim --render-engine ogre --gui-config ${GZ_DIR}/config/gui.config ${GZ_DIR}/worlds/${WORLD_SDF}
