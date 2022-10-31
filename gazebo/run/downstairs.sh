#!/usr/bin/env bash

WORLD_SDF="downstairs.sdf"
# WORLD_SDF="downstairs.processed.sdf"

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
GZ_DIR="${MYDIR}/.."

# export IGN_GAZEBO_RESOURCE_PATH="${MYDIR}/../models"
export GZ_SIM_RESOURCE_PATH="${MYDIR}/../models"

xacro ${MYDIR}/../../config/urdf/r2b2.xacro > ${MYDIR}/../../config/urdf/r2b2.urdf

# ign gazebo --render-engine ogre --gui-config ${GZ_DIR}/config/gui.config ${GZ_DIR}/worlds/downstairs.sdf
gz sim --render-engine ogre --gui-config ${GZ_DIR}/config/gui.config ${GZ_DIR}/worlds/${WORLD_SDF}

