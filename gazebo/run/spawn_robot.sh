#!/usr/bin/env bash

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
# GZ_DIR="${MYDIR}/.."
# export IGN_GAZEBO_RESOURCE_PATH="${MYDIR}/../models"

xacro ${MYDIR}/../../config/urdf/r2b2.xacro > /tmp/r2b2.urdf


# ign service -s /world/downstairs/remove \
# --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean --timeout 1000 \
# --req 'id: 163'

ign service -s /world/downstairs/create \
--reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 \
--req 'sdf_filename: "/tmp/r2b2.urdf", name: "r2b2_model", pose: {position: {x: 2 y: -1}}'