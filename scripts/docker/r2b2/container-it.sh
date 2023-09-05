#!/usr/bin/env bash

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source ${MYDIR}/env

# DOCKER_CONTAINER="${DOCKER_CONTAINER}-it"

if [[ -z $1 ]]; then
    CMDS="/bin/bash"
else
    CMDS="$@"
fi


docker run -it --rm \
--name ${DOCKER_CONTAINER} \
--net host \
--privileged \
--env DISPLAY \
--mount type=bind,source="${HOST_PROJ_DIR}/r2b2",target="${CONTAINER_CODE_MOUNT_DIR}/r2b2" \
--mount type=bind,source="${DOWNLOADS_DIR}",target=/root/Downloads \
--mount type=bind,source=/tmp/.X11-unix,target=/tmp.X11-unix \
${DOCKER_IMAGE} ${CMDS}