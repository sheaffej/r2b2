#!/usr/bin/env bash

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source ${MYDIR}/env

# DOCKER_IMAGE="r2b2-robot"
# CONTAINER_NAME="r2b2-robot-it"
# LABEL="r2b2"

# MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
# PROJ_DIR=${HOME}/r2b2_project
# DOWNLOADS_DIR=~/Downloads
# CODE_MOUNT="/workspaces"

# docker run -it --rm \
# --label ${LABEL} \
# --net host \
# --privileged \
# --env DISPLAY \
# --mount type=bind,source=$PROJ_DIR/r2b2,target=$CODE_MOUNT/r2b2 \
# --mount type=bind,source=$DOWNLOADS_DIR,target=/root/Downloads \
# ${DOCKER_IMAGE} /bin/bash


if [[ -z $1 ]]; then
    CMDS="/bin/bash"
else
    CMDS="$@"
fi

if [[ "$( docker container inspect -f '{{.State.Status}}' ${DOCKER_CONTAINER} )" = "running" ]]; then
    echo
    echo "Attaching to running container"
    echo
    docker exec -it ${DOCKER_CONTAINER} ${CMDS}
else
    echo
    echo "STARTING THE CONTAINER"
    echo "Exiting this shell will terminate all other shells"
    echo
    ${MYDIR}/container-it.sh "${CMDS}"
fi
