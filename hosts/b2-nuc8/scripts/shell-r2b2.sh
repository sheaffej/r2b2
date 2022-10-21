#!/usr/bin/env bash

DOCKER_IMAGE="r2b2-robot"
CONTAINER_NAME="r2b2-robot-it"
LABEL="r2b2"

# MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJ_DIR=/home/b2/r2b2_project
DOWNLOADS_DIR=~/Downloads
CODE_MOUNT="/workspaces"

docker run -it --rm \
--name ${CONTAINER_NAME} \
--label ${LABEL} \
--net host \
--privileged \
--env DISPLAY \
--mount type=bind,source=$PROJ_DIR/r2b2,target=$CODE_MOUNT/r2b2 \
--mount type=bind,source=$DOWNLOADS_DIR,target=/root/Downloads \
${DOCKER_IMAGE} /bin/bash