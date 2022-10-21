#!/usr/bin/env bash

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source ${MYDIR}/env

docker run -d --rm \
--name ${DOCKER_CONTAINER} \
--net host \
--privileged \
--env DISPLAY \
--mount type=bind,source="${HOST_PROJ_DIR}/r2b2",target="${CONTAINER_CODE_MOUNT_DIR}/r2b2" \
--mount type=bind,source="${DOWNLOADS_DIR}",target=/root/Downloads \
${DOCKER_IMAGE} $@