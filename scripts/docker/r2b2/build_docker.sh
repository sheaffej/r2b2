#!/usr/bin/env bash

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source ${MYDIR}/env

REPO_DIR="${HOME}/r2b2_project/r2b2"

cd ${REPO_DIR}
docker build -t ${DOCKER_IMAGE} .

