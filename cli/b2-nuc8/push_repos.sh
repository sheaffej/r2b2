#!/bin/bash

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
source ${MYDIR}/../common.sh
source ${MYDIR}/env

echo "~~~~ Pushing to ${HOST} ~~~~"
for REPO in ${REPOS}; do
    echo "Pushing ${REPO}"
    push_dir_to_host ${HOST} ${HOST_PROJ_DIR} ${REPO}
done