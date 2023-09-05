#!/bin/bash

MYDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
# source ${MYDIR}/../common/common.sh
# source ${MYDIR}/env

LOCAL_PROJ_DIR="${HOME}/Dropbox/code-projects/r2b2_project"
REPOS="r2b2"
HOST="b2-nuc8"
HOST_PROJ_DIR="/home/b2/r2b2_project"

function push_dir_to_host {
    # Follows the SCP pattern: user@host:project_dir/dir
    HOST=$1
    HOST_PROJ_DIR=$2
    SYNC_DIR=$3

    # To copy SYNC_DIR into HOST_PROJ_DIR, neither should have trailing slashes below
    # Filter out just the display of the .git changes
    rsync -azi -e ssh --delete \
    --exclude=".pytest_cache/" --exclude="*.pyc" --exclude="__pycache__" --exclude dump \
    "${LOCAL_PROJ_DIR}/${SYNC_DIR}" \
    "${HOST}:${HOST_PROJ_DIR}" | grep -v "/.git/"
}

echo "~~~~ Pushing to ${HOST} ~~~~"
for REPO in ${REPOS}; do
    echo "Pushing ${REPO}"
    push_dir_to_host ${HOST} ${HOST_PROJ_DIR} ${REPO}
done