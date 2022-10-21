# Variables

LOCAL_PROJ_DIR=~/Dropbox/code-projects/r2b2_project
HOSTS="ros-dev b2-pi4 b2-nuc8"

# ROS_MASTER_URI="http://${ROS_MASTER_HOST}:11311/"

# Functions
function push_dir_to_host {
    # Follows the SCP pattern: user@host:project_dir/dir
    HOST=$1
    HOST_PROJ_DIR=$2
    SYNC_DIR=$3

    # To copy SYNC_DIR into HOST_PROJ_DIR, neither should have trailing slashes below
    # Filter out just the display of the .git changes
    rsync -azi -e ssh --delete \
    --exclude=".pytest_cache/" --exclude="*.pyc" \
    "${LOCAL_PROJ_DIR}/${SYNC_DIR}" \
    "${HOST}:${HOST_PROJ_DIR}" | grep -v "/.git/"
}


function exec_ssh {
HOST="$1"
CMD="$2"
ssh -Tq -o ConnectTimeout=1 $HOST <<EOF
"${CMD}"
EOF
}