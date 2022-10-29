#!/usr/bin/env bash
set -e

# This should aready have been done by entrypoint.sh in .bashrc
# but including here for good measure
source "/opt/ros/$ROS_DISTRO/setup.bash"

echo
echo "~~~~~~~~~~~~~~~~~~~~~~~~~"
echo " Installing dependencies"
echo "~~~~~~~~~~~~~~~~~~~~~~~~~"
echo

pushd ${ROS_WS}

if [[ $1 == 'clean' ]]; then
    rm -Rf install build logs
fi

if [[ ! -f ${HOME}/.apt-rosdep-updated ]]; then
    if [[ $USER == "root" ]]; then
        apt update
    fi
    rosdep update
    touch ${HOME}/.apt-rosdep-updated
else
    echo "Skipping apt and rosdep update as it was previously run."
fi
rosdep install --from-paths src -y -i --skip-keys=librealsense2


echo
echo "~~~~~~~~~~~~~~~~~~~~~~~~"
echo " Building the workspace"
echo "~~~~~~~~~~~~~~~~~~~~~~~~"
echo

# This also assumes that the distro's setup.bash is called
# in entrypoint.sh via .bashrc

bash -c "colcon build --packages-select realsense2_camera realsense2_camera_msgs realsense2_description"

# bash -c "colcon build --packages-select sllidar_ros2 --symlink-install"
bash -c "colcon build --packages-select rplidar_ros --symlink-install"

bash -c "colcon build --packages-select roboclaw_interfaces roboclaw_driver2"

bash -c "colcon build --packages-select r2b2 r2b2_base --symlink-install"

echo
