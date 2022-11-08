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
    # unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH
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
rosdep install --from-paths src -y -i --skip-keys="librealsense2 gz-sim7 gz-transport12 gz-math7 gz-msgs9"


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

if [[ -d "${ROS_WS}/src/ros_gz" ]]; then
    bash -c "colcon build --packages-select-regex ros_gz --allow-overriding ros_gz_bridge ros_gz_image ros_gz_interfaces ros_gz_sim ros_gz_sim_demos"
fi

echo
echo "Soft linking r2b2 files:"
for d in launch config gazebo; do
    TARGET="${ROS_WS}/install/r2b2/share/r2b2/${d}"
    if [[ ! -e ${TARGET} ]]; then
        echo "  ${TARGET}"
        ln -s ${ROS_WS}/src/r2b2/${d} ${TARGET}
    else
        echo "  ${TARGET} [exists]"
    fi
done

echo
