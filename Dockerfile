FROM ros:humble-ros-base

ENV ROS_WS /ros2
ENV PKG_NAME r2b2
ENV PYTEST_ADDOPTS "--color=yes"
SHELL ["/bin/bash", "-c"]

# Overide the default ros_entrypoint.sh file
ADD entrypoint.sh /entrypoint.sh
RUN rm -f /ros_entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Install pip and system packages
RUN apt-get update \
	&& apt-get install -y \
        wget \
        vim \
        less \
        python-is-python3 \
        python3-pip \
        software-properties-common \
    # Add big ROS packages
        ros-${ROS_DISTRO}-navigation2 \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-robot-localization \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# ---------------------------
# Install Intel librealsense2
# ---------------------------

# # ~~~~ Galactic ~~~~
# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
# && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
# && apt-get update \
# && apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg \
# && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# ~~~~ Humble ~~~~
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        wget \
        dkms \
        git \
        libusb-1.0-0-dev \
        libudev-dev \
        libglfw3-dev \
        pkg-config \
        libgtk-3-dev \
        at \
        unzip \
        libpopt0 \
        rsync \
        libglu1 \
        libglu1-mesa \
        python3-colcon-common-extensions \
        python3-rosdep \
        build-essential \
        nano \
        usbutils && \
    mkdir -p ${ROS_WS}/src && \
    cd ${ROS_WS}/src && \
    git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development && \
    wget -q https://github.com/mengyui/librealsense2-dkms/releases/download/initial-support-for-kernel-5.15/librealsense2-dkms-dkms_1.3.14_amd64.deb && \
    wget -q https://github.com/mengyui/librealsense2-dkms/releases/download/initial-support-for-kernel-5.15/libssl1.1_1.1.1f-1ubuntu2.16_amd64.deb && \
    wget -q https://github.com/mengyui/librealsense2-dkms/releases/download/initial-support-for-kernel-5.15/PREBUILT-DEB-PACKAGE-librealsense2_2.50.0-0.realsense0.6128_amd64.zip && \
    unzip PREBUILT-DEB-PACKAGE-librealsense2_2.50.0-0.realsense0.6128_amd64.zip && \
    dpkg -i librealsense2-dkms-dkms_1.3.14_amd64.deb && \
    dpkg -i libssl1.1_1.1.1f-1ubuntu2.16_amd64.deb && \
    dpkg -i librealsense2-udev-rules_2.50.0-0~realsense0.6128_amd64.deb && \
    dpkg -i librealsense2_2.50.0-0~realsense0.6128_amd64.deb && \
    dpkg -i librealsense2-gl_2.50.0-0~realsense0.6128_amd64.deb && \
    dpkg -i librealsense2-net_2.50.0-0~realsense0.6128_amd64.deb && \
    dpkg -i librealsense2-utils_2.50.0-0~realsense0.6128_amd64.deb && \
    dpkg -i librealsense2-dev_2.50.0-0~realsense0.6128_amd64.deb && \
    rm -f *.deb *.zip && \
    cd realsense-ros/realsense2_camera && \
    sed -i 's/find_package(realsense2 2.51.1)/find_package(realsense2)/' CMakeLists.txt && \
    rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------
# Most Python dependencies are installed by rosdep (see package.xml)
# ------------------------------------------------------------------
# ADD rosdep.yaml /etc/ros/rosdep/rosdep.yaml
# RUN echo "yaml file:///etc/ros/rosdep/rosdep.yaml" > /etc/ros/rosdep/sources.list.d/40-custom.list

# -------------------------------------------
# Workarounds for Python deprexation warnings
# -------------------------------------------
# "PkgResourcesDeprecationWarning: 1.1build1 is an invalid version ..."
# https://askubuntu.com/questions/1406952/what-is-the-meaning-of-this-pkgresourcesdeprecationwarning-warning-from-pipenv
RUN pip install --upgrade --user setuptools==58.3.0
# "EasyInstallDeprecationWarning: easy_install command is deprecated ..."
# "EasyInstallDeprecationWarning: setuptoos command is deprecated ..."
# https://github.com/colcon/colcon-core/issues/454
ENV PYTHONWARNINGS="ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources,${PYTHONWARNINGS}"

# ---------------------
# Update apt and rosdep
# ---------------------
# And don't purge the caches this time
RUN apt-get update \
&&  rosdep update \
&&  touch /root/.apt-rosdep-updated


# --------------------------------
# Add bash aliases for convenience
# --------------------------------
RUN echo 'alias wsenv="source /opt/ros/${ROS_DISTRO}/setup.bash && source ${ROS_WS}/install/setup.bash"' >> /root/.bashrc \
&&  echo 'alias rcd="cd ${ROS_WS}/src/r2b2"' >> /root/.bashrc \
&&  echo 'wsenv' >> /root/.bashrc

RUN mkdir -p ${ROS_WS}/src

# -------------------------
# Add other source packages
# -------------------------

# To force rebuilding the layers below, execute on the host docker host "date > nocache-github"
ADD nocache-github /tmp/nocache-github

WORKDIR ${ROS_WS}/src 
RUN git clone https://github.com/sheaffej/roboclaw_interfaces.git
RUN git clone https://github.com/sheaffej/roboclaw_driver2.git
RUN git clone https://github.com/sheaffej/r2b2-base.git

# # ~~~~ Galactic ~~~~
# RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-beta

# # ~~~~ Galactic ~~~~
# RUN git clone https://github.com/Slamtec/sllidar_ros2.git
# ~~~~ Humble ~~~~
RUN git clone https://github.com/babakhani/rplidar_ros2


# --------------------------------------
# Build packages that don't change often
# --------------------------------------
RUN cd ${ROS_WS} \
&& source "/opt/ros/$ROS_DISTRO/setup.bash" \
&& rosdep install --from-paths src -y -i --skip-keys="librealsense2 gz-sim7 gz-transport12 gz-math7 gz-msgs9" \
&& colcon build --packages-select realsense2_camera realsense2_camera_msgs realsense2_description \
&& colcon build --packages-select rplidar_ros --symlink-install \
&& colcon build --packages-select roboclaw_interfaces roboclaw_driver2


# -----------------------
# Add additional packages
# -----------------------
RUN apt-get update \
&& apt-get install -y \
    ros-${ROS_DISTRO}-vision-msgs \
&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# ----------------
# Add this package
# ----------------
ADD . /workspaces/${PKG_NAME}
RUN ln -s /workspaces/${PKG_NAME} ${ROS_WS}/src/${PKG_NAME}


# -------------------
# Build the workspace
# -------------------
RUN ${ROS_WS}/src/${PKG_NAME}/run/build.sh
