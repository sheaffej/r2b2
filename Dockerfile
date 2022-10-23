FROM ros:galactic-ros-base

ENV ROS_WS /ros2
ENV PKG_NAME r2b2
ENV PYTEST_ADDOPTS "--color=yes"

# Install pip and system packages
RUN apt-get update \
	&& apt-get install -y \
        python-is-python3 \
        python3-pip \
        vim less \
        software-properties-common \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# ---------------------------
# Install Intel librealsense2
# ---------------------------
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
&& sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
&& apt-get update \
&& sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg \
&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# Update apt and rosdep, and don't purge the caches this time
RUN apt-get update \
&&  rosdep update \
&&  touch /root/.apt-rosdep-updated

# Overide the default ros_entrypoint.sh file
ADD entrypoint.sh /entrypoint.sh
RUN rm -f /ros_entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Python dependencies are installed by rosdep (see package.xml)

# Add bash aliases for convenience
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
RUN git clone https://github.com/Slamtec/sllidar_ros2.git
RUN git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-beta


# ----------------
# Add this package
# ----------------
ADD . /workspaces/${PKG_NAME}
RUN ln -s /workspaces/${PKG_NAME} ${ROS_WS}/src/${PKG_NAME}


# -------------------
# Build the workspace
# -------------------
RUN ${ROS_WS}/src/${PKG_NAME}/run/build.sh
