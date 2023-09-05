FROM ros:humble-perception

ENV ROS_WS /ros2
# ENV PKG_NAME r2b2
SHELL ["/bin/bash", "-c"]

# Overide the default ros_entrypoint.sh file
ADD entrypoint.sh /entrypoint.sh
RUN rm -f /ros_entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Install pip and system packages
RUN apt-get update \
	&& apt-get install -y \
        wget \
        unzip \
        vim \
        less \
        python-is-python3 \
        python3-pip \
        software-properties-common \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# For ros-industrial
RUN apt-get update \
	&& apt-get install -y \
        locales \
        curl \
        gcc \
        make \
        gnupg2 \
        lsb-release \
        git \
        meld \
        build-essential \
        libfontconfig1 \
        mesa-common-dev \
        libglu1-mesa-dev \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

RUN git clone -b humble https://github.com/ros-industrial/industrial_training.git ~/industrial_training

# Other packages
RUN apt-get update \
    && apt-get install -y \
        ros-humble-rviz2 \
        pcl-tools \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 
    

# --------------------------------
# Add bash aliases for convenience
# --------------------------------
RUN echo 'alias wsenv="source /opt/ros/${ROS_DISTRO}/setup.bash && source ${ROS_WS}/install/setup.bash"' >> /root/.bashrc \
&&  echo 'wsenv' >> /root/.bashrc
