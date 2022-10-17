FROM ros:galactic-ros-base

ENV ROS_WS /ros2
ENV PKG_NAME r2b2
ENV PYTEST_ADDOPTS "--color=yes"

# Install pip and system packages
RUN apt update \
	&& apt install -y \
        python-is-python3 \
        python3-pip \
        vim less \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# Python dependencies are installed by rosdep (see package.xml)

RUN echo 'alias wsenv="source /opt/ros/${ROS_DISTRO}/setup.bash && source ${ROS_WS}/install/setup.bash"' >> /root/.bashrc \
&&  echo 'alias rcd="cd ${ROS_WS}/src/r2b2"' >> /root/.bashrc


RUN mkdir -p ${ROS_WS}/src

# Add this package
RUN ln -s /workspaces/${PKG_NAME} ${ROS_WS}/src/${PKG_NAME}

# -------------------------
# Add other source packages
# -------------------------

# To force rebuilding the layers below, execute on the host docker host "date > nocache-github"
ADD nocache-github /tmp/nocache-github

WORKDIR ${ROS_WS}/src 
RUN git clone https://github.com/sheaffej/roboclaw_interfaces.git
RUN git clone https://github.com/sheaffej/roboclaw_driver2.git
RUN git clone https://github.com/sheaffej/r2b2-base.git
