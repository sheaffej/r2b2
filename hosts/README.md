# `hosts/`
These are scripts and files that are used outside of the robot's Docker container.

For example, these are scripts that start/stop the docker containers on specific hosts, or run commands natively on the host's operating system and not in the ROS2 environment. 

The majority of the files outside this directory are expected to be used inside of the ROS2 Docker container(s).

Hosts:
- `b2-nuc8`: The Intel NUC8 running on the R2B2 robot base
- `ros2-dev`: A Ubuntu VM on which I run RQT and Rviz during robot testing and operation