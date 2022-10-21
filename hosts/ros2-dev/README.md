# `ros2-dev` materials
These are scripts, config, launch files, etc to be used by my `ros2-dev` workstation machine when developing, testing, and running the R2B2 robot.

My `ros2-dev` machine is currently an Ubuntu 20.04.4 VM running in VMware Fusion on my Macbook Pro 13". This is my main control plane when working with the robot. And since it's a 13" Macbook Pro, it's pretty light, which makes it easy to carry around in one hand as I chase the robot around the downstairs floor of my house.

I prefer to run this ROS2 machine in a VM vs natively on my Macbook, since the ROS installs are fairly complicated and require a decent amount of environment setups. I'd prefer to not do that directly on my Macbook. Using a VM also means I can snapshot the environment, and rollback to a snapshot if/when I mess up the ROS environment. This gives me the freedom to try out different things in this dev environment, and not worry about getting back to a stable state.

The downside is the rendering of things like Rviz in a Linux VM running on a Macbook isn't perfect. And hooking up USB 3.x devices directly to the Macbook to be used by the ROS environment (like an Intel Realsense 3D camera) doesn't really work. The high-bandwidth 3.x data rates through the VMware layer is problematic. But all in all, the predictable environment and not having to worry about macOS nuances with ROS is worth it.



