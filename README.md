# Tactile-based Reactive Control Project for Franka Panda Robotic arm

## <img src="https://i.udemycdn.com/course/480x270/1797828_c391_3.jpg" width="48" height="27" /> packages
----------------
0. [libfranka](https://frankaemika.github.io/docs/libfranka.html) is needed
1. Tactile sensor driver for PCAN-USB FD using [PCAN_Basic](https://www.peak-system.com/fileadmin/media/linux/index.htm)
2. `franka_py` contains cognitive control and signal processing
3. `franka_msgs` contains custom message types
4. `franka_control` contains sensor driver and Franka Emika Panda control interface
5. Visualizing data in realtime with `MATLAB` ROS2 toolkit
6. `docker/podman build -t ros2franka .` creates a ROS2 image with libfranka installed. For OCI runtime, create a development container with `podman run -it --name ros2-foxy --privileged -v <path-to devel>:/mnt/devel -w /mnt/devel -v /tmp/.X11-unix:/tmp/.X11-unix --env DISPLAY --device /dev/dri --device /dev/snd --device /dev/input --net=host <tag-of-image> bash`.
`