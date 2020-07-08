# Tactile-based Hybrid Control for TacSense Project

## <img src="https://i.udemycdn.com/course/480x270/1797828_c391_3.jpg" width="48" height="27" /> packages
----------------
0. [libfranka](https://frankaemika.github.io/docs/libfranka.html) is needed
1. `franka_control` contains Franka Emika Panda control interface
2. `docker/podman build -t <image_tag> .` creates a ROS2 image with libfranka installed. For OCI runtime, create a development container with `podman run -it --name <container_name> --privileged -v <path-to devel>:/mnt/devel -w /mnt/devel -v /tmp/.X11-unix:/tmp/.X11-unix --env DISPLAY --device /dev/dri --device /dev/snd --device /dev/input --net=host <image_tag> bash`.
`