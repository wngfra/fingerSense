# Tactile-based Hybrid Control for TacSense Project

## <img src="https://i.udemycdn.com/course/480x270/1797828_c391_3.jpg" width="48" height="27" /> 

1. [libfranka](https://frankaemika.github.io/docs/libfranka.html) is needed
2. `franka_control` contains Franka Emika Panda control interface
3. `can2wifi2ros` contains codes for a DIY `CAN2WIFI` module, see `README` there
4. Using a [CPM-Finger](https://www.cyskin.com/cpm-finger-the-finger-for-textile-manipulation/) tactile sensor
5. `Dockerfile` is provided for development/deployment in containers, non-realtime config will be applied automatically when `PREEMPT_RT` not detected
6. Alternatively, pull images `wngfra/ros2franka` from [dockerhub](https://hub.docker.com/)
7. For OCI runtime, create a development container with `podman run -it --name <container_name> --privileged -v <path-to devel>:/mnt/devel -w /mnt/devel -v /tmp/.X11-unix:/tmp/.X11-unix --env DISPLAY --device /dev/dri --net=host wngfra/ros2franka:latest bash`.