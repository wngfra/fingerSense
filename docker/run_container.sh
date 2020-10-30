#!/usr/bin/bash
docker run --user ubuntu --name franka_dev -v $(realpath ~)/ros2_ws:/ubuntu/ros2_ws --net=host wngfra/ros2franka:latest
docker run --device /dev/dri --gpus all --user ubuntu --name visualization_dev -v $(realpath ~)/ros2_ws:/ubuntu/ros2_ws --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix wngfra/ros2cuda:visualization
