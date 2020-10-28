#!/usr/bin/bash
docker run --gpus all -it --tty --name $1 -v $(realpath ~)/ros2_ws:/ubuntu/ros2_ws --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix wngfra/ros2cuda:base
