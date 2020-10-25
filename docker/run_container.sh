#!/usr/bin/bash
docker run --gpus all -it --tty --user ubuntu --name $1 -v $(realpath ~)/ros2_ws:/home/ubuntu/ros2_ws --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -w /home/ubuntu/ros2_ws wngfra/ros2cuda:libfranka
