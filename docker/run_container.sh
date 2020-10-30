#!/usr/bin/bash
docker run -it --tty --device /dev/dri --device /dev/vga_arbiter --gpus all --user ubuntu --name $1 --group-add video -v $(realpath ~)/ros2_ws:/ubuntu/ros2_ws --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix wngfra/ros2cuda:libfranka
