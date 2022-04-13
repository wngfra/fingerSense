# Tactile-based Control and Perception for FingerSense Project

## <img src="https://i.udemycdn.com/course/480x270/1797828_c391_3.jpg" width="48" height="27" /> 

## Overview
1. [libfranka](https://frankaemika.github.io/docs/libfranka.html) is needed
2. `franka_control` contains Franka Emika Panda control interface
3. [can2wifi2ros](https://github.com/wngfra/can2wifi2ros) contains the ROS2 package for a DIY `CAN2WIFI` module
4. Using a [CPM-Finger](https://www.cyskin.com/cpm-finger-the-finger-for-textile-manipulation/) tactile sensor

## Docker suppor
1. Pre-built [wngfra/ros2cuda]((https://hub.docker.com/r/wngfra/ros2cuda) images for robot control and perception

## Robot Control
1. Non-realtime config will be applied to `robot` when `PREEMPT_RT` not detected
2. Reach (downwards) along z-axis for a surface with the robot's force-torque sensor, confirmed by a force threshold
3. Sliding parameter online update controlled by the perception agent
4. z-axis force control for sliding

## Quick Guide
1. Clone the repository and the submodules into a `ros2` workspace
```bash
git clone --recursive https://github.com/wngfra/fingerSense.git ~/ros2_ws/src/fingerSense
```
2. Create a development container using
```bash
docker run -it --tty --user ubuntu --name fingerSense -v --net=host $(realpath ~)/ros2_ws:/ubuntu/ros2_ws wngfra/ros2cuda:base
```
3. Create a deployment container with GUI support
```bash
docker run -it --tty --device /dev/dri --gpus all --user ubuntu --name fingerSense_gui -v $(realpath ~)/ros2_ws:/ubuntu/ros2_ws -v /tmp/.X11-unix:/tmp/.X11-unix:rw --net=host -e DISPLAY=$DISPLAY -e XAUTHORITY -e NVIDIA_DRIVER_CAPABILITIES=all wngfra/ros2cuda:base
```
4. Build the packages and source the setup script
```bash
colcon build --symlink-install && . install/setup.bash
```
