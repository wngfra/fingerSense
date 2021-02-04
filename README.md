# Tactile-based Control and Perception for FingerSense Project

## <img src="https://i.udemycdn.com/course/480x270/1797828_c391_3.jpg" width="48" height="27" /> 

## Overview
1. [libfranka](https://frankaemika.github.io/docs/libfranka.html) is needed
2. `franka_control` contains Franka Emika Panda control interface
3. `can2wifi2ros` contains codes for a DIY `CAN2WIFI` module, see `README` there
4. Using a [CPM-Finger](https://www.cyskin.com/cpm-finger-the-finger-for-textile-manipulation/) tactile sensor
5. Non-realtime config will be applied to `robot` when `PREEMPT_RT` not detected
6. Default `UDP` socket ip of `tactile_signal_publisher` is binded to `0.0.0.0` for use in containers
7. Added node state management; node state list is in `tactile_publisher.py`

## Container
1. `Dockerfile` for development/deployment containers
2. CUDA support added
3. Pre-built images are available at dockerhub `wngfra/ros2cuda`

## Robot Control
1. Reach (downwards) along z-axis for a surface with the robot's force-torque sensor, confirmed by a force threshold
2. Sliding parameter online update controlled by the perception agent
4. z-axis force control for sliding
