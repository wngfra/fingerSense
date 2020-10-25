#!/bin/bash
set -e

# setup ros2 environment
export CMAKE_PREFIX_PATH=\$AMENT_PREFIX_PATH:\$CMAKE_PREFIX_PATH
source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"