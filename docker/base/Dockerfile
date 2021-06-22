# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

ARG FROM_IMAGE=nvcr.io/nvidia/cuda:11.1-runtime-ubuntu20.04
FROM $FROM_IMAGE

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# install packages
RUN add-apt-repository universe && \
    apt-get update && apt-get install -q -y --no-install-recommends \
    curl \
    git \
    gnupg \
    lsb-release

# setup keys and sources.list
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Configure environment variables
ARG ROS_DISTRO=galactic
ENV ROS_DISTRO=$ROS_DISTRO
ENV ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_VERSION}-desktop && \
    rm -rf /var/lib/apt/lists/*

CMD ["bash"]