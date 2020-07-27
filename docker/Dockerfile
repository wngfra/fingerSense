# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

ARG FROM_IMAGE=osrf/ros:foxy-desktop
FROM $FROM_IMAGE

# install packages
RUN apt-get update && apt-get install -q -y \
    build-essential \
    libeigen3-dev \
    libpoco-dev \
    python3-pip \
    python3-tk \
    vim \
    --no-install-recommends \
	&& rm -rf /var/lib/apt/lists/*

RUN pip3 install autopep8 \
    jedi

# Download, build and install libfranka
RUN git clone --recursive https://github.com/frankaemika/libfranka.git /usr/src/libfranka

WORKDIR /usr/src/libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. \
    && make \
    && make install

WORKDIR /mnt/ros2_ws
RUN rm -rf /usr/src/libfranka/build

# ROS2 & CMake configurations
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc \
    && echo "export CMAKE_PREFIX_PATH=\$AMENT_PREFIX_PATH:\$CMAKE_PREFIX_PATH" >> ~/.bashrc

# Configure environment variables
ARG ROS_DISTRO=foxy
ENV ROS_DISTRO=$ROS_DISTRO
ENV ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3

ENTRYPOINT ["bash"]