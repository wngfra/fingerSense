# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

ARG FROM_IMAGE=osrf/ros:foxy-desktop
FROM $FROM_IMAGE

# install packages
RUN apt-get update && apt-get install -q -y \
    build-essential \
    libeigen3-dev \
    libpoco-dev \
    vim

# Download, build and install libfranka
WORKDIR /usr/src
RUN git clone --recursive https://github.com/frankaemika/libfranka.git
WORKDIR /usr/src/libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
RUN make
RUN make install

# Configure environment variables
ARG ROS_DISTRO=foxy
ENV ROS_DISTRO=$ROS_DISTRO
ENV ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3

ENV CMAKE_PREFIX_PATH=$AMENT_PREFIX_PATH:$CMAKE_PREFIX_PATH
ENV DISPLAY=$DISPLAY

# Enable X11 forwarding
VOLUME /tmp/.X11-unix /tmp/.X11-unix

CMD echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

ENTRYPOINT /usr/bin/bash