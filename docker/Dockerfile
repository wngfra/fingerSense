# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

ARG FROM_IMAGE=osrf/ros:foxy-desktop
FROM $FROM_IMAGE

# install packages
RUN apt-get update && apt-get install -q -y \
    build-essential \
    libeigen3-dev \
    libpoco-dev

RUN mkdir -p /mnt/devel; cd /mnt/devel; git clone --recursive https://github.com/frankaemika/libfranka.git; mkdir -p libfranka/build; cd libfranka/build; cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..; make; make install

ARG ROS_DISTRO=foxy
ENV ROS_DISTRO=$ROS_DISTRO
ENV ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3

WORKDIR /mnt/devel