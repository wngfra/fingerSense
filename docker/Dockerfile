# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

ARG FROM_IMAGE=nvcr.io/nvidia/cuda:11.0-runtime-ubuntu20.04
FROM $FROM_IMAGE

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
    build-essential \
    dirmngr \
    gnupg2 \
    iputils-ping \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libeigen3-dev \
    libpoco-dev \
    python3-dev \
    python3-matplotlib \
    python3-opencv \
    python3-pip \
    python3-tk \
    vim \
    --no-install-recommends \
	&& rm -rf /var/lib/apt/lists/*

RUN pip3 install autopep8 \
    jedi \
    jax jaxlib==0.1.56+cuda110 -f https://storage.googleapis.com/jax-releases/jax_releases.html \
    mayavi

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654


# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Configure environment variables
ARG ROS_DISTRO=foxy
ENV ROS_DISTRO=$ROS_DISTRO
ENV ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3

# label ros2 packages
LABEL org.osrfoundation.ros-foxy-desktop.sha256=ea087f425f8c087d943b6ad98fe52368127ec919bddb7410ab11842bfdf0c1be

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-desktop=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*


# Download, build and install libfranka
RUN git clone --recursive https://github.com/frankaemika/libfranka.git /usr/src/libfranka

WORKDIR /usr/src/libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. \
    && make \
    && make install

# Create a sudo user
RUN useradd -rm -d /home/ubuntu -s /bin/bash -u 1000 ubuntu
USER ubuntu
WORKDIR /home/ubuntu/ros2_ws

# setup entrypoint
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]