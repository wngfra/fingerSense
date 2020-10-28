# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

ARG FROM_IMAGE=wngfra/ros2cuda:base
FROM $FROM_IMAGE

# Install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    git \
    libeigen3-dev \
    libpoco-dev \
    python3-colcon-common-extensions \
    python3-dev \
    python3-matplotlib \
    python3-pip \
    python3-tk \
	&& rm -rf /var/lib/apt/lists/*

RUN pip3 install -U\
    argcomplete \
    pandas \
    scikit-fda \
    tensorly

# Configure environment variables
ARG ROS_DISTRO=foxy
ENV ROS_DISTRO=$ROS_DISTRO
ENV ROS_VERSION=2 \
    ROS_PYTHON_VERSION=3

# Download, build and install libfranka
RUN git clone --recursive https://github.com/frankaemika/libfranka.git /usr/src/libfranka

WORKDIR /usr/src/libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. \
    && make \
    && make install

# Create a non-root user
RUN useradd -rm -d /ubuntu -s /bin/bash -u 1000 ubuntu \
    && echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /ubuntu/.bashrc

# Change default user
WORKDIR /ubuntu/ros2_ws
USER ubuntu

CMD ["bash"]