# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

ARG FROM_IMAGE=wngfra/ros2cuda:base
FROM $FROM_IMAGE

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    git \
    iputils-ping \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libeigen3-dev \
    libpoco-dev \
    libpython3-dev \
    python3-colcon-common-extensions \
    python3-dev \
    python3-matplotlib \
    python3-opencv \
    python3-pip \
    python3-tk \
	&& rm -rf /var/lib/apt/lists/*

RUN pip3 install \
    argcomplete \
    autopep8 \
    jedi \
    jax jaxlib==0.1.56+cuda110 -f https://storage.googleapis.com/jax-releases/jax_releases.html \
    mayavi \
    pandas \
    scipy

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

# Create a sudo user
RUN useradd -rm -d /ubuntu -s /bin/bash -u 1000 ubuntu
USER ubuntu
WORKDIR /ubuntu/ros2_ws

# setup entrypoint
COPY ./ros_entrypoint.sh /ubuntu/
RUN chmod +x /ubuntu/ros_entrypoint.sh

ENTRYPOINT ["/ubuntu/ros_entrypoint.sh"]
CMD ["bash"]