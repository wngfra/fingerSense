# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

ARG FROM_IMAGE=stereolabs/zed:3.4-gl-devel-cuda11.1-ubuntu20.04
FROM $FROM_IMAGE

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    build-essential \
    dirmngr \
    gnupg2 \
	&& rm -rf /var/lib/apt/lists/*

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
    ros-foxy-desktop=0.9.2-1* \
    ros-foxy-launch* \
    && rm -rf /var/lib/apt/lists/*

# Install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    git \
    libeigen3-dev \
    libpoco-dev \
    python3-colcon-common-extensions \
    python3-dev \
    python3-matplotlib \
    python3-pip \
    python3-rosdep \    
    python3-tk \
    ros-$ROS_DISTRO-gazebo-ros-pkgs
RUN rm -rf /var/lib/apt/lists/*

RUN pip3 install -U\
    argcomplete

RUN rosdep init && rosdep update

# Download, build and install libfranka
RUN git clone --recursive https://github.com/frankaemika/libfranka.git /usr/src/libfranka
WORKDIR /usr/src/libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. \
    && make \
    && make install

WORKDIR /root/ros2_ws/src/
RUN git clone https://github.com/stereolabs/zed-ros2-wrapper.git \
    && cd zed-ros2-wrapper \
    && git pull origin dev_foxy \
    && git checkout dev_foxy

WORKDIR /root/ros2_ws/
RUN rosdep install --from-paths src --ignore-src -r -y \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc \
    && echo "source $(pwd)/install/local_setup.bash" >> /root/.bashrc

CMD ["bash"]