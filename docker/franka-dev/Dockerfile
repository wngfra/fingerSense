# Copyright (c) 2020 wngfra
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

ARG FROM_IMAGE=wngfra/ros2cuda:base
FROM $FROM_IMAGE

# Install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    libeigen3-dev \
    libpoco-dev \
    libqt5svg5-dev \
    libqt5websockets5-dev \
    libqt5opengl5-dev \
    libqt5x11extras5-dev \
    python3-colcon-common-extensions \
    python3-dev \
    python3-matplotlib \
    python3-pip \
    python3-tk \
    qtbase5-dev
    
RUN rm -rf /var/lib/apt/lists/*

RUN pip3 install -U\
    argcomplete \
    numpy \
    pandas \
    pyqt5 \
    pyside2 \
    scikit-fda \
    scikit-learn \
    scipy \
    tensorly \
    jax jaxlib==0.1.67+cuda111 -f https://storage.googleapis.com/jax-releases/jax_releases.html

# Download, build and install libfranka
RUN git clone --recursive https://github.com/frankaemika/libfranka.git /usr/src/libfranka
WORKDIR /usr/src/libfranka/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. \
    && make \
    && make install

# Create a non-root user
RUN useradd -rm -d /ubuntu -s /bin/bash -u 1000 ubuntu \
    && echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /ubuntu/.bashrc \
    && echo 'export XLA_PYTHON_CLIENT_MEM_FRACTION=.60' >> /ubuntu/.bashrc

# Change default user
WORKDIR /ubuntu/ros2_ws
USER ubuntu

CMD ["bash"]
