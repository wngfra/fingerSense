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
    ros-$ROS_DISTRO-gazebo-ros-pkgs
RUN rm -rf /var/lib/apt/lists/*

RUN pip3 install -U\
    argcomplete \
    numpy \
    pandas \
    pyqt5 \
    pyside2 \
    scikit-fda \
    scipy \
    tensorly
RUN pip3 install --upgrade jax jaxlib==0.1.61+cuda110 -f https://storage.googleapis.com/jax-releases/jax_releases.html
RUN pip3 install torch==1.7.1+cu110 torchvision==0.8.2+cu110 torchaudio===0.7.2 -f https://download.pytorch.org/whl/torch_stable.html

# Download, build and install mayavi
RUN git clone https://github.com/enthought/mayavi.git /usr/src/mayavi
WORKDIR /usr/src/mayavi
RUN pip3 install -r requirements.txt \
    && python3 setup.py install

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
