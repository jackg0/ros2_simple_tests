ARG ROS2_BASE_IMAGE=ROS2_BASE_IMAGE

# Start from ros2 base image, assumes rolling is used
FROM ${ROS2_BASE_IMAGE} as ros2-deps

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# basic system util apt installs
RUN apt-get update && apt-get install --no-install-recommends --yes \
    curl \
    gdb \
    git \
    make \
    pkg-config \
    unzip \
    software-properties-common \
    tmux \
    wget \
    && rm -rf /var/lib/apt/lists/*

# now build/install deps
RUN mkdir -p /src/Upstream

ADD . /src/Upstream/ros2_simple_tests
WORKDIR /src/Upstream/ros2_simple_tests

RUN . /opt/ros/rolling/setup.bash && \
    cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/ros/rolling && \
    cmake --build build -j $(nproc) && \
    cmake --install build
