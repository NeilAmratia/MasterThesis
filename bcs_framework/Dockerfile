# Use ROS2 Iron as base image
FROM ros:iron

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    llvm \
    clang \
    libclang-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/leethomason/tinyxml2.git && \
    cd tinyxml2 && \
    mkdir build && cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig

# Install tomlplusplus
RUN git clone https://github.com/marzer/tomlplusplus.git && \
    cd tomlplusplus && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make install

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

# Copy both packages
COPY src/bcs_framework /ros2_ws/src/bcs_framework/
COPY src/active_configuration_files /ros2_ws/src/active_configuration_files/

# Source ROS2 in bashrc
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]