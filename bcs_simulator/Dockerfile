# Use ROS2 Iron as base image
FROM ros:iron

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    qtbase5-dev \
    qtchooser \
    qtbase5-dev-tools \
    qtdeclarative5-dev \
    libqt5quickcontrols2-5 \
    qtquickcontrols2-5-dev \
    qml-module-qtquick2 \
    qml-module-qtquick-controls2 \
    qml-module-qtqml \
    qml-module-qtquick-layouts \
    qml-module-qtquick-window2 \
    qml-module-qtgraphicaleffects \
    x11-apps \
    libxcb-randr0-dev \
    libxcb-xtest0-dev \
    libxcb-xinerama0-dev \
    libxcb-shape0-dev \
    libxcb-xkb-dev \
    libpugixml-dev \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user to run the application
RUN useradd -m -s /bin/bash developer
USER developer

# Set display environment variable
ENV DISPLAY=host.docker.internal:0.0

# Create workspace
WORKDIR /ws
RUN mkdir -p /ws/src
COPY src/bcs_simulator /ws/src/bcs_simulator/
COPY --chown=developer:developer . /ws/src/bcs_simulator/

# Source ROS2 in bashrc
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
RUN echo "source /ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]