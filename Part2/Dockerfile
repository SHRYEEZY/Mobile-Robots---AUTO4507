FROM ros:jazzy
 
ENV DEBIAN_FRONTEND=noninteractive
 
# Core system + ROS packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-opencv \
    python3-matplotlib \
    python3-numpy \
    git \
    python3-pip \
    build-essential \
    libusb-1.0-0-dev \
    wget \
    doxygen \
    joystick \
    nano \
    ros-jazzy-joy \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-usb-cam \
    ros-jazzy-nmea-navsat-driver \
    ros-jazzy-robot-localization \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-tf2-tools \
    ros-jazzy-gps-msgs \
    ros-jazzy-gps-tools \
    ros-jazzy-phidgets-spatial \
    ros-jazzy-sick-scan-xd \
    ros-jazzy-rviz2 \
    ros-jazzy-foxglove-bridge \
    ros2-testing-apt-source \
    && apt-get update && apt-get install -y \
    ros-jazzy-depthai-ros-v3 \
    && rm -rf /var/lib/apt/lists/*
 
# Initialize rosdep
RUN rosdep init || true
RUN rosdep update
 
# DepthAI Python API
RUN pip3 install --break-system-packages depthai
 
# --- Build and install ARIA (AriaCoda fork) ---
WORKDIR /tmp
RUN git clone https://github.com/reedhedges/AriaCoda.git
WORKDIR /tmp/AriaCoda
RUN make -j$(nproc)
RUN make install
 
# Set ARIA environment variables
ENV ARIA_DIR=/usr/local/Aria
ENV LD_LIBRARY_PATH=${ARIA_DIR}/lib:${LD_LIBRARY_PATH}
ENV PATH=${ARIA_DIR}/bin:${PATH}
 
# --- ROS workspace ---
WORKDIR /root/ros2_ws
COPY src ./src
COPY launch ./launch
 
# Install ROS dependencies for all packages
RUN . /opt/ros/jazzy/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y
 
# Build the workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install
 
# Source workspace automatically
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc
 
# Copy launch files
COPY launch /root/ros2_ws/launch
 
CMD ["bash"]
