FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    git \
    build-essential \
    doxygen \
    nano \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-sick-scan-xd \
    ros2-testing-apt-source \
    && apt-get update \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# --- Build and install ARIA ---
WORKDIR /tmp
RUN git clone https://github.com/reedhedges/AriaCoda.git
WORKDIR /tmp/AriaCoda
RUN make -j$(nproc)
RUN make install

ENV ARIA_DIR=/usr/local/Aria
ENV LD_LIBRARY_PATH=${ARIA_DIR}/lib:${LD_LIBRARY_PATH}
ENV PATH=${ARIA_DIR}/bin:${PATH}

# --- ROS workspace ---
WORKDIR /root/ros2_ws
COPY src ./src
COPY launch ./launch

RUN . /opt/ros/jazzy/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

COPY launch /root/ros2_ws/launch

CMD ["bash"]
