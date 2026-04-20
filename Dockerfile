FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    make \
    g++ \
    unzip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-opencv \
    libaria3 \
    libaria3-dev \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-robot-localization \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

# Copy your packages into src
COPY ./pioneer_avoid /ros2_ws/src/pioneer_avoid
COPY ./pioneer_description /ros2_ws/src/pioneer_description
COPY ./pioneer_bringup /ros2_ws/src/pioneer_bringup
COPY ./pioneer_navigation /ros2_ws/src/pioneer_navigation

# If ariaNode is a ZIP file, copy and unzip it
COPY ./ariaNode.zip /tmp/ariaNode.zip
RUN mkdir -p /ros2_ws/src/ariaNode && \
    unzip /tmp/ariaNode.zip -d /ros2_ws/src/ariaNode || true

# rosdep
RUN source /opt/ros/jazzy/setup.bash && \
    rosdep init || true && \
    rosdep update || true

# Install ROS package dependencies from package.xml files
RUN source /opt/ros/jazzy/setup.bash && \
    cd /ros2_ws && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Build workspace
RUN source /opt/ros/jazzy/setup.bash && \
    cd /ros2_ws && \
    colcon build --merge-install

# Auto-source
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
