FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    python3-pip \
    libusb-1.0-0 \
    libglib2.0-0 \
    libgl1 \
    udev \
    nano \
    ros-jazzy-cv-bridge \
    ros-jazzy-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --break-system-packages \
    --ignore-installed \
    "numpy<2" depthai opencv-python-headless

WORKDIR /ros2_ws
COPY src/ src/
COPY launch/ launch/ 

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

RUN mkdir /output
VOLUME /output

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
