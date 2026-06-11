# Start from the official desktop image
FROM osrf/ros:humble-desktop

# Install development essentials
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    python3-colcon-common-extensions \
    cmake \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    libyaml-cpp-dev \
    ros-humble-librealsense2* \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-rko-lio \
    && rm -rf /var/lib/apt/lists/*

# Install Livox-SDK2
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /opt/Livox-SDK2/ \
    && cd /opt/Livox-SDK2 \
    && mkdir build && cd build \
    && CC=gcc CXX=g++ cmake .. && make -j$(nproc) \
    && make install

# Clone and build Livox ROS2 driver
RUN git clone --recursive https://github.com/Livox-SDK/livox_ros_driver2.git /opt/ws_livox/src/livox_ros_driver2 \
    && cd /opt/ws_livox/src/livox_ros_driver2 \
    && . /opt/ros/humble/setup.sh \
    && ./build.sh humble

# Clone and build Unitree cyclonedds packages
RUN git clone https://github.com/unitreerobotics/unitree_ros2.git /opt/unitree_ros2
RUN cd /opt/unitree_ros2/cyclonedds_ws && \
    . /opt/ros/humble/setup.sh && \
    CC=gcc CXX=g++ colcon build

# Source everything in bashrc
RUN echo "source /opt/ros/humble/setup.sh" >> ~/.bashrc && \
    echo "source /opt/ws_livox/install/setup.bash" >> ~/.bashrc && \
    echo "source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Copy project files
COPY . /workspace

WORKDIR /workspace
CMD ["bash"]