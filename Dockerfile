# Start from the official desktop image
FROM osrf/ros:humble-desktop

# Install development essentials
RUN apt-get update && apt-get install -y \
    libiridescence-dev \ 
    libboost-all-dev \ 
    libglfw3-dev \ 
    libmetis-dev \
    libgtsam-points-cuda12.2-dev \
    git \
    wget \
    curl \
    gpg \
    python3-colcon-common-extensions \
    cmake \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    libyaml-cpp-dev \
    ros-humble-librealsense2* \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-glim-ros-cuda12.2 \
    && rm -rf /var/lib/apt/lists/*

# Make shared Libraries Visable
RUN sudo ldconfig

# Automatically setup PPA via online script
RUN curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash

# Clone and build Unitree cyclonedds packages
RUN git clone https://github.com/unitreerobotics/unitree_ros2.git /opt/unitree_ros2
RUN cd /opt/unitree_ros2/cyclonedds_ws && \
    . /opt/ros/humble/setup.sh && \
    CC=gcc CXX=g++ colcon build

# Source everything in bashrc
RUN echo "source /opt/ros/humble/setup.sh" >> ~/.bashrc && \
    echo "source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Copy project files
COPY . /workspace

WORKDIR /workspace
CMD ["bash"]