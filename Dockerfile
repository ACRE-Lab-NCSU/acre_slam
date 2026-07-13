FROM osrf/ros:humble-desktop

# Install development essentials
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    gpg \
    python3-colcon-common-extensions \
    cmake \
    build-essential \
    libomp-dev \
    libboost-all-dev \
    libmetis-dev \
    libfmt-dev \
    libspdlog-dev \
    libglm-dev \
    libglfw3-dev \
    libpng-dev \
    libjpeg-dev \
    libeigen3-dev \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    libyaml-cpp-dev \
    ros-humble-librealsense2* \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Clone and build Unitree CycloneDDS packages
RUN git clone https://github.com/unitreerobotics/unitree_ros2.git /opt/unitree_ros2 && \
    cd /opt/unitree_ros2/cyclonedds_ws && \
    . /opt/ros/humble/setup.sh && \
    CC=gcc CXX=g++ colcon build --symlink-install

# Install PCL and ANYbotics Gridmap
RUN apt-get update && apt-get install -y \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/anybotics_ws/src && \
    cd /opt/anybotics_ws/src && \
    git clone https://github.com/anybotics/grid_map.git --branch humble && \
    apt-get update && \
    rosdep update && \
    cd /opt/anybotics_ws && \
    . /opt/ros/humble/setup.sh && \
    rosdep install -y --ignore-src --from-paths src && \
    colcon build --symlink-install && \
    rm -rf /var/lib/apt/lists/*

# Intsall Livox LiDAR SDK and ROS2 Driver
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /opt/Livox-SDK2 && \
    cd /opt/Livox-SDK2 && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install

RUN apt-get update && \
    mkdir -p /opt/ws_livox/src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git /opt/ws_livox/src/livox_ros_driver2 && \
    cd /opt/ws_livox/src/livox_ros_driver2 && \
    . /opt/ros/humble/setup.sh && \
    ./build.sh humble && \
    rm -rf /var/lib/apt/lists/*

# Install FAST-LIO2
RUN apt-get update && \
    mkdir -p /opt/fastlio_ws/src && \
    git clone https://github.com/Ericsii/FAST_LIO.git --recursive /opt/fastlio_ws/src/FAST_LIO && \
    . /opt/ros/humble/setup.sh && \
    . /opt/ws_livox/install/setup.sh && \
    cd /opt/fastlio_ws && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf /var/lib/apt/lists/*

# Entrypoint
RUN printf '#!/bin/bash\n\
source /opt/ros/humble/setup.sh\n\
source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash\n\
source /opt/anybotics_ws/install/setup.bash\n\
source /opt/ws_livox/install/setup.bash\n\
source /opt/fastlio_ws/install/setup.bash\n\
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n\
exec "$@"\n' > /entrypoint.sh && chmod +x /entrypoint.sh

COPY . /workspace
WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]