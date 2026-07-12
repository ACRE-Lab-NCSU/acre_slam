ARG CUDA=true

FROM osrf/ros:humble-desktop

ARG CUDA

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

# Build GTSAM 4.3a0 (The version is important for GLIM compatability)
RUN git clone https://github.com/borglab/gtsam /tmp/gtsam && \
    cd /tmp/gtsam && git checkout 4.3a0 && \
    mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_WITH_TBB=OFF \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/gtsam

# Build Iridescence (visualization)
RUN git clone --recursive https://github.com/koide3/iridescence /tmp/iridescence && \
    mkdir /tmp/iridescence/build && cd /tmp/iridescence/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/iridescence

# Build gtsam_points
RUN git clone https://github.com/koide3/gtsam_points /tmp/gtsam_points && \
    mkdir /tmp/gtsam_points/build && cd /tmp/gtsam_points/build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
        $([ "$CUDA" = "true" ] && echo "-DBUILD_WITH_CUDA=ON" || echo "-DBUILD_WITH_CUDA=OFF") && \
    make -j$(nproc) && make install && \
    rm -rf /tmp/gtsam_points

RUN ldconfig

# Build GLIM + glim_ros2 from source
RUN mkdir -p /opt/glim_ws/src && \
    cd /opt/glim_ws/src && \
    git clone https://github.com/koide3/glim && \
    git clone https://github.com/koide3/glim_ros2

RUN cd /opt/glim_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args \
        $([ "$CUDA" = "true" ] && echo "-DBUILD_WITH_CUDA=ON" || echo "-DBUILD_WITH_CUDA=OFF") \
        -DBUILD_WITH_VIEWER=ON \
        -DBUILD_WITH_MARCH_NATIVE=OFF

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


# Entrypoint
RUN printf '#!/bin/bash\n\
source /opt/ros/humble/setup.sh\n\
source /opt/glim_ws/install/setup.bash\n\
source /opt/unitree_ros2/cyclonedds_ws/install/setup.bash\n\
source /opt/anybotics_ws/install/setup.bash\n\
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n\
exec "$@"\n' > /entrypoint.sh && chmod +x /entrypoint.sh

COPY . /workspace
WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]