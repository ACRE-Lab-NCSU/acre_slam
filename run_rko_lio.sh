#!/bin/bash

# Kill any existing transforms
kill $(pgrep -f static_transform_publisher) 2>/dev/null

# Publish IMU to lidar transform
ros2 run tf2_ros static_transform_publisher \
    0.187 0 0.0803 0 0.2269 0 \
    base_link livox_frame &

# odom to base_link identity
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 odom base_link &

# Launch rko_lio
ros2 launch rko_lio odometry.launch.py \
    config_file:=/workspace/config/rko_lio.yaml \
    rviz:=true \
    rviz_config_file:=/workspace/config/go2_rko.rviz