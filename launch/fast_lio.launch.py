import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('acre_cbf')
    urdf_path = os.path.join(pkg_share, 'urdf', 'go2.urdf.xacro')

    # Livox MID360 to Internal Go2 IMU
    base_link_to_body_tf = ['0.1870', '0.0', '0.0803', '0', '0.226893', '0', 'base_link', 'body']

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
        ),

        # base_link -> body
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_body',
            arguments=base_link_to_body_tf,
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
            ),
            launch_arguments={
                # set FAST-LIO2's world/odom frame param here if it supports one,
                # so 'camera_init' never appears in the tree at all
                'config_file': 'go2_extrinsics.yaml',
            }.items()
        ),
    ])