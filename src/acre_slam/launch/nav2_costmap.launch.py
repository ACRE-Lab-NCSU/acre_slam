import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    desc_share = get_package_share_directory('acre_go2_description')
    urdf_path = os.path.join(desc_share, 'urdf', 'go2', 'go2.urdf.xacro')

    return LaunchDescription([

        #Node(
        #    package="joint_state_publisher",
        #    executable="joint_state_publisher",
        #    name="joint_state_publisher",
        #),

        # Robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
        ),

        # Static TF bridge (per your confirmed imu/base_link extrinsic)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_link',
            arguments=['-0.1297', '0.0', '-0.1558', '0', '-0.226893', '0', 'body', 'base_link'],
            output='screen'
        ),

        # Nav2 costmap
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap',
            namespace='costmap',
            parameters=['/workspace/src/acre_slam/config/nav2/costmap.yaml'],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
            ],
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['costmap/costmap']}]
        ),
    ])