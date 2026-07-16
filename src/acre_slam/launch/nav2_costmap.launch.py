from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap',
            namespace='costmap',
            parameters=['/workspace/src/acre_slam/config/nav2/costmap.yaml'],
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['costmap/costmap']
            }]
        ),
    ])