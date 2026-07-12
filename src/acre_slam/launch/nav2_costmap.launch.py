from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    return LaunchDescription([

        # Local costmap
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap',
            namespace='costmap',
            parameters=['/workspace/src/acre_slam/config/nav2/costmap.yaml'],
            output='screen'
        ),

        TimerAction(period=5.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/costmap/costmap', 'configure'],
                output='screen'
            ),
        ]),
        TimerAction(period=8.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/costmap/costmap', 'activate'],
                output='screen'
            ),
        ]),
    ])