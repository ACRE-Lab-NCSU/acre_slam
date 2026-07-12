from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Static TF with respawn enabled
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='livox_to_base_link',
            arguments=[
                '--x', '-0.1870',
                '--y', '0',
                '--z', '-0.0803',
                '--qx', '0',
                '--qy', '-0.1132',
                '--qz', '0',
                '--qw', '0.9936',
                '--frame-id', 'livox_frame',
                '--child-frame-id', 'base_link'
            ],
            respawn=True,
            respawn_delay=0.5
        ),

        TimerAction(period=2.0, actions=[
            Node(
                package='glim_ros',
                executable='glim_rosnode',
                name='glim_rosnode',
                parameters=[{
                    'config_path': '/workspace/src/acre_slam/config/glim/'
                }],
                output='screen',
                respawn=False
            )
        ])
    ])