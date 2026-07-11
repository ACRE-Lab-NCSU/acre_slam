from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

# legkilo_node copies whatever --config_file path it's given into its own
# result/temp/ folder on startup. That copy is a cross-device link if the
# source is under a bind-mounted path like /workspace, and
# boost::filesystem::copy_file doesn't fall back - it just throws and the
# node SIGABRTs. Stage the config onto the container's own filesystem first.
STAGED_CONFIG = '/tmp/kilo_config.yaml'


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='/workspace/src/acre_slam/config/kilo/kilo_config.yaml'
        ),

        # config_file is a LaunchConfiguration substitution - it must be
        # passed as its own cmd list element (becomes $1) rather than
        # embedded in the script string, or launch never resolves it.
        ExecuteProcess(
            cmd=['bash', '-c',
                 f'cp "$1" {STAGED_CONFIG} && '
                 f'exec ros2 run legkilo legkilo_node --config_file {STAGED_CONFIG}',
                 'bash', config_file],
            output='screen'
        )
    ])
