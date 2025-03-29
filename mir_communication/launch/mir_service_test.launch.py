from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    mir_node = Node(
        package='mir_communication',
        namespace='',
        executable='send_service',
        name='mir_append_client_node',
        output='screen',
        arguments=['--ros-args', '--log-level', 'debug']
    )
    return LaunchDescription([
        mir_node
    ])