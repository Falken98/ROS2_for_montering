from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mir_communication',
            namespace='',
            executable='mir_node',
            name='mir_communication_node'
        )
    ])