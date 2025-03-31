from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_robotiq_urcap_control',
            namespace='',
            executable='open_close_node',
            name='robotiq_urcap_test_node',
            arguments=['--ros-args', '--log-level', 'debug']
        )
    ])