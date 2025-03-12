from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_robotiq_urcap_control',
            namespace='',
            executable='robotiq_urcap_ctrl_ros2',
            name='robotiq_urcap_control_node'
        ),
        Node(
            package='ros2_robotiq_urcap_control',
            namespace='',
            executable='open_close_node',
            name='robotiq_urcap_test_node'
        )
    ])