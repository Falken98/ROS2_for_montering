from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():

    ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='172.31.1.144',
        description='The IP address of the robot'
    )

    gripper_node = Node(
        package='ros2_robotiq_urcap_control',
        namespace='',
        executable='robotiq_urcap_ctrl_ros2',
        name='robotiq_urcap_control_node',
        output='screen',
        parameters=[],
        arguments=[LaunchConfiguration('ip'), '--ros-args', '--log-level', 'debug']
    )
    return LaunchDescription([
        ip_arg,
        gripper_node
    ])