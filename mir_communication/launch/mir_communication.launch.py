from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():

    ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='192.168.12.20',
        description='The IP address of the MiR robot'
    )

    mir_node = Node(
        package='mir_communication',
        namespace='',
        executable='mir_node',
        name='mir_communication_node',
        output='screen',
        parameters=[{'ip': LaunchConfiguration('ip')}],
        arguments=['--ros-args', '--log-level', 'debug']
    )
    return LaunchDescription([
        ip_arg,
        mir_node
    ])