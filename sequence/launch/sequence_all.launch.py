from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():

    sequence_node = Node(
        package='sequence',
        namespace='',
        executable='bt_node',
        name='bt_node',
        output='screen',
        parameters=[],
        arguments=['--ros-args', '--log-level', 'debug']
    )

    griper_ip_arg = DeclareLaunchArgument(
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
        arguments=[LaunchConfiguration('ip'), '--ros-args', '--log-level'] # , 'debug']
    )

    mir_ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='172.31.1.148',
        description='The IP address of the MiR robot'
    )

    mir_node = Node(
        package='mir_communication',
        namespace='',
        executable='mir_node',
        name='mir_communication_node',
        output='screen',
        parameters=[{'ip': LaunchConfiguration('ip')}],
        arguments=['--ros-args', '--log-level'] # , 'debug']
    )

    # Add any other nodes you want to launch here


    return LaunchDescription([
        sequence_node,
        griper_ip_arg,
        gripper_node,
        mir_ip_arg,
        mir_node
        # Add any other nodes you want to launch here
    ])