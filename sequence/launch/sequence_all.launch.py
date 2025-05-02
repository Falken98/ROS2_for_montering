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
        arguments=['--ros-args']
    )

    griper_ip_arg = DeclareLaunchArgument(
        'griper_ip',
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
        arguments=[LaunchConfiguration('griper_ip'), '--ros-args'] # , '--log-level', 'debug']
    )

    mir_ip_arg = DeclareLaunchArgument(
        'mir_ip',
        default_value='172.31.1.148',
        description='The IP address of the MiR robot'
    )

    mir_node = Node(
        package='mir_communication',
        namespace='',
        executable='mir_node',
        name='mir_communication_node',
        output='screen',
        parameters=[{'ip': LaunchConfiguration('mir_ip')}],
        arguments=['--ros-args'] # , '--log-level', 'debug']
    )

    moveit_node = Node(
        package='move_it_ur5e',
        namespace='',
        executable='moveit_action_server',
        name='moveit_ur5e_node'
    )

    # Add any other nodes you want to launch here


    return LaunchDescription([
        sequence_node,
        griper_ip_arg,
        gripper_node,
        mir_ip_arg,
        mir_node,
        moveit_node
        # Add any other nodes you want to launch here
    ])