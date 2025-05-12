from launch import LaunchDescription
from launch_ros.actions import Node

from pathlib import Path
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # moveit_config = (
    #     MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
    #     .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": 'ur5e'})
    #     .to_moveit_configs()
    # )

    sequence_node = Node(
        package='sequence',
        namespace='',
        executable='bt_node',
        name='bt_node',
        output='screen',
        parameters=[
            # moveit_config.robot_description,
            # moveit_config.robot_description_semantic,
            # moveit_config.robot_description_kinematics,
            # moveit_config.planning_pipelines,
            # moveit_config.joint_limits,
        ],
        arguments=['--ros-args']
    )

    # move_group_node = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[
    #         moveit_config.to_dict(),
    #     ],
    # )

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

    # moveit_node = Node(
    #     package='move_it_ur5e',
    #     namespace='',
    #     executable='moveit_action_server',
    #     name='moveit_ur5e_node'
    # )

    # Add any other nodes you want to launch here


    return LaunchDescription([
        sequence_node,
        griper_ip_arg,
        gripper_node,
        mir_ip_arg,
        mir_node,
        # move_group_node
        # Add any other nodes you want to launch here
    ])