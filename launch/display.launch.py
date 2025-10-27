from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'sam_bot.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    default_world_path = os.path.join(pkg_share, 'worlds', 'trsa_two_rooms.world')

    # Launch arguments
    gui_arg = DeclareLaunchArgument('gui', default_value='True', description='Enable joint_state_publisher_gui')
    model_arg = DeclareLaunchArgument('model', default_value=default_model_path, description='Path to robot model')
    rviz_arg = DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path, description='Path to RViz config')
    world_arg = DeclareLaunchArgument('world', default_value=default_world_path, description='Path to world file')

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', LaunchConfiguration('model')]),
            'use_sim_time': True
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', LaunchConfiguration('model')]),
            'use_sim_time': True
        }],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', ''],  # abre RViz sem ficheiro de configuração
        output='screen'
    )


    gazebo_process = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            LaunchConfiguration('world')
        ],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'sam_bot',
            '-x', '0', '-y', '0', '-z', '0.0',
            '-Y', '3.14159'
        ],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        world_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        gazebo_process,
        spawn_entity,
        rviz_node
    ])
