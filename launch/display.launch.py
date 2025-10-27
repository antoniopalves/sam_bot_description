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
    world_path = os.path.join(pkg_share, 'worlds', 'my_world.sdf')

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', LaunchConfiguration('model')]),
            'use_sim_time': True
        }]
    )

    # Joint State Publisher (non-GUI)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', default_model_path]),
            'use_sim_time': True
        }],
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Gazebo process
    gazebo_process = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path
        ],
        output='screen'
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'sam_bot',
            '-x', '0',
            '-y', '0',
            '-z', '0.0',
            '-Y', '3.14159'  # 180 degrees rotation around Z
        ],
        output='screen'
    )


    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Path to robot model'
        ),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Path to RViz config'
        ),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        gazebo_process,
        spawn_entity,
        rviz_node
    ])
