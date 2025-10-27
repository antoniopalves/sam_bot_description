from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg = get_package_share_directory('sam_bot_description')
    world = os.path.join(pkg, 'worlds', 'trsa_two_rooms.world')
    xacro_path = os.path.join(pkg, 'urdf', 'sam_bot.xacro')

    # Processa o ficheiro Xacro -> URDF XML
    robot_description_config = xacro.process_file(xacro_path).toxml()

    return LaunchDescription([
        # Lança o Gazebo com o mundo indicado
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'),
                             'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world}.items(),
        ),

        # Publica a descrição do robô (necessário para o spawn)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),

        # Cria o robô dentro do mundo (spawn)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'sam_bot'],
            output='screen'
        ),
    ])
