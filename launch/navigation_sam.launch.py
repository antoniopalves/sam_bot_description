from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    nav2_bringup_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')
    map_file = os.path.expanduser('~/trsa_two_rooms_map.yaml')
    params_file = os.path.expanduser('~/ros2_ws/src/sam_bot_description/config/nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'True',
                'params_file': params_file,
                'slam': 'False',
                'use_composition': 'False',
                'autostart': 'True'
            }.items()
        )
    ])
