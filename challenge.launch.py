import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
   
    # Path to YOUR files
    world_path = os.path.join(get_package_share_directory('turtlebot3_challenge'), 'worlds', 'challenge_world.world')
    map_yaml_file = os.path.join(get_package_share_directory('turtlebot3_challenge'), 'maps', 'my_map.yaml')

    return LaunchDescription([
        # 1. Launch Gazebo with your world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([pkg_tb3_gazebo, '/launch/turtlebot3_waffle.launch.py']),
            launch_arguments={'world': world_path}.items(),
        ),
        # 2. Launch Navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([pkg_nav2_bringup, '/launch/bringup_launch.py']),
            launch_arguments={'map': map_yaml_file, 'params_file': os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml')}.items(),
        ),
    ])