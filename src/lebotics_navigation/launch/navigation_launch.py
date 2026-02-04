import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths to your files
    pkg_dir = get_package_share_directory('lebotics_navigation')
    map_file = os.path.join(pkg_dir, 'maps', 'my_map.yaml')
    
    # 2. Path to standard TurtleBot3 & Nav2 launches
    # These packages must be installed via sudo apt install
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'turtlebot3_world.launch.py')
        ])
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch', 'navigation2.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true', 'map': map_file}.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),
        gazebo_launch,
        nav2_launch
    ])

