from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os

def generate_launch_description():
    # Package where your map.launch.py lives
    map_pkg = FindPackageShare('robot_navigation').find('robot_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include the map launch file
    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(map_pkg, 'launch', 'map.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    explorer = TimerAction(
        period=10.0,  # wait 10 seconds before starting explorer
        actions=[
            Node(
                package='frontier_explorer',
                executable='explorer',
                name='explorer',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        map_launch,
        explorer
    ])
