from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, LogInfo
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pkg_share = FindPackageShare('robot_navigation').find('robot_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_pkg = FindPackageShare('slam_toolbox').find('slam_toolbox')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    nav2_pkg = FindPackageShare('nav2_bringup').find('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        }.items(),
    )

    # NOTE: Store the map
    # ros2 run nav2_map_server map_saver_cli -f two_rooms_map

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        slam_launch,
        nav2_launch,
    ])
