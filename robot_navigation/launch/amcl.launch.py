import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_navigation')
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'map', 'two_rooms_map.yaml'),
        description='Full path to map file to load'
    )

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file to use'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'True'
        }.items()
    )

    navigate_server_node = Node(
        package='robot_navigation',
        executable='navigate_server',
        name='navigate_to_goal',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_map_arg)
    ld.add_action(declare_params_file_arg)
    ld.add_action(declare_use_sim_time_arg)

    ld.add_action(nav2_bringup_cmd)
    ld.add_action(navigate_server_node)

    return ld