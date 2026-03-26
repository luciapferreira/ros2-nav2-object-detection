from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='robot_navigation').find('robot_navigation')

    display_launch_path = os.path.join(pkg_share, 'launch', 'display.launch.py')
    joy_params = os.path.join(pkg_share,'config','joystick.yaml')
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')

    model_arg = LaunchConfiguration('model')
    rvizconfig_arg = LaunchConfiguration('rvizconfig')
    use_sim_time = LaunchConfiguration('use_sim_time')

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(display_launch_path),
        launch_arguments={
            'model': model_arg,
            'rvizconfig': rvizconfig_arg,
            'use_sim_time': use_sim_time
        }.items()
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    teleop_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/joy/cmd_vel')],
        output='screen'
    )

    # NOTE:
    # To run keyboard teleop, use:
    # ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/teleop/cmd_vel

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        output='screen',
        remappings=[('cmd_vel_out', '/cmd_vel')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=os.path.join(pkg_share, 'urdf', 'bot_description.urdf'), description='Absolute path to robot model file'),
        DeclareLaunchArgument('rvizconfig', default_value=os.path.join(pkg_share, 'config', 'config.rviz'), description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),

        display_launch,

        # teleop subsystem
        joy_node,
        teleop_joy_node,
        twist_mux_node,
    ])

