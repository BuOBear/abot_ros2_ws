"""Starts the mission service only; navigation requires /mission/start."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('route_file', default_value=''),
        DeclareLaunchArgument('map_id', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('preparing_timeout_sec', default_value='10.0'),
        Node(package='abot_mission', executable='mission_node', output='screen',
             parameters=[{
                 'route_file': LaunchConfiguration('route_file'),
                 'map_id': LaunchConfiguration('map_id'),
                 'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'preparing_timeout_sec': ParameterValue(
                     LaunchConfiguration('preparing_timeout_sec'), value_type=float),
             }]),
    ])
