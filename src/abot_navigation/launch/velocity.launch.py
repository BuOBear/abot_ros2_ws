"""Authority -> mux -> smoother -> collision monitor -> final safety gate."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = os.path.join(get_package_share_directory('abot_navigation'),
                          'config', 'velocity.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    bond_timeout = ParameterValue(LaunchConfiguration('bond_timeout'), value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('bond_timeout', default_value='4.0',
                              description='Lifecycle bond timeout in seconds'),
        Node(package='abot_navigation', executable='velocity_authority',
             name='velocity_authority', output='screen',
             parameters=[params, {'use_sim_time': use_sim_time}]),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux',
             output='screen', parameters=[params, {'use_sim_time': use_sim_time}],
             remappings=[('cmd_vel_out', '/cmd_vel/muxed')]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=[params, {'use_sim_time': use_sim_time}],
             remappings=[('cmd_vel', '/cmd_vel/muxed'),
                         ('cmd_vel_smoothed', '/cmd_vel/smoothed')]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen',
             parameters=[params, {'use_sim_time': use_sim_time}]),
        Node(package='abot_navigation', executable='velocity_gate',
             name='velocity_gate', output='screen',
             parameters=[params, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_velocity', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'bond_timeout': bond_timeout,
                          'node_names': ['velocity_smoother', 'collision_monitor']}]),
    ])
