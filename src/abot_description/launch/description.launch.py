"""Publish the abot URDF frames from the installed description package."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('abot_description')
    model = os.path.join(share, 'urdf', 'abot.urdf.xacro')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', model]), value_type=str),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ),
    ])
