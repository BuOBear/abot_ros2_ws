"""Launch the scan follower without granting tracking velocity authority."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('abot_tracking')) / 'config' / 'lidar_follower.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan_filtered'),
        DeclareLaunchArgument('enable_linear_motion', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              choices=['true', 'false']),
        Node(package='abot_tracking', executable='lidar_follower', name='lidar_follower',
             parameters=[str(config), {
                 'scan_topic': LaunchConfiguration('scan_topic'),
                 'enable_linear_motion': LaunchConfiguration('enable_linear_motion'),
                 'use_sim_time': LaunchConfiguration('use_sim_time'),
             }], output='screen'),
    ])
