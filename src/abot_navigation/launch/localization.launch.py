"""Local odometry filter. EKF is the sole odom -> base_footprint TF owner."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('abot_navigation'), 'config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
            name='imu_filter', output='screen',
            parameters=[os.path.join(config, 'imu_filter.yaml'),
                        {'use_sim_time': use_sim_time}],
            remappings=[('imu/data_raw', '/imu/data_raw'),
                        ('imu/data', '/imu/data')],
        ),
        Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node', output='screen',
            parameters=[os.path.join(config, 'ekf.yaml'),
                        {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odom')],
        ),
    ])
