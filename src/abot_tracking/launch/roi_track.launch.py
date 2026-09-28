"""Run ROI observations alone by default, with an optional single follower."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(get_package_share_directory('abot_tracking')) / 'config' / 'follower.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('seed_topic', default_value='/roi_lk_tracker/roi_seed'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_follower', default_value='false',
                              choices=['true', 'false']),
        Node(package='abot_tracking', executable='roi_lk_tracker', name='roi_lk_tracker',
             parameters=[{'image_topic': LaunchConfiguration('image_topic'),
                          'seed_topic': LaunchConfiguration('seed_topic'),
                          'use_sim_time': ParameterValue(
                              LaunchConfiguration('use_sim_time'), value_type=bool)}],
             output='screen'),
        Node(package='abot_tracking', executable='vision_follower', name='vision_follower',
             parameters=[str(config), {'target_class': 'roi',
                                       'detections_topic': '/roi_lk_tracker/roi',
                                       'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                                       'use_sim_time': ParameterValue(
                                           LaunchConfiguration('use_sim_time'), value_type=bool)}],
             condition=IfCondition(LaunchConfiguration('enable_follower')),
             output='screen'),
    ])
