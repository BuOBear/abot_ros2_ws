"""Color detector and yaw-only follower; camera and velocity chain are separate."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    detector_config = Path(get_package_share_directory('abot_perception')) / 'config' / 'colors.yaml'
    follower_config = Path(get_package_share_directory('abot_tracking')) / 'config' / 'follower.yaml'
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_detector', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_follower', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('detector_config', default_value=str(detector_config)),
        DeclareLaunchArgument('follower_config', default_value=str(follower_config)),
        Node(package='abot_perception', executable='color_detector', name='color_detector',
             parameters=[LaunchConfiguration('detector_config'), clock],
             output='screen', condition=IfCondition(LaunchConfiguration('enable_detector'))),
        Node(package='abot_tracking', executable='vision_follower', name='vision_follower',
             parameters=[LaunchConfiguration('follower_config'), clock],
             output='screen', condition=IfCondition(LaunchConfiguration('enable_follower'))),
    ])
