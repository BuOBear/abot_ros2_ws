from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('abot_perception'))
    config = package_share / 'config' / 'fire.yaml'
    return LaunchDescription([
        Node(
            package='abot_perception',
            executable='fire_detector',
            name='fire_detector',
            output='screen',
            parameters=[str(config)],
        ),
    ])
