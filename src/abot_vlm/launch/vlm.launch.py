from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        Node(
            package='abot_vlm', executable='vlm_action', name='vlm_action',
            output='screen',
            parameters=[{'image_topic': LaunchConfiguration('image_topic')}],
        ),
    ])
