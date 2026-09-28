"""One physical owner per sensor; hardware selection is explicit."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory('abot_bringup')) / 'config'
    default_camera_info_url = f'file://{config / "camera_calibration.yaml"}'
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['false'],
                              description='Physical sensors require wall clock time'),
        DeclareLaunchArgument('enable_lidar', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_camera', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('lidar_config', default_value=str(config / 'lidar.yaml')),
        DeclareLaunchArgument('camera_config', default_value=str(config / 'camera.yaml')),
        DeclareLaunchArgument('camera_info_url', default_value=default_camera_info_url),
        DeclareLaunchArgument('filter_config', default_value=str(config / 'laser_filter.yaml')),
        Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
             name='ydlidar_ros2_driver_node',
             parameters=[LaunchConfiguration('lidar_config'), clock], output='screen',
             condition=IfCondition(LaunchConfiguration('enable_lidar'))),
        Node(package='laser_filters', executable='scan_to_scan_filter_chain',
             name='scan_to_scan_filter_chain',
             parameters=[LaunchConfiguration('filter_config'), clock],
             remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')],
             output='screen', condition=IfCondition(LaunchConfiguration('enable_lidar'))),
        Node(package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
             parameters=[LaunchConfiguration('camera_config'), {
                 'camera_info_url': ParameterValue(
                     LaunchConfiguration('camera_info_url'), value_type=str)}, clock],
             remappings=[('image_raw', '/camera/image_raw'),
                         ('camera_info', '/camera/camera_info')],
             output='screen', condition=IfCondition(LaunchConfiguration('enable_camera'))),
    ])
