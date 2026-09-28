"""Installed P0 composition. Defaults launch only the robot description."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode


def _compose(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def enabled(name):
        return value(name) == 'true'

    if enabled('enable_hardware') and enabled('use_sim_time'):
        raise RuntimeError('Physical base hardware requires use_sim_time:=false.')
    if (enabled('enable_lidar') or enabled('enable_camera')) and enabled('use_sim_time'):
        raise RuntimeError('Physical sensors require use_sim_time:=false.')
    if value('mode') != 'disabled' and not enabled('enable_state_estimation'):
        raise RuntimeError('Navigation requires enable_state_estimation:=true.')
    if value('mode') != 'disabled' and not enabled('enable_velocity'):
        raise RuntimeError('Navigation requires enable_velocity:=true for the final command chain.')

    def include(package, filename, **arguments):
        path = Path(get_package_share_directory(package)) / 'launch' / filename
        arguments.setdefault('use_sim_time', value('use_sim_time'))
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                        launch_arguments=arguments.items())

    actions = [include('abot_description', 'description.launch.py')]
    if enabled('enable_hardware'):
        actions.append(LifecycleNode(
            package='abot_hardware', executable='base_driver', name='abot_hardware',
            namespace='', parameters=[value('hardware_config'), {'use_sim_time': False}],
            output='screen'))
    if enabled('enable_lidar') or enabled('enable_camera'):
        actions.append(include('abot_bringup', 'sensors.launch.py',
                               enable_lidar=value('enable_lidar'), enable_camera=value('enable_camera'),
                               lidar_config=value('lidar_config'), camera_config=value('camera_config'),
                               camera_info_url=value('camera_info_url'),
                               filter_config=value('filter_config')))
    if enabled('enable_state_estimation'):
        actions.append(include('abot_navigation', 'localization.launch.py'))
    if enabled('enable_velocity'):
        actions.append(include('abot_navigation', 'velocity.launch.py',
                               bond_timeout=value('bond_timeout')))
    if value('mode') != 'disabled':
        actions.append(include('abot_navigation', 'navigation.launch.py',
                               mode=value('mode'), map=value('map'),
                               bond_timeout=value('bond_timeout')))
    return actions


def generate_launch_description():
    config = Path(get_package_share_directory('abot_bringup')) / 'config'
    args = [DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
            DeclareLaunchArgument('mode', default_value='disabled',
                                  choices=['disabled', 'mapping', 'localization']),
            DeclareLaunchArgument('map', default_value='', description='Absolute installed/deployment map YAML')]
    args.append(DeclareLaunchArgument('bond_timeout', default_value='4.0',
                                      description='Lifecycle bond timeout in seconds'))
    for name in ('hardware', 'lidar', 'camera', 'state_estimation', 'velocity'):
        args.append(DeclareLaunchArgument('enable_' + name, default_value='false', choices=['true', 'false']))
    for name, filename in [('hardware', 'hardware.yaml'), ('lidar', 'lidar.yaml'),
                           ('camera', 'camera.yaml'), ('filter', 'laser_filter.yaml')]:
        args.append(DeclareLaunchArgument(name + '_config', default_value=str(config / filename)))
    args.append(DeclareLaunchArgument('camera_info_url',
                                      default_value=f'file://{config / "camera_calibration.yaml"}'))
    return LaunchDescription(args + [OpaqueFunction(function=_compose)])
