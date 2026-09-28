"""Physical ABOT components plus opt-in wrist camera, YOLO11 and servo board."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from jy_real_robot.description import base_description
from jy_real_robot.core import local_weights
from jy_real_robot.servo import load_profile


def compose(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    def on(name):
        return value('enable_' + name) == 'true'

    def include(package, filename, **arguments):
        path = Path(get_package_share_directory(package)) / 'launch' / filename
        arguments['use_sim_time'] = 'false'
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                        launch_arguments=arguments.items())

    if value('mode') != 'disabled' and not (on('state_estimation') and on('velocity')):
        raise ValueError('navigation requires enable_state_estimation and enable_velocity')
    # Check dependencies before starting any processes.
    if on('arm'):
        load_profile(value('arm_profile'))
    if on('yolo'):
        local_weights(value('weights'))
    model = Path(get_package_share_directory('abot_description')) / 'urdf/abot.urdf.xacro'
    actions = [Node(package='robot_state_publisher', executable='robot_state_publisher',
                    parameters=[{'robot_description': base_description(model), 'use_sim_time': False}],
                    output='screen')]
    if on('hardware'):
        actions.append(LifecycleNode(package='abot_hardware', executable='base_driver',
                       name='abot_hardware', namespace='', output='screen',
                       parameters=[value('hardware_config'), {'use_sim_time': False}]))
    if on('lidar') or on('camera'):
        actions.append(include('abot_bringup', 'sensors.launch.py',
            enable_lidar=value('enable_lidar'), enable_camera=value('enable_camera'),
            lidar_config=value('lidar_config'), filter_config=value('filter_config'),
            camera_config=value('camera_config'), camera_info_url=value('camera_info_url')))
    if on('state_estimation'):
        actions.append(include('abot_navigation', 'localization.launch.py'))
    if on('velocity'):
        actions.append(include('abot_navigation', 'velocity.launch.py',
                               bond_timeout=value('bond_timeout')))
    if value('mode') != 'disabled':
        actions.append(include('abot_navigation', 'navigation.launch.py',
                               mode=value('mode'), map=value('map'),
                               bond_timeout=value('bond_timeout')))
    if on('yolo'):
        actions.append(Node(package='jy_real_robot', executable='yolo11_detector', output='screen',
            parameters=[{'weights': value('weights'), 'device': value('inference_device'),
                         'use_sim_time': False}]))
    if on('arm'):
        actions.append(Node(package='jy_real_robot', executable='servo_arm', output='screen',
            parameters=[{'profile': value('arm_profile'), 'port': value('arm_port'),
                         'use_sim_time': False}]))
    return actions


def generate_launch_description():
    base_config = Path(get_package_share_directory('abot_bringup')) / 'config'
    config = Path(get_package_share_directory('jy_real_robot')) / 'config'
    args = []
    for component in ('hardware', 'lidar', 'camera', 'state_estimation', 'velocity', 'yolo', 'arm'):
        args.append(DeclareLaunchArgument('enable_' + component, default_value='false',
                                         choices=['true', 'false']))
    defaults = {'mode': 'disabled', 'map': '', 'bond_timeout': '4.0',
                'arm_port': '/dev/jy_arm',
                'arm_profile': str(config / 'arm_profile.yaml'),
                'camera_config': str(config / 'camera.yaml'), 'camera_info_url': '',
                'weights': '', 'inference_device': 'cpu'}
    for key, filename in [('hardware', 'hardware.yaml'), ('lidar', 'lidar.yaml'),
                          ('filter', 'laser_filter.yaml')]:
        defaults[key + '_config'] = str(base_config / filename)
    for name, default in defaults.items():
        extra = {'choices': ['disabled', 'mapping', 'localization']} if name == 'mode' else {}
        args.append(DeclareLaunchArgument(name, default_value=default, **extra))
    return LaunchDescription(args + [OpaqueFunction(function=compose)])
