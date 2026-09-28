"""Mutually exclusive mapping or map localization plus Nav2 motion servers.

The installed Humble nav2_bringup/navigation_launch.py contains a velocity
smoother that writes cmd_vel. Starting the servers here routes all Nav2 motion
to /cmd_vel/nav, leaving exactly one smoother in velocity.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


NAV_NODES = [
    ('nav2_controller', 'controller_server'),
    ('nav2_smoother', 'smoother_server'),  # path smoother, not velocity smoother
    ('nav2_planner', 'planner_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
    ('nav2_waypoint_follower', 'waypoint_follower'),
]


def launch_setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    map_path = LaunchConfiguration('map').perform(context)
    if mode not in ('mapping', 'localization'):
        raise RuntimeError("mode must be 'mapping' or 'localization'")
    if mode == 'localization' and not os.path.isfile(map_path):
        raise RuntimeError('localization requires an existing map YAML path: ' + map_path)

    config = os.path.join(get_package_share_directory('abot_navigation'), 'config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    bond_timeout = ParameterValue(LaunchConfiguration('bond_timeout'), value_type=float)
    rewrites = {'use_sim_time': use_sim_time}
    if mode == 'localization':
        rewrites['yaml_filename'] = map_path
    params = ParameterFile(RewrittenYaml(
        source_file=os.path.join(config, 'nav2.yaml'),
        param_rewrites=rewrites, convert_types=True), allow_substs=True)
    nodes = []

    if mode == 'mapping':
        nodes += [
            Node(package='slam_toolbox', executable='sync_slam_toolbox_node',
                 name='slam_toolbox', output='screen',
                 parameters=[os.path.join(config, 'slam.yaml'),
                             {'use_sim_time': use_sim_time}]),
            Node(package='nav2_map_server', executable='map_saver_server',
                 name='map_saver', output='screen', parameters=[params]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_mapping', output='screen',
                 parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                              'bond_timeout': bond_timeout,
                              'node_names': ['map_saver']}]),
        ]
    else:
        nodes += [
            Node(package='nav2_map_server', executable='map_server',
                 name='map_server', output='screen', parameters=[params]),
            Node(package='nav2_amcl', executable='amcl', name='amcl',
                 output='screen', parameters=[params]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                              'bond_timeout': bond_timeout,
                              'node_names': ['map_server', 'amcl']}]),
        ]

    for package, executable in NAV_NODES:
        nodes.append(Node(
            package=package, executable=executable, name=executable,
            output='screen', parameters=[params],
            remappings=[('cmd_vel', '/cmd_vel/nav'),
                        ('cmd_vel_nav', '/cmd_vel/nav')],
        ))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                     'bond_timeout': bond_timeout,
                     'node_names': [name for _, name in NAV_NODES]}],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='localization',
                              description='mapping or localization'),
        DeclareLaunchArgument('map', default_value='',
                              description='Existing map YAML in localization mode'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('bond_timeout', default_value='4.0',
                              description='Lifecycle bond timeout in seconds'),
        OpaqueFunction(function=launch_setup),
    ])
