"""Select one visual source and one target for guarded yaw-only tracking."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def selected_nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    source = value('source')
    topic = {'color': '/perception/colors', 'tag': '/perception/tags',
             'template': '/perception/templates', 'person': '/perception/people'}[source]
    detector = {'color': 'color_detector', 'tag': 'tag_detector',
                'template': 'template_detector', 'person': 'person_detector'}[source]
    config = Path(get_package_share_directory('abot_perception')) / 'config' / 'colors.yaml'
    follower_config = Path(get_package_share_directory('abot_tracking')) / 'config' / 'follower.yaml'
    dictionary = value('tag_dictionary')
    target_class = {
        'color': value('color'),
        'tag': f'tag:{dictionary.removeprefix("DICT_").lower()}:{value("tag_id")}',
        'template': f'template:{value("template_id")}',
        'person': 'person',
    }[source]
    # Match the detector's default processing interval. The value still
    # bounds how long a lost target may command yaw and needs bench tuning.
    default_detection_timeout = {'color': 0.3, 'tag': 0.3,
                                 'template': 0.5, 'person': 0.8}[source]
    timeout_arg = value('detection_timeout')
    detection_timeout = (default_detection_timeout if timeout_arg == 'auto'
                         else float(timeout_arg))
    camera_timeout_arg = value('camera_info_timeout')
    camera_info_timeout = (max(0.5, detection_timeout)
                           if camera_timeout_arg == 'auto'
                           else float(camera_timeout_arg))
    if (not math.isfinite(detection_timeout) or detection_timeout <= 0 or
            not math.isfinite(camera_info_timeout) or camera_info_timeout <= 0):
        raise ValueError('Visual timeout must be finite and positive')
    detector_params = {
        'image_topic': value('image_topic'),
        'use_sim_time': value('use_sim_time') == 'true',
    }
    if source == 'tag':
        detector_params['dictionary'] = dictionary
    elif source == 'template':
        detector_params['template_id'] = value('template_id')
        detector_params['template_ids'] = value('template_ids')
        detector_params['template_dir'] = value('template_dir')
        if detector_params['template_ids']:
            selected_ids = [item.strip() for item in
                            detector_params['template_ids'].split(',')]
            if (value('enable_follower') == 'true' and
                    detector_params['template_id'] not in selected_ids):
                raise ValueError('template_id target must be included in template_ids')
    params = [str(config), detector_params] if source == 'color' else [detector_params]
    return [
        Node(package='abot_perception', executable=detector, name=detector,
             parameters=params, output='screen',
             condition=IfCondition(LaunchConfiguration('enable_detector'))),
        Node(package='abot_tracking', executable='vision_follower', name='vision_follower',
             parameters=[str(follower_config), {
                 'target_class': target_class,
                 'detections_topic': topic,
                 'camera_info_topic': value('camera_info_topic'),
                 'camera_info_timeout': camera_info_timeout,
                 'detection_timeout': detection_timeout,
                 'use_sim_time': value('use_sim_time') == 'true',
             }], output='screen',
             condition=IfCondition(LaunchConfiguration('enable_follower'))),
    ]


def generate_launch_description():
    default_templates = str(Path(get_package_share_directory('abot_perception')) / 'templates')
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='color',
                              choices=['color', 'tag', 'template', 'person']),
        DeclareLaunchArgument('color', default_value='red',
                              choices=['red', 'yellow', 'green']),
        DeclareLaunchArgument('tag_dictionary', default_value='DICT_APRILTAG_36h11'),
        DeclareLaunchArgument('tag_id', default_value='0'),
        DeclareLaunchArgument('template_id', default_value='10'),
        DeclareLaunchArgument('template_ids', default_value=''),
        DeclareLaunchArgument('template_dir', default_value=default_templates),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('detection_timeout', default_value='auto'),
        DeclareLaunchArgument('camera_info_timeout', default_value='auto'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_detector', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('enable_follower', default_value='true',
                              choices=['true', 'false']),
        OpaqueFunction(function=selected_nodes),
    ])
