#!/usr/bin/env python3
"""Installed, camera-free ROS 2 smoke for tag, template and person sources.

Run after sourcing /opt/ros/humble/setup.bash and install/setup.bash. The
synthetic images exercise the actual detector and follower nodes through DDS.
No camera, robot base, control mode, or final /cmd_vel publisher is started.
"""

import argparse
from importlib.util import module_from_spec, spec_from_file_location
import os
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=119,
                        help='first of three isolated ROS_DOMAIN_ID values')
    args = parser.parse_args()
    if not 0 <= args.domain <= 229:
        parser.error('--domain must leave room for two more domain IDs')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    profile = Path(__file__).resolve().parents[1] / 'deployment' / 'fastdds-local-test.xml'
    if profile.is_file():
        os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', str(profile))

    import cv2
    import numpy as np
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Twist
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch.utilities import perform_substitutions
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from vision_msgs.msg import Detection2DArray
    import yaml
    from abot_perception.person_detector import PersonDetector
    from abot_perception.tag_detector import TagDetector, tag_dictionary
    from abot_perception.template_detector import TemplateDetector
    from abot_tracking.vision_follower import VisionFollower

    bridge = CvBridge()
    dictionary = tag_dictionary('DICT_APRILTAG_36h11')
    marker = cv2.aruco.drawMarker(dictionary, 7, 160)
    tag_image = np.full((300, 420, 3), 255, dtype=np.uint8)
    tag_image[70:230, 190:350] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    template_path = (Path(get_package_share_directory('abot_perception')) /
                     'templates' / '10.png')
    template = cv2.imread(str(template_path), cv2.IMREAD_COLOR)
    if template is None:
        raise RuntimeError(f'Installed template asset missing: {template_path}')
    template_image = np.full((480, 640, 3), 150, dtype=np.uint8)
    height, width = template.shape[:2]
    template_image[100:100 + height, 280:280 + width] = template
    people_blank = np.zeros((240, 320, 3), dtype=np.uint8)

    def check_installed_launch():
        """Execute the installed OpaqueFunction, including source selection."""
        path = (Path(get_package_share_directory('abot_tracking')) /
                'launch' / 'vision_follow.launch.py')
        spec = spec_from_file_location('installed_vision_follow_launch', path)
        module = module_from_spec(spec)
        spec.loader.exec_module(module)

        def expanded(context, values):
            return {perform_substitutions(context, key):
                    yaml.safe_load(perform_substitutions(context, value))
                    if isinstance(value, tuple) else value
                    for key, value in values.items()}

        cases = [
            ('tag', {'tag_id': '7'}, 'tag_detector', '/perception/tags',
             'tag:apriltag_36h11:7', 0.3, 0.5),
            ('template', {'template_id': '10'}, 'template_detector',
             '/perception/templates', 'template:10', 0.5, 0.5),
            ('person', {}, 'person_detector', '/perception/people',
             'person', 0.8, 0.8),
        ]
        for (source, extra, executable, topic, target_class,
             detection_timeout, camera_timeout) in cases:
            context = LaunchContext()
            context.launch_configurations.update(source=source, **extra)
            description = module.generate_launch_description()
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            nodes = [node for action in description.entities
                     if isinstance(action, OpaqueFunction)
                     for node in action.execute(context)]
            if len(nodes) != 2:
                raise RuntimeError(f'{source}: expected one detector and one follower')
            detector, follower = nodes
            if (detector.node_package != 'abot_perception' or
                    detector.node_executable != executable or
                    follower.node_package != 'abot_tracking' or
                    follower.node_executable != 'vision_follower'):
                raise RuntimeError(f'{source}: launch selected the wrong executables')
            # Humble's Node action keeps normalized parameters here until it
            # emits the temporary ROS parameter file during execution.
            detector_params = expanded(context, detector._Node__parameters[-1])
            follower_params = expanded(context, follower._Node__parameters[-1])
            if (detector_params['image_topic'] != '/camera/image_raw' or
                    follower_params['camera_info_topic'] != '/camera/camera_info' or
                    follower_params['detections_topic'] != topic or
                    follower_params['target_class'] != target_class or
                    follower_params['detection_timeout'] != detection_timeout or
                    follower_params['camera_info_timeout'] != camera_timeout):
                raise RuntimeError(f'{source}: installed launch parameter mismatch')
            if source == 'tag' and detector_params['dictionary'] != 'DICT_APRILTAG_36h11':
                raise RuntimeError('tag: wrong installed dictionary')
            if source == 'template':
                installed_template = Path(detector_params['template_dir']) / '10.png'
                if not installed_template.is_file():
                    raise RuntimeError(f'template: asset missing: {installed_template}')
            print(f'PASS: installed launch source={source}: {executable} -> '
                  f'{topic} -> {target_class}; timeouts '
                  f'{detection_timeout}/{camera_timeout} s', flush=True)

    def run_source(index, label, detector_type, topic, target_class, image,
                   blank, expected_positive):
        rclpy.init(args=['--ros-args', '-p', f'target_class:={target_class}',
                         '-p', f'detections_topic:={topic}'],
                    domain_id=args.domain + index)
        detector = follower = probe = executor = None
        try:
            detector = detector_type()
            follower = VisionFollower()
            probe = rclpy.create_node(f'visual_sources_probe_{label}')
            assert follower.get_parameter('target_class').value == target_class
            assert follower.get_parameter('detections_topic').value == topic
            executor = SingleThreadedExecutor()
            for node in (detector, follower, probe):
                executor.add_node(node)
            image_pub = probe.create_publisher(Image, '/camera/image_raw',
                                               qos_profile_sensor_data)
            info_pub = probe.create_publisher(CameraInfo, '/camera/camera_info',
                                              qos_profile_sensor_data)
            observations = {'detections': [], 'commands': []}
            probe.create_subscription(
                Detection2DArray, topic,
                lambda msg: observations['detections'].append((time.monotonic(), msg)),
                qos_profile_sensor_data)
            probe.create_subscription(
                Twist, '/cmd_vel/tracking',
                lambda msg: observations['commands'].append((time.monotonic(), msg)), 10)

            def send_frame(pixels):
                stamp = probe.get_clock().now().to_msg()
                camera = CameraInfo()
                camera.header.stamp = stamp
                camera.header.frame_id = 'camera_optical_frame'
                camera.height, camera.width = pixels.shape[:2]
                message = bridge.cv2_to_imgmsg(pixels, encoding='bgr8')
                message.header = camera.header
                info_pub.publish(camera)
                image_pub.publish(message)

            def until(predicate, pixels, description, timeout=8.0):
                deadline = time.monotonic() + timeout
                next_frame = 0.0
                while time.monotonic() < deadline:
                    if pixels is not None and time.monotonic() >= next_frame:
                        send_frame(pixels)
                        next_frame = time.monotonic() + 0.23
                    executor.spin_once(timeout_sec=0.03)
                    if predicate():
                        print(f'PASS: {label}: {description}', flush=True)
                        return
                raise RuntimeError(f'Timed out: {label}: {description}')

            def has_positive_after(start):
                return any(at > start and msg.header.frame_id == 'camera_optical_frame'
                           and any(d.results and d.results[0].hypothesis.class_id == target_class
                                   for d in msg.detections)
                           for at, msg in observations['detections'])

            def has_empty_after(start):
                return any(at > start and msg.header.frame_id == 'camera_optical_frame'
                           and not msg.detections
                           for at, msg in observations['detections'])

            def has_command_after(start, moving):
                return any(at > start and
                           (msg.angular.z < -0.05 if moving else msg.angular.z == 0.0)
                           and msg.linear.x == 0.0
                           for at, msg in observations['commands'])

            until(lambda: image_pub.get_subscription_count() > 0 and
                  info_pub.get_subscription_count() > 0 and
                  len(probe.get_publishers_info_by_topic(topic)) == 1,
                  blank, 'installed nodes discover private source topic')

            if expected_positive:
                positive_started = time.monotonic()
                until(lambda: has_positive_after(positive_started), image,
                      f'{target_class} detection from synthetic image')
                positive_at = next(at for at, msg in reversed(observations['detections'])
                                   if at > positive_started and any(
                                       d.results and d.results[0].hypothesis.class_id == target_class
                                       for d in msg.detections))
                until(lambda: has_command_after(positive_at, True), image,
                      'selected follower turns toward target through /cmd_vel/tracking')
                clear_started = time.monotonic()
            else:
                clear_started = time.monotonic()

            until(lambda: has_empty_after(clear_started), blank,
                  'blank image produces a stamped empty detection')
            empty_at = next(at for at, msg in reversed(observations['detections'])
                            if at > clear_started and not msg.detections)
            until(lambda: has_command_after(empty_at, False), blank,
                  'empty detection yields zero tracking command')
            stream_stopped = time.monotonic()
            until(lambda: time.monotonic() - stream_stopped > 0.8 and
                  observations['commands'] and
                  observations['commands'][-1][0] > stream_stopped + 0.6 and
                  observations['commands'][-1][1].angular.z == 0.0,
                  None, 'stopped image stream keeps publishing zero')
            if probe.get_publishers_info_by_topic('/cmd_vel'):
                raise RuntimeError(f'{label}: unexpected final /cmd_vel publisher')
            print(f'PASS: {label}: no final /cmd_vel publisher', flush=True)
        finally:
            if executor is not None:
                executor.shutdown()
            for node in (probe, follower, detector):
                if node is not None:
                    node.destroy_node()
            rclpy.try_shutdown()

    run_source(0, 'tag', TagDetector, '/perception/tags',
               'tag:apriltag_36h11:7', tag_image,
               np.full_like(tag_image, 255), True)
    run_source(1, 'template', TemplateDetector, '/perception/templates',
               'template:10', template_image,
               np.full_like(template_image, 150), True)
    run_source(2, 'person', PersonDetector, '/perception/people',
               'person', people_blank, people_blank, False)
    check_installed_launch()
    print('NOTE: person positive detection requires a representative real-person image; '
          'HOG blank and lost-stream behavior passed.', flush=True)


if __name__ == '__main__':
    main()
