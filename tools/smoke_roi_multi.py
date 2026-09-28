#!/usr/bin/env python3
"""Installed, device-free graph smoke for multi-template and seeded ROI nodes."""

import argparse
import os
import inspect
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=154,
                        help='first of three isolated ROS_DOMAIN_ID values')
    options = parser.parse_args()
    if not 0 <= options.domain <= 229:
        parser.error('--domain must leave room for two more domain IDs')
    workspace = Path(__file__).resolve().parents[1]
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE',
                          str(workspace / 'deployment' / 'fastdds-local-test.xml'))

    import cv2
    from cv_bridge import CvBridge
    import numpy as np
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import CameraInfo, Image
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    import yaml

    from abot_perception.template_detector import TemplateDetector
    from abot_tracking.roi_lk_tracker import RoiLKTracker
    from abot_tracking.vision_follower import VisionFollower

    assert '/install/' in inspect.getfile(TemplateDetector)
    assert '/install/' in inspect.getfile(RoiLKTracker)
    bridge = CvBridge()

    def check_installed_launch():
        path = (Path(get_package_share_directory('abot_tracking')) /
                'launch' / 'vision_follow.launch.py')
        spec = spec_from_file_location('installed_vision_follow_launch', path)
        module = module_from_spec(spec)
        spec.loader.exec_module(module)

        def selected(template_id, enable_follower='true'):
            context = LaunchContext()
            context.launch_configurations.update(
                source='template', template_id=template_id,
                template_ids='10,20', enable_follower=enable_follower)
            description = module.generate_launch_description()
            for action in description.entities:
                if isinstance(action, DeclareLaunchArgument):
                    action.execute(context)
            return context, [node for action in description.entities
                             if isinstance(action, OpaqueFunction)
                             for node in action.execute(context)]

        context, nodes = selected('20')
        def expanded(values):
            return {
                perform_substitutions(context, name):
                yaml.safe_load(perform_substitutions(context, value))
                if isinstance(value, tuple) else value
                for name, value in values.items()
            }
        detector_params = expanded(nodes[0]._Node__parameters[-1])
        follower_params = expanded(nodes[1]._Node__parameters[-1])
        if (detector_params['template_ids'] != '10,20' or
                follower_params['target_class'] != 'template:20'):
            raise RuntimeError('Installed launch lost template list or selected target')
        try:
            selected('30')
        except ValueError as exc:
            if 'included' not in str(exc):
                raise
        else:
            raise RuntimeError('Installed launch accepted a target absent from template_ids')
        selected('30', enable_follower='false')
        roi_path = (Path(get_package_share_directory('abot_tracking')) /
                    'launch' / 'roi_track.launch.py')
        roi_spec = spec_from_file_location('installed_roi_track_launch', roi_path)
        roi_module = module_from_spec(roi_spec)
        roi_spec.loader.exec_module(roi_module)
        roi_context = LaunchContext()
        roi_description = roi_module.generate_launch_description()
        for action in roi_description.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(roi_context)
        follower_nodes = [action for action in roi_description.entities
                          if isinstance(action, Node) and
                          action.node_executable == 'vision_follower']
        if (len(follower_nodes) != 1 or
                follower_nodes[0].condition.evaluate(roi_context)):
            raise RuntimeError('Installed ROI launch enabled follower by default')
        print('PASS: installed launch binds template list to selected follower target',
              flush=True)
        print('PASS: installed ROI launch defaults to observation only', flush=True)

    def spin_until(executor, predicate, publish, *, timeout=8.0):
        deadline = time.monotonic() + timeout
        next_publish = 0.0
        while time.monotonic() < deadline:
            if publish is not None and time.monotonic() >= next_publish:
                publish()
                next_publish = time.monotonic() + 0.12
            executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        raise RuntimeError('Timed out waiting for installed ROS graph observation')

    def image_message(pixels, stamp):
        message = bridge.cv2_to_imgmsg(pixels, encoding='bgr8')
        message.header.stamp = stamp
        message.header.frame_id = 'camera_optical_frame'
        return message

    def multi_template():
        rclpy.init(domain_id=options.domain)
        detector = probe = executor = None
        try:
            detector = TemplateDetector(parameter_overrides=[
                Parameter('template_ids', value='10,20')])
            probe = rclpy.create_node('multi_template_probe')
            executor = SingleThreadedExecutor()
            executor.add_node(detector)
            executor.add_node(probe)
            images = probe.create_publisher(Image, '/camera/image_raw',
                                            qos_profile_sensor_data)
            outputs = []
            probe.create_subscription(Detection2DArray, '/perception/templates',
                                      outputs.append, qos_profile_sensor_data)
            template_dir = Path(get_package_share_directory('abot_perception')) / 'templates'
            composite = np.full((480, 640, 3), 150, dtype=np.uint8)
            for template_id, x in (('10', 20), ('20', 370)):
                template = cv2.imread(str(template_dir / f'{template_id}.png'))
                if template is None:
                    raise RuntimeError(f'Installed template {template_id} is missing')
                height, width = template.shape[:2]
                composite[100:100 + height, x:x + width] = template
            blank = np.full_like(composite, 150)
            spin_until(executor, lambda: images.get_subscription_count() == 1,
                       None)

            def publish(pixels):
                images.publish(image_message(pixels, probe.get_clock().now().to_msg()))

            spin_until(executor, lambda: any(
                [d.results[0].hypothesis.class_id for d in msg.detections]
                == ['template:10', 'template:20'] for msg in outputs),
                lambda: publish(composite))
            positive_stamp = outputs[-1].header.stamp
            print('PASS: installed multi-template node recognized 10 and 20 in one frame',
                  flush=True)
            spin_until(executor, lambda: any(
                not msg.detections and msg.header.stamp != positive_stamp
                for msg in outputs), lambda: publish(blank))
            print('PASS: blank frame cleared both template results', flush=True)
        finally:
            if executor is not None:
                executor.shutdown()
            for node in (probe, detector):
                if node is not None:
                    node.destroy_node()
            rclpy.try_shutdown()

    def roi_observations(enable_follower):
        ros_args = (['--ros-args', '-p', 'target_class:=roi',
                     '-p', 'detections_topic:=/roi_lk_tracker/roi']
                    if enable_follower else None)
        rclpy.init(args=ros_args, domain_id=options.domain +
                   (2 if enable_follower else 1))
        tracker = follower = probe = executor = None
        try:
            tracker = RoiLKTracker()
            if enable_follower:
                follower = VisionFollower()
            probe = rclpy.create_node('roi_lk_probe')
            executor = SingleThreadedExecutor()
            executor.add_node(tracker)
            if follower is not None:
                executor.add_node(follower)
            executor.add_node(probe)
            images = probe.create_publisher(Image, '/camera/image_raw',
                                            qos_profile_sensor_data)
            info_pub = probe.create_publisher(CameraInfo, '/camera/camera_info',
                                              qos_profile_sensor_data)
            seeds = probe.create_publisher(Detection2DArray,
                                           '/roi_lk_tracker/roi_seed', 1)
            outputs = []
            commands = []
            probe.create_subscription(Detection2DArray, '/roi_lk_tracker/roi',
                                      outputs.append, qos_profile_sensor_data)
            probe.create_subscription(Twist, '/cmd_vel/tracking', commands.append, 10)
            spin_until(executor, lambda: images.get_subscription_count() == 1 and
                       seeds.get_subscription_count() == 1 and
                       (not enable_follower or info_pub.get_subscription_count() == 1),
                       None)
            velocity_publishers = probe.get_publishers_info_by_topic('/cmd_vel/tracking')
            if len(velocity_publishers) != int(enable_follower):
                raise RuntimeError('ROI launch publisher isolation failed')

            gray = np.zeros((160, 220), dtype=np.uint8)
            rng = np.random.default_rng(7)
            gray[35:115, 90:170] = rng.integers(0, 256, (80, 80), dtype=np.uint8)
            first = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            shifted = cv2.warpAffine(first, np.float32([[1, 0, 5], [0, 1, 3]]),
                                     (220, 160))

            def publish_frame(pixels, stamp):
                if enable_follower:
                    info = CameraInfo()
                    info.header.stamp = stamp
                    info.header.frame_id = 'camera_optical_frame'
                    info.width, info.height = 220, 160
                    info_pub.publish(info)
                images.publish(image_message(pixels, stamp))

            stamp = probe.get_clock().now().to_msg()
            seed = Detection2DArray()
            seed.header.stamp = stamp
            seed.header.frame_id = 'camera_optical_frame'
            detection = Detection2D()
            detection.header = seed.header
            detection.bbox.center.position.x = 130.0
            detection.bbox.center.position.y = 75.0
            detection.bbox.size_x = 80.0
            detection.bbox.size_y = 80.0
            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = 'roi_seed'
            result.hypothesis.score = 1.0
            detection.results.append(result)
            seed.detections.append(detection)
            seeds.publish(seed)
            publish_frame(first, stamp)
            spin_until(executor, lambda: tracker.policy.active is not None, None,
                       timeout=2.0)

            def publish_shifted():
                publish_frame(shifted, probe.get_clock().now().to_msg())

            spin_until(executor, lambda: any(
                msg.detections and
                abs(msg.detections[0].bbox.center.position.x - 135) < 1.0 and
                abs(msg.detections[0].bbox.center.position.y - 78) < 1.0
                for msg in outputs), publish_shifted, timeout=2.0)
            if enable_follower:
                spin_until(executor, lambda: any(cmd.angular.z < -0.05 and
                                                 cmd.linear.x == 0.0
                                                 for cmd in commands),
                           publish_shifted, timeout=2.0)
                print('PASS: optional ROI follower turned through /cmd_vel/tracking',
                      flush=True)
            else:
                print('PASS: installed ROI node moved the seeded box with LK flow',
                      flush=True)
            before_loss = len(outputs)
            publish_frame(np.zeros_like(first), probe.get_clock().now().to_msg())
            spin_until(executor, lambda: any(not msg.detections
                                             for msg in outputs[before_loss:]),
                       None, timeout=2.0)
            if enable_follower:
                before_zero = len(commands)
                spin_until(executor, lambda: any(cmd.angular.z == 0.0 and
                                                 cmd.linear.x == 0.0
                                                 for cmd in commands[before_zero:]),
                           None, timeout=2.0)
            if probe.get_publishers_info_by_topic('/cmd_vel'):
                raise RuntimeError('ROI node published final velocity')
            print('PASS: feature loss emitted empty ROI; no final velocity publisher',
                  flush=True)
        finally:
            if executor is not None:
                executor.shutdown()
            for node in (probe, follower, tracker):
                if node is not None:
                    node.destroy_node()
            rclpy.try_shutdown()

    check_installed_launch()
    multi_template()
    roi_observations(False)
    roi_observations(True)


if __name__ == '__main__':
    main()
