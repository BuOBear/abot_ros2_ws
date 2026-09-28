#!/usr/bin/env python3
"""Exercise installed color detection and visual following without a camera or base."""

import argparse
import os
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=118)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'

    import cv2
    import numpy as np
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from vision_msgs.msg import Detection2DArray
    from abot_perception.color_detector import ColorDetector
    from abot_tracking.vision_follower import VisionFollower

    rclpy.init()
    detector = ColorDetector()
    follower = VisionFollower()
    probe = rclpy.create_node('vision_smoke_probe')
    executor = SingleThreadedExecutor()
    for node in (detector, follower, probe):
        executor.add_node(node)
    image_pub = probe.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
    info_pub = probe.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
    last = {'detections': None, 'detections_at': None,
            'command': None, 'command_at': None}
    probe.create_subscription(Detection2DArray, '/perception/colors',
                              lambda msg: last.update(detections=msg,
                                                      detections_at=time.monotonic()),
                              qos_profile_sensor_data)
    probe.create_subscription(Twist, '/cmd_vel/tracking',
                              lambda msg: last.update(command=msg, command_at=time.monotonic()), 10)
    bridge = CvBridge()
    red = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(red, (450, 190), (530, 290), (0, 0, 255), -1)
    blank = np.zeros_like(red)

    def send_frame(frame):
        stamp = probe.get_clock().now().to_msg()
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = 'camera_optical_frame'
        info.width, info.height = 640, 480
        image = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image.header = info.header
        info_pub.publish(info)
        image_pub.publish(image)

    def until(predicate, frame, label, timeout=6.0):
        deadline = time.monotonic() + timeout
        next_frame = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() >= next_frame:
                send_frame(frame)
                next_frame = time.monotonic() + 0.12
            executor.spin_once(timeout_sec=0.03)
            if predicate():
                print('PASS:', label, flush=True)
                return
        raise RuntimeError('Timed out: ' + label)

    try:
        until(lambda: image_pub.get_subscription_count() > 0 and
              info_pub.get_subscription_count() > 0,
              blank, 'installed nodes discover camera topics')
        until(lambda: last['detections'] is not None and
              any(item.results[0].hypothesis.class_id == 'red'
                  for item in last['detections'].detections) and
              last['command'] is not None and last['command_at'] > last['detections_at'] and
              last['command'].angular.z < -0.05,
              red, 'red target right turns toward image center')
        until(lambda: last['detections'] is not None and
              not last['detections'].detections and last['command'] is not None and
              last['command_at'] > last['detections_at'] and last['command'].angular.z == 0.0,
              blank, 'empty detection stops follower')
        stop_at = time.monotonic()
        while time.monotonic() - stop_at < 0.8:
            executor.spin_once(timeout_sec=0.03)
        assert last['command_at'] is not None and time.monotonic() - last['command_at'] < 0.15
        assert last['command'].angular.z == 0.0
        assert not probe.get_publishers_info_by_topic('/cmd_vel')
        print('PASS: stale image stream keeps fresh zero and no final velocity publisher',
              flush=True)
    finally:
        executor.shutdown()
        for node in (probe, follower, detector):
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
