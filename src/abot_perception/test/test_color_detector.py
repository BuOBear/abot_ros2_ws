import cv2
import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge

from abot_perception.color_detector import ColorDetector, detect_blobs, hsv_bounds


def ranges():
    return {
        'red': (hsv_bounds([0, 112, 41], [15, 255, 255]),
                hsv_bounds([170, 112, 41], [179, 255, 255])),
        'yellow': (hsv_bounds([20, 100, 100], [34, 255, 255]),),
        'green': (hsv_bounds([35, 78, 71], [85, 255, 255]),),
    }


def test_color_classes_and_minimum_area():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    cv2.rectangle(image, (15, 20), (75, 100), (0, 0, 255), -1)
    cv2.rectangle(image, (110, 20), (170, 100), (0, 255, 255), -1)
    cv2.rectangle(image, (210, 20), (280, 100), (0, 255, 0), -1)
    found = {blob.class_id: blob for blob in detect_blobs(image, ranges(), 500)}
    assert set(found) == {'red', 'yellow', 'green'}
    assert 40 < found['red'].center_x < 50
    assert 130 < found['yellow'].center_x < 150
    assert 240 < found['green'].center_x < 260
    assert detect_blobs(np.zeros_like(image), ranges(), 500) == []
    assert detect_blobs(image, ranges(), 10000) == []


@pytest.mark.parametrize('lower,upper', [
    ([0, 0], [10, 255, 255]),
    ([180, 0, 0], [180, 255, 255]),
    ([10, 0, 0], [5, 255, 255]),
])
def test_invalid_hsv_configuration_rejected(lower, upper):
    with pytest.raises(ValueError):
        hsv_bounds(lower, upper)


def test_ros_detection_keeps_camera_stamp_and_clears_on_empty_image():
    rclpy.init()
    node = None
    try:
        node = ColorDetector()
        published = []
        node.detections = type('Recorder', (), {'publish': lambda _, msg: published.append(msg)})()
        image = np.zeros((100, 120, 3), dtype=np.uint8)
        cv2.rectangle(image, (50, 20), (90, 80), (0, 0, 255), -1)
        bridge = CvBridge()
        message = bridge.cv2_to_imgmsg(image, encoding='bgr8')
        message.header.frame_id = 'camera_optical_frame'
        message.header.stamp = node.get_clock().now().to_msg()
        node.on_image(message)
        assert len(published[-1].detections) == 1
        assert published[-1].header == message.header
        assert published[-1].detections[0].results[0].hypothesis.class_id == 'red'
        assert published[-1].detections[0].bbox.center.position.x > 60
        node.last_processed = None
        blank = bridge.cv2_to_imgmsg(np.zeros_like(image), encoding='bgr8')
        blank.header = message.header
        node.on_image(blank)
        assert published[-1].header == message.header
        assert published[-1].detections == []

        node.last_processed = None
        node.on_image(message)
        malformed = bridge.cv2_to_imgmsg(image, encoding='bgr8')
        malformed.header.frame_id = 'camera_optical_frame'
        malformed.header.stamp = node.get_clock().now().to_msg()
        malformed.encoding = 'not_a_ros_encoding'
        node.on_image(malformed)
        assert published[-1].header == malformed.header
        assert published[-1].detections == []
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
