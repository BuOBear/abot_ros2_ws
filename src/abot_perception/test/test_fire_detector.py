import cv2
import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge

from abot_perception.color_detector import hsv_bounds
from abot_perception.fire_detector import FireDetector, detect_fire_candidate


def fire_ranges():
    return {
        'red': (hsv_bounds([0, 128, 46], [5, 255, 255]),
                hsv_bounds([156, 128, 46], [179, 255, 255])),
        'yellow': (hsv_bounds([15, 128, 46], [50, 255, 255]),),
    }


def many_warm_blobs():
    image = np.zeros((260, 360, 3), dtype=np.uint8)
    wrapped_red = cv2.cvtColor(
        np.full((18, 18, 3), (179, 255, 255), dtype=np.uint8),
        cv2.COLOR_HSV2BGR)[0, 0].tolist()
    colors = [(0, 0, 255), (0, 255, 255), tuple(wrapped_red)]
    for row in range(3):
        for column in range(4):
            x = 10 + column * 80
            y = 10 + row * 70
            color = colors[(row * 4 + column) % len(colors)]
            cv2.rectangle(image, (x, y), (x + 17, y + 17), color, -1)
    return image


def test_many_red_yellow_and_hue_wrap_contours_pass_as_one_union_candidate():
    boxes = detect_fire_candidate(many_warm_blobs(), fire_ranges(), 10, 40)

    assert len(boxes) == 1
    assert boxes[0].class_id == 'fire_candidate'
    assert boxes[0].size_x > 250
    assert boxes[0].size_y > 150
    assert 0.0 < boxes[0].score <= 1.0


def test_one_large_warm_contour_fails_legacy_count_gate():
    image = np.zeros((260, 360, 3), dtype=np.uint8)
    cv2.rectangle(image, (20, 20), (320, 220), (0, 0, 255), -1)
    assert detect_fire_candidate(image, fire_ranges(), 10, 40) == []
    assert detect_fire_candidate(np.zeros_like(image), fire_ranges(), 10, 40) == []


@pytest.mark.parametrize('min_contours', [-1, 1.5, True])
def test_invalid_contour_count_is_rejected(min_contours):
    with pytest.raises(ValueError):
        detect_fire_candidate(np.zeros((10, 10, 3), dtype=np.uint8),
                              fire_ranges(), min_contours, 40)


@pytest.mark.parametrize('min_area', [0, -1, float('nan'), float('inf'), True])
def test_invalid_aggregate_area_is_rejected(min_area):
    with pytest.raises(ValueError):
        detect_fire_candidate(np.zeros((10, 10, 3), dtype=np.uint8),
                              fire_ranges(), 10, min_area)


def test_ros_source_preserves_image_header_and_publishes_empty_on_loss():
    rclpy.init()
    node = None
    try:
        node = FireDetector()
        published = []
        node.detections = type('Recorder', (), {
            'publish': lambda _, msg: published.append(msg)})()
        bridge = CvBridge()
        image = many_warm_blobs()
        message = bridge.cv2_to_imgmsg(image, encoding='bgr8')
        message.header.frame_id = 'camera_optical_frame'
        message.header.stamp = node.get_clock().now().to_msg()

        node.on_image(message)

        assert len(published[-1].detections) == 1
        assert published[-1].header == message.header
        detection = published[-1].detections[0]
        assert detection.header == message.header
        assert detection.results[0].hypothesis.class_id == 'fire_candidate'

        node.last_processed = None
        blank = bridge.cv2_to_imgmsg(np.zeros_like(image), encoding='bgr8')
        blank.header = message.header
        node.on_image(blank)
        assert published[-1].header == blank.header
        assert published[-1].detections == []

        node.last_processed = None
        malformed = bridge.cv2_to_imgmsg(image, encoding='bgr8')
        malformed.header = message.header
        malformed.encoding = 'not_a_ros_encoding'
        node.on_image(malformed)
        assert published[-1].header == malformed.header
        assert published[-1].detections == []
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
