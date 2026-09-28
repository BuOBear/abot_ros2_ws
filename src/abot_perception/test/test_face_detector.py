from pathlib import Path

import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge

from abot_perception.face_detector import FaceDetector, detect_faces


class FakeCascade:
    def __init__(self, rectangles):
        self.rectangles = rectangles
        self.calls = []

    def detectMultiScale(self, image, **kwargs):
        self.calls.append((image, kwargs))
        return self.rectangles


def test_face_boxes_clip_to_image_and_report_candidate_score():
    image = np.zeros((60, 80, 3), dtype=np.uint8)
    cascade = FakeCascade([(-4, 10, 20, 30), (40, 20, 60, 20),
                           (10, 10, 0, 5), (0, 0, np.nan, 10)])

    boxes = detect_faces(image, cascade)

    assert len(cascade.calls) == 1
    gray, options = cascade.calls[0]
    assert gray.shape == (60, 80)
    assert gray.dtype == np.uint8
    assert options == {'scaleFactor': 1.1, 'minNeighbors': 5, 'minSize': (30, 30)}
    assert [(box.class_id, box.center_x, box.center_y,
             box.size_x, box.size_y, box.score) for box in boxes] == [
                 ('face', 8.0, 25.0, 16.0, 30.0, 1.0),
                 ('face', 60.0, 30.0, 40.0, 20.0, 1.0)]


@pytest.mark.parametrize('image,kwargs', [
    (np.zeros((0, 10, 3), dtype=np.uint8), {}),
    (np.zeros((10, 10), dtype=np.uint8), {}),
    (np.zeros((10, 10, 3), dtype=np.float32), {}),
    (np.zeros((10, 10, 3), dtype=np.uint8), {'scale_factor': 1.0}),
    (np.zeros((10, 10, 3), dtype=np.uint8), {'min_neighbors': 1.5}),
    (np.zeros((10, 10, 3), dtype=np.uint8), {'min_size_px': 0}),
])
def test_invalid_face_input_or_threshold_rejected(image, kwargs):
    with pytest.raises(ValueError):
        detect_faces(image, FakeCascade([]), **kwargs)


def test_ros_face_source_preserves_camera_header_and_publishes_empty(
        monkeypatch):
    package_dir = Path(__file__).parents[1]
    monkeypatch.setattr('abot_perception.face_detector.get_package_share_directory',
                        lambda _: str(package_dir))
    video_capture_calls = []

    def forbidden_video_capture(*args, **kwargs):
        video_capture_calls.append((args, kwargs))
        raise AssertionError('FaceDetector must consume Image messages only')

    monkeypatch.setattr('abot_perception.face_detector.cv2.VideoCapture',
                        forbidden_video_capture)
    rclpy.init()
    node = None
    try:
        node = FaceDetector()
        node.cascade = FakeCascade([(20, 15, 40, 50)])
        published = []
        node.detections = type(
            'Recorder', (), {'publish': lambda _, msg: published.append(msg)})()

        image = np.zeros((100, 120, 3), dtype=np.uint8)
        message = CvBridge().cv2_to_imgmsg(image, encoding='bgr8')
        message.header.frame_id = 'camera_optical_frame'
        message.header.stamp = node.get_clock().now().to_msg()
        node.on_image(message)
        assert published[-1].header == message.header
        assert published[-1].detections[0].header == message.header
        assert published[-1].detections[0].results[0].hypothesis.class_id == 'face'
        assert published[-1].detections[0].results[0].hypothesis.score == 1.0

        node.cascade = FakeCascade([])
        node.last_processed = None
        blank = CvBridge().cv2_to_imgmsg(image, encoding='bgr8')
        blank.header = message.header
        node.on_image(blank)
        assert published[-1].header == blank.header
        assert published[-1].detections == []

        node.cascade = FakeCascade([(20, 15, 40, 50)])
        node.last_processed = None
        node.on_image(message)
        malformed = CvBridge().cv2_to_imgmsg(image, encoding='bgr8')
        malformed.header.frame_id = 'camera_optical_frame'
        malformed.header.stamp = node.get_clock().now().to_msg()
        malformed.step = 0
        node.on_image(malformed)
        assert published[-1].header == malformed.header
        assert published[-1].detections == []
        assert video_capture_calls == []
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
