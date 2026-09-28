"""Synthetic stamped ROI motion, loss, and replay checks."""

import cv2
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
import numpy as np
import rclpy
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from abot_tracking.roi_lk_tracker import RoiLKPolicy, RoiLKTracker, make_detection


def key(seconds):
    return round(seconds * 1_000_000_000)


def stamp(seconds):
    value = key(seconds)
    return Time(sec=value // 1_000_000_000, nanosec=value % 1_000_000_000)


def textured_image():
    rng = np.random.default_rng(7)
    image = np.zeros((160, 220), dtype=np.uint8)
    image[35:115, 70:150] = rng.integers(0, 256, (80, 80), dtype=np.uint8)
    return image


def shifted(image, dx, dy):
    matrix = np.float32([[1, 0, dx], [0, 1, dy]])
    return cv2.warpAffine(image, matrix, (image.shape[1], image.shape[0]))


def image(policy, pixels, seconds, *, frame='camera_optical_frame', now=None):
    if now is None:
        now = seconds
    return policy.receive_image(pixels, key(seconds), frame, stamp(seconds), now, seconds)


def test_seed_before_image_tracks_translation_without_shrinking_box():
    policy = RoiLKPolicy()
    first = textured_image()
    box = (110.0, 75.0, 80.0, 80.0)
    assert policy.receive_seed(key(1.0), 'camera_optical_frame', box, 1.0, 1.0)
    seeded = image(policy, first, 1.0)
    assert seeded.box == box
    moved = image(policy, shifted(first, 5, 3), 1.05)
    assert moved.box is not None
    assert abs(moved.box[0] - 115) < 1.0
    assert abs(moved.box[1] - 78) < 1.0
    assert moved.box[2:] == box[2:]
    array = make_detection(moved)
    assert array.header.stamp == stamp(1.05)
    assert array.header.frame_id == 'camera_optical_frame'
    assert len(array.detections) == 1
    assert array.detections[0].results[0].hypothesis.class_id == 'roi'


def test_image_before_seed_requires_exact_cached_stamp_and_bounds_cache():
    policy = RoiLKPolicy(max_cached_images=2)
    first = textured_image()
    assert image(policy, first, 1.0) is None
    assert image(policy, first, 1.05) is None
    assert len(policy.images) == 2
    assert policy.receive_seed(key(1.0), 'camera_optical_frame',
                               (110.0, 75.0, 80.0, 80.0), 1.06, 1.06)
    moved = image(policy, shifted(first, 5, 0), 1.10)
    assert moved.box is not None
    assert abs(moved.box[0] - 115) < 1.0
    assert len(policy.images) <= 2


def test_loss_replay_geometry_and_clock_rewind_clear_tracking():
    first = textured_image()
    box = (110.0, 75.0, 80.0, 80.0)
    policy = RoiLKPolicy()
    assert policy.receive_seed(key(1.0), 'camera_optical_frame', box, 1.0, 1.0)
    assert image(policy, first, 1.0).box == box
    assert image(policy, np.zeros_like(first), 1.05).box is None
    assert policy.active is None
    assert image(policy, first, 1.05) is None  # replay
    assert not policy.receive_seed(key(1.0), 'camera_optical_frame', box, 1.06, 1.06)
    assert image(policy, first, 1.10) is None

    assert policy.receive_seed(key(1.15), 'camera_optical_frame', box, 1.15, 1.15)
    assert image(policy, first, 1.15).box == box
    resized = np.zeros((170, 220), np.uint8)
    resized[:160, :] = first
    assert image(policy, resized, 1.20).box is None
    assert policy.active is None
    assert policy.receive_seed(key(1.20), 'camera_optical_frame',
                               (110.0, 75.0, 80.0, 80.0), 1.21, 1.21)
    assert policy.active is not None

    assert policy.receive_seed(key(1.25), 'camera_optical_frame', box, 1.25, 1.25)
    assert image(policy, first, 1.25).box == box
    policy.tick(1.30, 0.5)  # /clock rewind
    assert policy.active is None
    assert policy.last_image_key is None
    assert policy.last_seed_key is None


def test_stale_unmatched_and_unsupported_seed_cannot_start():
    policy = RoiLKPolicy()
    first = textured_image()
    box = (110.0, 75.0, 80.0, 80.0)
    assert not policy.receive_seed(key(1.0), 'camera_optical_frame', box, 1.5, 1.5)
    assert image(policy, first, 1.6) is None
    assert not policy.receive_seed(key(1.55), 'camera_optical_frame', box, 1.61, 1.61)
    assert not policy.receive_seed(key(1.65), 'camera_optical_frame',
                                   (110.0, 75.0, 300.0, 80.0), 1.65, 1.65)
    assert image(policy, first, 1.65) is None
    assert policy.active is None
    assert policy.receive_seed(key(1.7), 'camera_optical_frame', box, 1.7, 1.7)
    assert image(policy, np.zeros_like(first), 1.7).box is None
    assert policy.active is None


def test_timer_expiry_and_forward_clock_jump_drop_state_and_cache():
    first = textured_image()
    box = (110.0, 75.0, 80.0, 80.0)
    policy = RoiLKPolicy()
    assert policy.receive_seed(key(1.0), 'camera_optical_frame', box, 1.0, 1.0)
    assert image(policy, first, 1.0).box == box
    policy.tick(1.31, 1.31)
    assert policy.active is None
    assert len(policy.images) == 0
    assert image(policy, first, 1.35) is None
    assert policy.receive_seed(key(1.40), 'camera_optical_frame', box, 1.40, 1.40)
    assert image(policy, first, 1.40).box == box
    policy.tick(1.45, 3.0)
    assert policy.active is None
    assert policy.last_image_key is None
    assert policy.last_seed_key is None


def test_invalid_image_and_seed_publish_empty_on_fresh_acquisition():
    rclpy.init()
    node = RoiLKTracker()
    published = []
    node.publisher = type('Collector', (), {'publish': lambda _, msg: published.append(msg)})()
    current = [1.0]
    node._times = lambda: (current[0], current[0])
    bridge = CvBridge()
    first = textured_image()

    def frame(seconds, pixels=first):
        message = bridge.cv2_to_imgmsg(pixels, encoding='mono8')
        message.header.stamp = stamp(seconds)
        message.header.frame_id = 'camera_optical_frame'
        return message

    def seed(seconds, class_id='roi_seed'):
        array = Detection2DArray()
        array.header.stamp = stamp(seconds)
        array.header.frame_id = 'camera_optical_frame'
        detection = Detection2D()
        detection.header = array.header
        detection.bbox.center.position.x = 110.0
        detection.bbox.center.position.y = 75.0
        detection.bbox.size_x = 80.0
        detection.bbox.size_y = 80.0
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = class_id
        result.hypothesis.score = 1.0
        detection.results.append(result)
        array.detections.append(detection)
        return array

    try:
        node.on_seed(seed(1.0))
        node.on_image(frame(1.0))
        assert len(published[-1].detections) == 1
        current[0] = 1.05
        bad = frame(1.05)
        bad.encoding = 'not_a_real_encoding'
        node.on_image(bad)
        assert published[-1].header.stamp == stamp(1.05)
        assert published[-1].detections == []
        assert node.policy.active is None

        current[0] = 1.10
        node.on_seed(seed(1.10))
        node.on_image(frame(1.10))
        assert len(published[-1].detections) == 1
        current[0] = 1.15
        node.on_seed(seed(1.15, class_id='wrong'))
        assert node.policy.active is None
        node.on_image(frame(1.15))
        assert published[-1].header.stamp == stamp(1.15)
        assert published[-1].detections == []
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
