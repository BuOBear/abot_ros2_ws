"""Observation-only, stamped ROI Lucas-Kanade tracker.

An exact image/seed acquisition-stamp match starts a track.  The box keeps its
seeded size and moves by the median displacement of forward/backward-consistent
features; a feature hull is never substituted for the ROI.
"""

from collections import OrderedDict
from dataclasses import dataclass
import math
import threading
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return None
    value = stamp.sec * 1_000_000_000 + stamp.nanosec
    return value if value > 0 else None


@dataclass(frozen=True)
class Observation:
    stamp: object
    frame: str
    box: tuple | None


class RoiLKPolicy:
    """State machine for one explicitly seeded ROI, with bounded image history."""

    def __init__(self, *, seed_timeout=0.3, image_timeout=0.3,
                 max_frame_gap=0.25, max_cached_images=4,
                 max_image_pixels=2_073_600, min_features=8,
                 fb_error_px=1.0, consensus_error_px=3.0):
        self.seed_timeout = float(seed_timeout)
        self.image_timeout = float(image_timeout)
        self.max_frame_gap = float(max_frame_gap)
        self.max_cached_images = int(max_cached_images)
        self.max_image_pixels = int(max_image_pixels)
        self.min_features = int(min_features)
        self.fb_error_px = float(fb_error_px)
        self.consensus_error_px = float(consensus_error_px)
        if (not all(math.isfinite(v) and v > 0 for v in
                    (self.seed_timeout, self.image_timeout, self.max_frame_gap,
                     self.fb_error_px, self.consensus_error_px)) or
                not 1 <= self.max_cached_images <= 16 or
                not 1 <= self.max_image_pixels <= 33_554_432 or
                not 4 <= self.min_features <= 200):
            raise ValueError('Invalid ROI LK parameters')
        self.images = OrderedDict()
        self.pending_seed = None
        self.active = None
        self.last_image_key = None
        self.last_seed_key = None
        self.last_ros_now = None
        self.last_steady_now = None
        self.geometry = None

    def clear(self, *, clear_cache=True):
        self.active = None
        self.pending_seed = None
        if clear_cache:
            self.images.clear()

    def _clock_ok(self, now, ros_now):
        if not math.isfinite(now) or not math.isfinite(ros_now):
            self.clear()
            return False
        if self.last_ros_now is not None:
            ros_step = ros_now - self.last_ros_now
            steady_step = now - self.last_steady_now
            tolerance = max(0.5, 2 * self.image_timeout)
            if (ros_step < -1e-6 or steady_step < 0 or
                    abs(ros_step - steady_step) > tolerance):
                self.clear()
                self.last_image_key = None
                self.last_seed_key = None
                self.geometry = None
        self.last_ros_now = ros_now
        self.last_steady_now = now
        return True

    @staticmethod
    def _fresh(key, received_at, timeout, now, ros_now):
        age = ros_now - key * 1e-9
        return (-1e-6 <= age <= timeout and received_at is not None and
                0 <= now - received_at <= timeout)

    def _expire(self, now, ros_now):
        for key, (_, _, received_at) in list(self.images.items()):
            if not self._fresh(key, received_at, self.seed_timeout, now, ros_now):
                del self.images[key]
        if (self.pending_seed is not None and
                not self._fresh(self.pending_seed[0], self.pending_seed[3],
                                self.seed_timeout, now, ros_now)):
            self.pending_seed = None
        if (self.active is not None and
                not self._fresh(self.active['stamp_key'], self.active['received_at'],
                                self.image_timeout, now, ros_now)):
            self.clear()

    def tick(self, now, ros_now):
        if self._clock_ok(now, ros_now):
            self._expire(now, ros_now)

    @staticmethod
    def _box_ok(box, width, height):
        x, y, w, h = box
        return (all(math.isfinite(v) for v in box) and w >= 2 and h >= 2 and
                x - w / 2 >= 0 and y - h / 2 >= 0 and
                x + w / 2 <= width and y + h / 2 <= height)

    def receive_seed(self, stamp_key, frame, box, now, ros_now):
        if not self._clock_ok(now, ros_now):
            return False
        self._expire(now, ros_now)
        if (stamp_key is None or not frame or
                self.last_seed_key is not None and stamp_key <= self.last_seed_key or
                self.active is not None and stamp_key <= self.active['stamp_key'] or
                not -1e-6 <= ros_now - stamp_key * 1e-9 <= self.seed_timeout):
            self.clear()
            return False
        self.last_seed_key = stamp_key
        if (len(box) != 4 or not all(math.isfinite(v) for v in box) or
                box[2] < 2 or box[3] < 2 or
                box[0] - box[2] / 2 < 0 or box[1] - box[3] / 2 < 0):
            self.clear()
            return False
        cached = self.images.get(stamp_key)
        if cached is not None:
            gray, image_frame, received_at = cached
            if (image_frame != frame or
                    not self._fresh(stamp_key, received_at, self.seed_timeout,
                                    now, ros_now) or
                    not self._box_ok(box, gray.shape[1], gray.shape[0])):
                self.clear()
                return False
            self.clear(clear_cache=False)
            return self._start(gray, stamp_key, frame, box, received_at)
        if self.last_image_key is not None and stamp_key <= self.last_image_key:
            self.clear()
            return False
        # A seed may arrive before its image through independent ROS topics.
        # Only one pending seed is kept; a newer seed replaces the older one.
        self.clear(clear_cache=False)
        self.pending_seed = (stamp_key, frame, tuple(box), now)
        return True

    def _features(self, gray, box):
        x, y, w, h = box
        mask = np.zeros_like(gray)
        x0, y0 = int(math.ceil(x - w / 2)), int(math.ceil(y - h / 2))
        x1, y1 = int(math.floor(x + w / 2)), int(math.floor(y + h / 2))
        mask[y0:y1, x0:x1] = 255
        try:
            points = cv2.goodFeaturesToTrack(
                gray, maxCorners=200, qualityLevel=0.02, minDistance=7,
                mask=mask, blockSize=7, useHarrisDetector=True, k=0.04)
        except cv2.error:
            return None
        return None if points is None else np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)

    def _start(self, gray, stamp_key, frame, box, received_at):
        points = self._features(gray, box)
        if points is None or len(points) < self.min_features:
            self.clear()
            return False
        self.active = {'gray': gray, 'points': points, 'box': tuple(box),
                       'stamp_key': stamp_key, 'received_at': received_at,
                       'frame': frame}
        return True

    def _advance(self, gray, stamp_key, frame, received_at):
        old = self.active
        if (old['frame'] != frame or gray.shape != old['gray'].shape or
                not 0 < (stamp_key - old['stamp_key']) * 1e-9 <= self.max_frame_gap or
                not self._fresh(old['stamp_key'], old['received_at'],
                                self.image_timeout, received_at, stamp_key * 1e-9)):
            self.clear()
            return None
        params = dict(winSize=(15, 15), maxLevel=2,
                      criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.01))
        p0 = old['points']
        try:
            p1, forward, _ = cv2.calcOpticalFlowPyrLK(old['gray'], gray, p0, None, **params)
            if p1 is None or forward is None:
                self.clear()
                return None
            p0r, reverse, _ = cv2.calcOpticalFlowPyrLK(gray, old['gray'], p1, None, **params)
            if p0r is None or reverse is None:
                self.clear()
                return None
        except cv2.error:
            self.clear()
            return None
        before, after, backward = p0.reshape(-1, 2), p1.reshape(-1, 2), p0r.reshape(-1, 2)
        valid = (forward.reshape(-1) == 1) & (reverse.reshape(-1) == 1)
        valid &= np.isfinite(after).all(axis=1) & np.isfinite(backward).all(axis=1)
        valid &= np.max(np.abs(before - backward), axis=1) <= self.fb_error_px
        valid &= (after[:, 0] >= 0) & (after[:, 0] < gray.shape[1])
        valid &= (after[:, 1] >= 0) & (after[:, 1] < gray.shape[0])
        if np.count_nonzero(valid) < self.min_features:
            self.clear()
            return None
        flow = after[valid] - before[valid]
        median = np.median(flow, axis=0)
        coherent = np.max(np.abs(flow - median), axis=1) <= self.consensus_error_px
        if np.count_nonzero(coherent) < self.min_features:
            self.clear()
            return None
        displacement = np.median(flow[coherent], axis=0)
        x, y, w, h = old['box']
        box = (float(x + displacement[0]), float(y + displacement[1]), w, h)
        if not self._box_ok(box, gray.shape[1], gray.shape[0]):
            self.clear()
            return None
        self.active = {'gray': gray, 'points': after[valid][coherent].reshape(-1, 1, 2),
                       'box': box, 'stamp_key': stamp_key,
                       'received_at': received_at, 'frame': frame}
        return box

    def receive_image(self, gray, stamp_key, frame, stamp, now, ros_now):
        if not self._clock_ok(now, ros_now):
            return None
        self._expire(now, ros_now)
        if (stamp_key is None or not frame or not isinstance(gray, np.ndarray) or
                gray.ndim != 2 or gray.dtype != np.uint8 or
                gray.size == 0 or gray.size > self.max_image_pixels or
                self.last_image_key is not None and stamp_key <= self.last_image_key or
                not -1e-6 <= ros_now - stamp_key * 1e-9 <= self.image_timeout):
            self.clear()
            return None
        geometry = (frame, gray.shape)
        lost_on_geometry = False
        if self.geometry is not None and geometry != self.geometry:
            lost_on_geometry = self.active is not None
            # A new-mode seed can arrive before its first image.  Preserve
            # only that exact pending seed across the geometry boundary.
            pending = (self.pending_seed if self.pending_seed is not None and
                       self.pending_seed[0] == stamp_key else None)
            self.clear()
            self.pending_seed = pending
        self.geometry = geometry
        self.last_image_key = stamp_key
        self.images[stamp_key] = (gray, frame, now)
        while len(self.images) > self.max_cached_images:
            self.images.popitem(last=False)
        self._expire(now, ros_now)
        if self.active is not None:
            box = self._advance(gray, stamp_key, frame, now)
            return Observation(stamp, frame, box)
        if self.pending_seed is not None and self.pending_seed[0] == stamp_key:
            _, seed_frame, box, _ = self.pending_seed
            self.pending_seed = None
            if seed_frame == frame and self._box_ok(box, gray.shape[1], gray.shape[0]):
                if self._start(gray, stamp_key, frame, box, now):
                    return Observation(stamp, frame, box)
            self.clear()
            return Observation(stamp, frame, None)
        if lost_on_geometry:
            return Observation(stamp, frame, None)
        return None

    def reject_image(self, stamp_key, frame, stamp, now, ros_now):
        """Invalidate on decode failure and publish loss only for a fresh image header."""
        clock_ok = self._clock_ok(now, ros_now)
        emit = (clock_ok and stamp_key is not None and frame and
                (self.last_image_key is None or stamp_key > self.last_image_key) and
                -1e-6 <= ros_now - stamp_key * 1e-9 <= self.image_timeout)
        self.clear()
        if emit:
            self.last_image_key = stamp_key
            return Observation(stamp, frame, None)
        return None


def make_detection(observation):
    array = Detection2DArray()
    array.header.stamp = observation.stamp
    array.header.frame_id = observation.frame
    if observation.box is not None:
        detection = Detection2D()
        detection.header = array.header
        detection.id = 'roi'
        x, y, w, h = observation.box
        detection.bbox.center.position.x = x
        detection.bbox.center.position.y = y
        detection.bbox.size_x = w
        detection.bbox.size_y = h
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = 'roi'
        result.hypothesis.score = 1.0
        detection.results.append(result)
        array.detections.append(detection)
    return array


class RoiLKTracker(Node):
    def __init__(self):
        super().__init__('roi_lk_tracker')
        defaults = {
            'seed_timeout': 0.3, 'image_timeout': 0.3,
            'max_frame_gap': 0.25, 'max_cached_images': 4,
            'max_image_pixels': 2_073_600, 'min_features': 8,
            'fb_error_px': 1.0, 'consensus_error_px': 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('seed_topic', '~/roi_seed')
        self.policy = RoiLKPolicy(**{name: self.get_parameter(name).value for name in defaults})
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.loss_pending = False
        self.create_subscription(Image, str(self.get_parameter('image_topic').value),
                                 self.on_image, qos_profile_sensor_data)
        seed_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Detection2DArray,
                                 str(self.get_parameter('seed_topic').value),
                                 self.on_seed, seed_qos)
        self.publisher = self.create_publisher(Detection2DArray, '~/roi', seed_qos)
        self.create_timer(0.05, self.on_timer,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _times(self):
        return time.monotonic(), self.get_clock().now().nanoseconds * 1e-9

    def on_timer(self):
        now, ros_now = self._times()
        with self.lock:
            self.policy.tick(now, ros_now)

    def on_seed(self, array):
        now, ros_now = self._times()
        with self.lock:
            key = stamp_ns(array.header.stamp)
            was_active = self.policy.active is not None
            if len(array.detections) != 1:
                self.policy.clear()
                self.loss_pending |= was_active
                return
            detection = array.detections[0]
            if (detection.header.stamp != array.header.stamp or
                    detection.header.frame_id != array.header.frame_id or
                    len(detection.results) != 1 or
                    detection.results[0].hypothesis.class_id != 'roi_seed' or
                    not math.isfinite(detection.results[0].hypothesis.score) or
                    detection.results[0].hypothesis.score <= 0 or
                    not math.isfinite(detection.bbox.center.theta) or
                    abs(detection.bbox.center.theta) > 1e-6):
                self.policy.clear()
                self.loss_pending |= was_active
                return
            box = detection.bbox
            accepted = self.policy.receive_seed(
                key, array.header.frame_id,
                (box.center.position.x, box.center.position.y, box.size_x, box.size_y),
                now, ros_now)
            if accepted:
                self.loss_pending = False
            else:
                self.loss_pending |= was_active

    def on_image(self, image):
        now, ros_now = self._times()
        try:
            if (image.width <= 0 or image.height <= 0 or
                    image.width * image.height > self.policy.max_image_pixels):
                raise ValueError('Invalid image dimensions')
            gray = self.bridge.imgmsg_to_cv2(image, desired_encoding='mono8')
            if gray.shape != (image.height, image.width):
                raise ValueError('Invalid image geometry')
            gray = np.ascontiguousarray(gray)
        except (CvBridgeError, ValueError, cv2.error):
            with self.lock:
                observation = self.policy.reject_image(
                    stamp_ns(image.header.stamp), image.header.frame_id,
                    image.header.stamp, now, ros_now)
                self.loss_pending = observation is None and self.loss_pending
                if observation is not None:
                    self.publisher.publish(make_detection(observation))
            return
        with self.lock:
            prior_key = self.policy.last_image_key
            image_key = stamp_ns(image.header.stamp)
            observation = self.policy.receive_image(
                gray, image_key, image.header.frame_id,
                image.header.stamp, now, ros_now)
            if observation is None and self.loss_pending and (
                    image_key is not None and
                    (prior_key is None or image_key > prior_key) and
                    self.policy.last_image_key == image_key):
                observation = Observation(image.header.stamp, image.header.frame_id, None)
            if observation is not None:
                self.loss_pending = False
                self.publisher.publish(make_detection(observation))


def main():
    rclpy.init()
    node = RoiLKTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
