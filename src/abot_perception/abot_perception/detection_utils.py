"""Shared stamped image-to-Detection2DArray conversion for visual sources."""

from dataclasses import dataclass
import math
import time

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


@dataclass(frozen=True)
class ImageBox:
    class_id: str
    center_x: float
    center_y: float
    size_x: float
    size_y: float
    score: float


def to_detection_array(header, boxes):
    array = Detection2DArray()
    array.header = header
    for box in boxes:
        detection = Detection2D()
        detection.header = header
        detection.bbox.center.position.x = box.center_x
        detection.bbox.center.position.y = box.center_y
        detection.bbox.size_x = box.size_x
        detection.bbox.size_y = box.size_y
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = box.class_id
        result.hypothesis.score = box.score
        detection.results.append(result)
        array.detections.append(detection)
    return array


class ImageDetector(Node):
    """One camera input and one private detection topic per recognition source."""

    def __init__(self, name, default_output_topic, default_hz, parameter_overrides=None):
        super().__init__(name, parameter_overrides=parameter_overrides)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', default_output_topic)
        self.declare_parameter('max_processing_hz', default_hz)
        image_topic = str(self.get_parameter('image_topic').value).strip()
        output_topic = str(self.get_parameter('output_topic').value).strip()
        if not image_topic or not output_topic:
            raise ValueError('image_topic and output_topic must not be empty')
        try:
            hz = float(self.get_parameter('max_processing_hz').value)
        except (TypeError, ValueError) as exc:
            raise ValueError('max_processing_hz must be finite and positive') from exc
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError('max_processing_hz must be finite and positive')
        self.period = 1.0 / hz
        self.last_processed = None
        self.has_detections = False
        self.bridge = CvBridge()
        self.detections = self.create_publisher(
            Detection2DArray, output_topic,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, image_topic,
            self.on_image, qos_profile_sensor_data)

    def find_boxes(self, image_bgr):
        raise NotImplementedError

    def _valid_image_layout(self, msg):
        """Reject truncated/empty ROS Image messages before rate limiting."""
        try:
            height = int(msg.height)
            width = int(msg.width)
            step = int(msg.step)
            cv_type = self.bridge.encoding_to_cvtype2(str(msg.encoding))
            depth = cv_type & 7
            channels = ((cv_type >> 3) & 63) + 1
            bytes_per_channel = (1, 1, 2, 2, 4, 4, 8, 2)[depth]
            return (height > 0 and width > 0 and step >= width * channels * bytes_per_channel
                    and len(msg.data) >= height * step)
        except (AttributeError, TypeError, ValueError, OverflowError, CvBridgeError):
            return False

    def _publish_empty(self, header):
        self.has_detections = False
        self.detections.publish(to_detection_array(header, []))

    def on_image(self, msg):
        now = time.monotonic()
        if not self._valid_image_layout(msg):
            # Clear immediately if a target was active. When already clear,
            # rate-limit repeated malformed frames to protect the DDS topic.
            if (self.last_processed is None or now - self.last_processed >= self.period
                    or self.has_detections):
                self.last_processed = now
                self.get_logger().warn('Cannot process camera image: empty or truncated Image data')
                self._publish_empty(msg.header)
            return
        if self.last_processed is not None and now - self.last_processed < self.period:
            return
        self.last_processed = now
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            boxes = self.find_boxes(image)
        except (CvBridgeError, cv2.error, ValueError) as exc:
            self.get_logger().warn(f'Cannot process camera image: {exc}')
            self._publish_empty(msg.header)
            return
        # Empty arrays clear the previous target on this source. The camera
        # acquisition stamp/frame are never replaced with detector wall time.
        self.has_detections = bool(boxes)
        self.detections.publish(to_detection_array(msg.header, boxes))


def spin_detector(detector_type):
    rclpy.init()
    node = detector_type()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
