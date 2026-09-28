"""Optional known-plane estimate. Never publishes a grasp pose or arm command."""
from collections import deque
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from .core import fresh, calibrated_ray, rotation_from_quaternion, intersect_plane
from .detector import stamp_ns


def matching_camera_info(header, infos):
    """Use calibration from the detection's source image, not the newest frame."""
    for info in reversed(infos):
        if (info.header.frame_id == header.frame_id
                and stamp_ns(info.header.stamp) == stamp_ns(header.stamp)):
            return info
    return None


def matching_image_size(header, images):
    """Recover source-image dimensions omitted by Detection2DArray."""
    for stamp, frame, width, height in reversed(images):
        if frame == header.frame_id and stamp == stamp_ns(header.stamp):
            return width, height
    return None


class TableTarget(Node):
    def __init__(self):
        super().__init__('table_target')
        defaults = {'calibration_verified': False, 'target_frame': 'base_link',
                    'target_class': '', 'plane_normal': [0.0, 0.0, 1.0],
                    'plane_offset': 0.0, 'maximum_age_sec': 0.3, 'minimum_confidence': 0.6}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        if not self.settings['calibration_verified'] or not self.settings['target_class']:
            raise ValueError('measured plane, camera calibration, wrist TF and target_class required')
        normal = np.asarray(self.settings['plane_normal'])
        if (normal.shape != (3,) or not np.all(np.isfinite(normal)) or np.linalg.norm(normal) < 1e-8
                or not math.isfinite(self.settings['plane_offset'])
                or not 0 < self.settings['maximum_age_sec'] <= 1
                or not 0 < self.settings['minimum_confidence'] <= 1):
            raise ValueError('invalid plane or detection limits')
        self.infos = deque(maxlen=128)
        self.images = deque(maxlen=128)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PointStamped, '/perception/plane_point', 2)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.image, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/perception/detections', self.detect, 2)

    def camera(self, info):
        self.infos.append(info)

    def image(self, message):
        self.images.append((stamp_ns(message.header.stamp), message.header.frame_id,
                            message.width, message.height))

    def detect(self, message):
        info = matching_camera_info(message.header, self.infos)
        if info is None:
            return
        if matching_image_size(message.header, self.images) != (info.width, info.height):
            return
        if not fresh(stamp_ns(message.header.stamp), self.get_clock().now().nanoseconds,
                     self.settings['maximum_age_sec']):
            return
        # Cropped/binned CameraInfo needs adjusted intrinsics; fail closed for now.
        if info.binning_x > 1 or info.binning_y > 1 or info.roi.width or info.roi.height:
            return
        candidates = []
        for item in message.detections:
            for result in item.results:
                h = result.hypothesis
                if h.class_id == self.settings['target_class'] and self.settings['minimum_confidence'] <= h.score <= 1:
                    candidates.append((h.score, item))
        if not candidates:
            return
        item = max(candidates, key=lambda pair: pair[0])[1]
        u, v = item.bbox.center.position.x, item.bbox.center.position.y
        if not 0 <= u < info.width or not 0 <= v < info.height:
            return
        try:
            ray = calibrated_ray(u, v, info.k, info.d, info.distortion_model)
            transform = self.buffer.lookup_transform(self.settings['target_frame'], message.header.frame_id,
                                                     Time.from_msg(message.header.stamp)).transform
            t, q = transform.translation, transform.rotation
            point = intersect_plane(ray, [t.x, t.y, t.z], rotation_from_quaternion([q.x, q.y, q.z, q.w]),
                                    self.settings['plane_normal'], self.settings['plane_offset'])
            output = PointStamped()
            output.header.stamp = message.header.stamp
            output.header.frame_id = self.settings['target_frame']
            output.point.x, output.point.y, output.point.z = map(float, point)
            self.publisher.publish(output)
        except Exception as exc:
            self.get_logger().warning(f'plane estimate rejected: {exc}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TableTarget()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
