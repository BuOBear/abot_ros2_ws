"""Turn toward a fresh visual target through the existing tracking input.

The old tracker_pkg follower wrote /cmd_vel directly and assumed a joystick
button array was present. This node never grants itself control: the existing
velocity authority must explicitly receive /control/mode = tracking.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class TrackingPolicy:
    """Pure freshness and image-center control policy for one selected class."""

    def __init__(self, target_class, kp_yaw, max_yaw_rate, center_deadband,
                 min_bbox_area_px, camera_info_timeout, detection_timeout):
        self.target_class = str(target_class)
        self.kp_yaw = float(kp_yaw)
        self.max_yaw_rate = float(max_yaw_rate)
        self.center_deadband = float(center_deadband)
        self.min_bbox_area_px = float(min_bbox_area_px)
        self.camera_info_timeout = float(camera_info_timeout)
        self.detection_timeout = float(detection_timeout)
        values = (self.kp_yaw, self.max_yaw_rate, self.center_deadband,
                  self.min_bbox_area_px, self.camera_info_timeout, self.detection_timeout)
        if (not self.target_class or not all(math.isfinite(value) for value in values) or
                self.kp_yaw <= 0 or self.max_yaw_rate <= 0 or
                not 0 <= self.center_deadband < 1 or self.min_bbox_area_px <= 0 or
                self.camera_info_timeout <= 0 or self.detection_timeout <= 0):
            raise ValueError('Invalid visual tracking parameters')
        self.camera_width = None
        self.camera_height = None
        self.camera_frame = None
        self.camera_stamp = None
        self.camera_stamp_key = None
        self.camera_at = None
        self.camera_history = {}
        self.target_camera_stamp = None
        self.target_camera_at = None
        self.detection_stamp = None
        self.detection_stamp_key = None
        self.detection_at = None
        self.target_error = None
        self.last_ros_now = None

    def _sync_clock(self, ros_now):
        # A /clock rewind starts a new acquisition epoch. Old stamp watermarks
        # must not prevent the follower from recovering after it has stopped.
        if self.last_ros_now is not None and ros_now < self.last_ros_now - 1e-6:
            self.camera_width = None
            self.camera_height = None
            self.camera_frame = None
            self.camera_stamp = None
            self.camera_stamp_key = None
            self.camera_at = None
            self.camera_history.clear()
            self.target_camera_stamp = None
            self.target_camera_at = None
            self.detection_stamp = None
            self.detection_stamp_key = None
            self.detection_at = None
            self.target_error = None
        self.last_ros_now = ros_now

    @staticmethod
    def _fresh(stamp, received_at, timeout, now, ros_now):
        return (stamp is not None and received_at is not None and
                0 <= now - received_at <= timeout and
                0 <= ros_now - stamp <= timeout)

    def receive_camera_info(self, info, now, ros_now):
        self._sync_clock(ros_now)
        stamp = stamp_seconds(info.header.stamp)
        key = (info.header.stamp.sec, info.header.stamp.nanosec)
        if self.camera_stamp_key is not None and key <= self.camera_stamp_key:
            return
        if (not info.header.frame_id or info.width <= 0 or info.height <= 0 or
                not 0 <= ros_now - stamp <= self.camera_info_timeout):
            self.camera_at = None
            self.target_error = None
            self.camera_history.clear()
            return
        # A mode or optical-frame change invalidates the old pixel error.
        if (info.width != self.camera_width or info.height != self.camera_height or
                info.header.frame_id != self.camera_frame):
            self.target_error = None
            self.detection_at = None
            self.camera_history.clear()
        self.camera_width = info.width
        self.camera_height = info.height
        self.camera_frame = info.header.frame_id
        self.camera_stamp = stamp
        self.camera_stamp_key = key
        self.camera_at = now
        self.camera_history[key] = (stamp, now)
        # A 120 fps camera can produce 60 CameraInfo messages within the
        # default 0.5 s validity window. Keep the whole fresh window while
        # bounding memory if an unusually high rate is configured.
        while self.camera_history:
            oldest_stamp, oldest_at = next(iter(self.camera_history.values()))
            if (len(self.camera_history) <= 256 and
                    now - oldest_at <= self.camera_info_timeout and
                    ros_now - oldest_stamp <= self.camera_info_timeout):
                break
            self.camera_history.pop(next(iter(self.camera_history)))

    def receive_detections(self, array, now, ros_now):
        self._sync_clock(ros_now)
        stamp = stamp_seconds(array.header.stamp)
        key = (array.header.stamp.sec, array.header.stamp.nanosec)
        if self.detection_stamp_key is not None and key <= self.detection_stamp_key:
            return
        self.target_error = None
        self.detection_at = None
        matched_info = self.camera_history.get(key)
        if (matched_info is None or array.header.frame_id != self.camera_frame or
                not 0 <= ros_now - stamp <= self.detection_timeout or
                not self._fresh(matched_info[0], matched_info[1],
                                self.camera_info_timeout, now, ros_now)):
            return
        self.detection_stamp = stamp
        self.detection_stamp_key = key
        candidates = []
        for detection in array.detections:
            if (detection.header.stamp != array.header.stamp or
                    detection.header.frame_id != array.header.frame_id):
                continue
            if not any(result.hypothesis.class_id == self.target_class and
                       math.isfinite(result.hypothesis.score) and
                       result.hypothesis.score >= 0
                       for result in detection.results):
                continue
            box = detection.bbox
            x, y, width, height = (box.center.position.x, box.center.position.y,
                                   box.size_x, box.size_y)
            if (not all(math.isfinite(v) for v in (x, y, width, height)) or
                    width <= 0 or height <= 0 or
                    x - width / 2 < 0 or x + width / 2 > self.camera_width or
                    y - height / 2 < 0 or y + height / 2 > self.camera_height or
                    width * height < self.min_bbox_area_px):
                continue
            candidates.append((width * height, x))
        if not candidates:
            return
        _, center_x = max(candidates)
        self.target_error = (center_x - self.camera_width / 2.0) / (self.camera_width / 2.0)
        self.detection_at = now
        self.target_camera_stamp, self.target_camera_at = matched_info

    def yaw_rate(self, now, ros_now):
        self._sync_clock(ros_now)
        if (not self._fresh(self.target_camera_stamp, self.target_camera_at,
                            self.camera_info_timeout, now, ros_now) or
                not self._fresh(self.detection_stamp, self.detection_at,
                                self.detection_timeout, now, ros_now) or
                self.target_error is None or abs(self.target_error) <= self.center_deadband):
            return 0.0
        # Image x grows right; ROS positive yaw turns left.
        command = -self.kp_yaw * self.target_error
        return max(-self.max_yaw_rate, min(self.max_yaw_rate, command))


class VisionFollower(Node):
    def __init__(self):
        super().__init__('vision_follower')
        defaults = {
            'target_class': 'red', 'kp_yaw': 0.5, 'max_yaw_rate': 0.25,
            'center_deadband': 0.05, 'min_bbox_area_px': 500.0,
            'camera_info_timeout': 0.5, 'detection_timeout': 0.3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter('detections_topic', '/perception/colors')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.policy = TrackingPolicy(**{
            name: self.get_parameter(name).value for name in defaults})
        self.create_subscription(CameraInfo,
                                 str(self.get_parameter('camera_info_topic').value),
                                 self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray,
                                 str(self.get_parameter('detections_topic').value),
                                 self.on_detections, qos_profile_sensor_data)
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        self.command_pub = self.create_publisher(Twist, '/cmd_vel/tracking', command_qos)
        self.create_timer(0.05, self.publish_current,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_camera_info(self, info):
        self.policy.receive_camera_info(info, time.monotonic(),
                                        self.get_clock().now().nanoseconds * 1e-9)

    def on_detections(self, array):
        self.policy.receive_detections(array, time.monotonic(),
                                       self.get_clock().now().nanoseconds * 1e-9)

    def publish_current(self):
        command = Twist()
        command.angular.z = self.policy.yaw_rate(
            time.monotonic(), self.get_clock().now().nanoseconds * 1e-9)
        self.command_pub.publish(command)


def main():
    rclpy.init()
    node = VisionFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
