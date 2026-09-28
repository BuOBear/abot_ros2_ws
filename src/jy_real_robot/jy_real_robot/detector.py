"""YOLO11 camera subscriber. No downloads, invented classes, or motion output."""
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from .core import LatestFrame, detections_from_result, fresh, local_weights


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def to_message(header, rows):
    output = Detection2DArray()
    output.header = header
    for label, score, x1, y1, x2, y2 in rows:
        item = Detection2D()
        item.header = header
        item.bbox.center.position.x = (x1 + x2) / 2.0
        item.bbox.center.position.y = (y1 + y2) / 2.0
        item.bbox.size_x, item.bbox.size_y = x2 - x1, y2 - y1
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = label
        hypothesis.hypothesis.score = score
        item.results = [hypothesis]
        output.detections.append(item)
    return output


class Detector(Node):
    def __init__(self):
        super().__init__('yolo11_detector')
        defaults = {'weights': '', 'device': 'cpu', 'confidence': 0.5, 'imgsz': 640,
                    'maximum_age_sec': 0.5, 'image_topic': '/camera/image_raw',
                    'output_topic': '/perception/detections'}
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.confidence = self.get_parameter('confidence').value
        self.maximum_age = self.get_parameter('maximum_age_sec').value
        self.imgsz = self.get_parameter('imgsz').value
        if not 0 < self.confidence <= 1 or not 0 < self.maximum_age <= 5 or self.imgsz < 32:
            raise ValueError('invalid detector configuration')
        filename = local_weights(self.get_parameter('weights').value)
        # Import only when explicitly launching inference. Pure geometry/tests need no torch.
        from ultralytics import YOLO
        self.model = YOLO(filename, task='detect')
        if self.model.task != 'detect':
            raise ValueError('this node requires a detection model')
        self.device = self.get_parameter('device').value
        self.bridge = CvBridge()
        self.queue = LatestFrame()
        self.results = LatestFrame()
        self.stop = threading.Event()
        self.wake = threading.Event()
        self.publisher = self.create_publisher(
            Detection2DArray, self.get_parameter('output_topic').value, 2)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.subscription = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.receive, qos_profile_sensor_data)
        self.timer = self.create_timer(0.02, self.flush)
        self.worker = threading.Thread(target=self.infer, daemon=True)
        self.worker.start()

    def receive(self, message):
        if not message.header.frame_id or not fresh(
                stamp_ns(message.header.stamp), self.get_clock().now().nanoseconds, self.maximum_age):
            return
        self.queue.put(message)
        self.wake.set()

    def infer(self):
        while not self.stop.is_set():
            self.wake.wait(0.1)
            self.wake.clear()
            message = self.queue.take()
            if message is None:
                continue
            try:
                if not fresh(stamp_ns(message.header.stamp), self.get_clock().now().nanoseconds,
                             self.maximum_age):
                    continue
                pixels = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
                result = self.model.predict(pixels, conf=self.confidence, imgsz=self.imgsz,
                                            device=self.device, verbose=False)[0]
                rows = detections_from_result(result, message.width, message.height, self.confidence)
                self.results.put((message.header, rows, ''))
            except Exception as exc:
                self.results.put((message.header, [], f'inference failed: {exc}'))

    def flush(self):
        value = self.results.take()
        if value is None:
            return
        header, rows, error = value
        if not fresh(stamp_ns(header.stamp), self.get_clock().now().nanoseconds, self.maximum_age):
            error = 'inference result expired; discarded'
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(name='jy/yolo11', hardware_id='camera')
        status.level = DiagnosticStatus.ERROR if error else DiagnosticStatus.OK
        status.message = error or 'local detection model active'
        diagnostic.status = [status]
        self.diagnostics.publish(diagnostic)
        if not error:
            self.publisher.publish(to_message(header, rows))

    def destroy_node(self):
        self.stop.set()
        self.wake.set()
        self.worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Detector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
