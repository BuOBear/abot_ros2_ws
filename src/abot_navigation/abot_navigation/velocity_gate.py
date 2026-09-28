"""Final command publisher with a steady-time fail-closed watchdog.

The collision monitor may stop publishing a steady zero command after its
stop_pub_timeout, or be unable to transform a scan when odometry TF disappears.
This node is the only publisher of /cmd_vel and keeps the hardware input at zero
unless the monitor, scan, and odometry TF are all current.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .velocity_authority import ScanFreshness, has_usable_scan_data, scan_quality_settings


class VelocityGate(Node):
    def __init__(self):
        super().__init__('velocity_gate')
        self.declare_parameter('input_timeout', 0.2)
        self.declare_parameter('scan_timeout', 0.5)
        self.input_timeout = float(self.get_parameter('input_timeout').value)
        if not math.isfinite(self.input_timeout) or self.input_timeout <= 0:
            raise ValueError('input_timeout must be finite and positive')
        self.scan_freshness = ScanFreshness(float(self.get_parameter('scan_timeout').value))
        self.scan_quality = scan_quality_settings(self)
        self.tf_freshness = ScanFreshness(self.scan_freshness.timeout)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_frame = None
        self.scan_time = None
        self.command = None
        self.command_at = None
        self.create_subscription(LaserScan, '/scan_filtered', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel/collision_checked', self.on_command, 1)
        self.output = self.create_publisher(Twist, '/cmd_vel', 1)
        # A ROS-time timer would freeze along with /clock and the hardware input.
        self.create_timer(0.05, self.publish_current,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_scan(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        was_fresh = self.scan_freshness.fresh(now, ros_now)
        if not has_usable_scan_data(msg, **self.scan_quality):
            self.scan_freshness.invalidate(stamp)
            self.scan_frame = None
            self.scan_time = None
            self.command = None
            self.command_at = None
            return
        if not was_fresh:
            # Recovery needs a new collision-monitor command, not one cached
            # before the observation stream failed.
            self.command = None
            self.command_at = None
        self.scan_frame = msg.header.frame_id
        self.scan_time = rclpy.time.Time.from_msg(msg.header.stamp)
        self.scan_freshness.receive(stamp, ros_now, now)

    def on_command(self, msg):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        if not self.scan_freshness.fresh(now, ros_now) or not self.transforms_ready(now, ros_now):
            self.command = None
            self.command_at = None
            return
        components = (msg.linear.x, msg.linear.y, msg.linear.z,
                      msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(value) for value in components):
            self.command = None
            self.command_at = None
            return
        self.command = msg
        self.command_at = now

    def transforms_ready(self, now, ros_now):
        if not self.scan_frame or self.scan_time is None:
            return False
        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            self.tf_freshness.receive(stamp, ros_now, now)
            return self.tf_freshness.fresh(now, ros_now) and self.tf_buffer.can_transform(
                'odom', self.scan_frame, self.scan_time)
        except TransformException:
            return False

    def publish_current(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        ready = (self.command_at is not None and now - self.command_at <= self.input_timeout
                 and self.scan_freshness.fresh(now, ros_now)
                 and self.transforms_ready(now, ros_now))
        if not ready:
            self.command = None
            self.command_at = None
        self.output.publish(self.command if ready else Twist())


def main():
    rclpy.init()
    node = VelocityGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
