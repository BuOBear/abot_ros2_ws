"""Safety properties of the only final /cmd_vel publisher."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from abot_navigation.velocity_gate import VelocityGate


def test_gate_requires_current_monitor_input_scan_and_tf():
    rclpy.init()
    gate = None
    try:
        gate = VelocityGate()
        published = []
        gate.output = type('Recorder', (), {'publish': lambda _, msg: published.append(msg)})()
        command = Twist()
        command.linear.y = 0.1
        ros_now = gate.get_clock().now().nanoseconds * 1e-9
        gate.scan_freshness.receive(ros_now, ros_now, time.monotonic())
        gate.transforms_ready = lambda *_: True
        gate.on_command(command)
        gate.publish_current()
        assert published[-1].linear.y == 0.1

        gate.transforms_ready = lambda *_: False
        gate.publish_current()
        assert published[-1].linear.y == 0.0

        gate.transforms_ready = lambda *_: True
        gate.scan_freshness.invalidate(ros_now + 0.01)
        gate.publish_current()
        assert published[-1].linear.y == 0.0

        ros_now = gate.get_clock().now().nanoseconds * 1e-9
        gate.scan_freshness.receive(ros_now, ros_now, time.monotonic())
        gate.publish_current()
        assert published[-1].linear.y == 0.0  # Old monitor command cannot return.
        gate.on_command(command)
        gate.command_at = time.monotonic() - gate.input_timeout - 0.01
        gate.publish_current()
        gate.publish_current()
        assert [msg.linear.y for msg in published[-2:]] == [0.0, 0.0]
    finally:
        if gate is not None:
            gate.destroy_node()
        rclpy.shutdown()


def test_gate_recovery_requires_new_monitor_callback_after_observed_fault():
    rclpy.init()
    gate = None
    try:
        gate = VelocityGate()
        published = []
        gate.output = type('Recorder', (), {'publish': lambda _, msg: published.append(msg)})()
        tf_ready = [True]
        gate.transforms_ready = lambda *_: tf_ready[0]

        def components(msg):
            return (msg.linear.x, msg.linear.y, msg.linear.z,
                    msg.angular.x, msg.angular.y, msg.angular.z)

        def send_scan(stamp_ns, ranges):
            scan = LaserScan()
            scan.header.frame_id = 'laser_link'
            scan.header.stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = 2 * math.pi / 359
            scan.range_min = 0.05
            scan.range_max = 10.0
            scan.ranges = ranges
            gate.on_scan(scan)

        command = Twist()
        command.linear.x, command.linear.y, command.linear.z = 0.1, -0.1, 0.01
        command.angular.x, command.angular.y, command.angular.z = 0.02, -0.02, 0.1
        base_ns = gate.get_clock().now().nanoseconds - 100_000_000
        clear_scan = [math.inf] * 360

        send_scan(base_ns, clear_scan)
        gate.on_command(command)
        gate.publish_current()
        assert components(published[-1]) == components(command)

        send_scan(base_ns + 10_000_000, [math.nan] * 360)
        gate.on_command(command)  # Receipt during the scan fault cannot arm motion.
        gate.publish_current()
        assert components(published[-1]) == (0.0,) * 6
        assert gate.command is None and gate.command_at is None
        send_scan(base_ns + 20_000_000, clear_scan)
        gate.publish_current()
        assert components(published[-1]) == (0.0,) * 6
        gate.on_command(command)
        gate.publish_current()
        assert components(published[-1]) == components(command)

        gate.scan_freshness.expires_at = time.monotonic() - 1.0
        gate.on_command(command)  # Stale scan is also observed at receipt.
        gate.publish_current()
        assert components(published[-1]) == (0.0,) * 6
        send_scan(base_ns + 30_000_000, clear_scan)
        gate.publish_current()
        assert components(published[-1]) == (0.0,) * 6
        gate.on_command(command)
        gate.publish_current()
        assert components(published[-1]) == components(command)

        tf_ready[0] = False
        gate.on_command(command)
        gate.publish_current()
        assert components(published[-1]) == (0.0,) * 6
        assert gate.command is None and gate.command_at is None
        tf_ready[0] = True
        gate.publish_current()
        assert components(published[-1]) == (0.0,) * 6
        gate.on_command(command)
        gate.publish_current()
        assert components(published[-1]) == components(command)

        gate.command_at = time.monotonic() - gate.input_timeout - 0.01
        gate.publish_current()
        assert components(published[-1]) == (0.0,) * 6
        assert gate.command is None and gate.command_at is None
    finally:
        if gate is not None:
            gate.destroy_node()
        rclpy.shutdown()
