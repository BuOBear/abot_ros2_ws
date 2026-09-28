"""Conservative two-scan lidar follower for the tracking velocity input.

The ROS1 tracker selected the closest return corroborated in a nearby beam of
the previous scan. It did not identify a person or maintain a target identity.
This node preserves that limited evidence and keeps velocity authority external.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class LidarFollowPolicy:
    """Validate scans, find a temporally corroborated return, and bound motion."""

    def __init__(self, *, expected_frame='laser_link', scan_timeout=0.4,
                 max_scan_gap=0.3, forward_half_angle=0.7853981633974483,
                 window_beams=2, distance_tolerance=0.15,
                 min_target_distance=0.35, max_target_distance=2.5,
                 target_distance=1.0, distance_deadband=0.1,
                 angle_deadband=0.05, linear_alignment_angle=0.15,
                 kp_linear=0.6, kp_yaw=0.7,
                 max_linear_speed=0.10, max_yaw_rate=0.25,
                 enable_linear_motion=False):
        self.expected_frame = str(expected_frame)
        self.scan_timeout = float(scan_timeout)
        self.max_scan_gap = float(max_scan_gap)
        self.forward_half_angle = float(forward_half_angle)
        self.window_beams = window_beams
        self.distance_tolerance = float(distance_tolerance)
        self.min_target_distance = float(min_target_distance)
        self.max_target_distance = float(max_target_distance)
        self.target_distance = float(target_distance)
        self.distance_deadband = float(distance_deadband)
        self.angle_deadband = float(angle_deadband)
        self.linear_alignment_angle = float(linear_alignment_angle)
        self.kp_linear = float(kp_linear)
        self.kp_yaw = float(kp_yaw)
        self.max_linear_speed = float(max_linear_speed)
        self.max_yaw_rate = float(max_yaw_rate)
        if (not self.expected_frame or type(self.window_beams) is not int or
                self.window_beams < 0 or type(enable_linear_motion) is not bool or
                not all(math.isfinite(value) for value in (
                    self.scan_timeout, self.max_scan_gap, self.forward_half_angle,
                    self.distance_tolerance, self.min_target_distance,
                    self.max_target_distance, self.target_distance,
                    self.distance_deadband, self.angle_deadband,
                    self.linear_alignment_angle,
                    self.kp_linear, self.kp_yaw, self.max_linear_speed,
                    self.max_yaw_rate)) or
                self.scan_timeout <= 0 or self.max_scan_gap <= 0 or
                self.max_scan_gap > self.scan_timeout or
                not 0 < self.forward_half_angle <= math.pi / 2 or
                self.distance_tolerance <= 0 or self.min_target_distance <= 0 or
                self.max_target_distance <= self.min_target_distance or
                not self.min_target_distance < self.target_distance < self.max_target_distance or
                self.distance_deadband < 0 or self.angle_deadband < 0 or
                self.angle_deadband >= self.forward_half_angle or
                not self.angle_deadband <= self.linear_alignment_angle < self.forward_half_angle or
                self.kp_linear <= 0 or self.kp_yaw <= 0 or
                self.max_linear_speed <= 0 or self.max_yaw_rate <= 0):
            raise ValueError('Invalid lidar follower parameters')
        self.enable_linear_motion = enable_linear_motion
        self.previous = None
        self.previous_stamp = None
        self.previous_at = None
        self.geometry = None
        self.target = None
        self.target_stamp = None
        self.target_at = None
        self.last_ros_now = None

    def _clear(self, clear_previous=True):
        self.target = None
        self.target_stamp = None
        self.target_at = None
        if clear_previous:
            self.previous = None
            self.previous_stamp = None
            self.previous_at = None
            self.geometry = None

    def _valid_geometry(self, scan):
        count = len(scan.ranges)
        if (scan.header.frame_id != self.expected_frame or count < 180 or
                not all(math.isfinite(v) for v in (
                    scan.angle_min, scan.angle_max, scan.angle_increment,
                    scan.range_min, scan.range_max)) or
                scan.angle_increment <= 0 or scan.range_min < 0 or
                scan.range_max <= scan.range_min or
                scan.range_max < self.max_target_distance or
                scan.range_min > self.min_target_distance or
                abs(scan.angle_max - (scan.angle_min + (count - 1) * scan.angle_increment))
                > scan.angle_increment * 0.5 or
                count * scan.angle_increment < math.radians(350) or
                count * scan.angle_increment > 2 * math.pi + 1.5 * scan.angle_increment):
            return False
        if not (scan.angle_min <= -self.forward_half_angle and
                scan.angle_max >= self.forward_half_angle):
            return False
        # Preserve the full-circle scan quality demanded by the downstream
        # velocity gate. +Inf is a valid clear ray; NaN and out-of-range rays
        # provide no coverage evidence. The seam joins end blind runs.
        valid = [value == math.inf or
                 (math.isfinite(value) and scan.range_min <= value <= scan.range_max)
                 for value in scan.ranges]
        if sum(valid) / count < 0.8:
            return False
        run = longest = 0
        for usable in valid:
            run = 0 if usable else run + 1
            longest = max(longest, run)
        leading = next((index for index, usable in enumerate(valid) if usable), count)
        trailing = next((index for index, usable in enumerate(reversed(valid)) if usable), count)
        unobserved = max(0.0, 2 * math.pi - count * scan.angle_increment)
        blind = max(longest * scan.angle_increment,
                    (leading + trailing) * scan.angle_increment + unobserved)
        return blind <= math.radians(20)

    def receive_scan(self, scan, now, ros_now):
        if self.last_ros_now is not None and ros_now < self.last_ros_now - 1e-6:
            self._clear()
        self.last_ros_now = ros_now
        stamp = stamp_seconds(scan.header.stamp)
        if (not math.isfinite(now) or not math.isfinite(ros_now) or
                not 0 <= ros_now - stamp <= self.scan_timeout or
                not self._valid_geometry(scan)):
            self._clear()
            return
        # Delayed, duplicate and reordered samples cannot renew a target.
        if self.previous_stamp is not None and stamp <= self.previous_stamp:
            self._clear()
            return
        geometry = (scan.header.frame_id, len(scan.ranges), scan.angle_min,
                    scan.angle_increment, scan.range_min, scan.range_max)
        same_geometry = geometry == self.geometry
        previous_fresh = (self.previous is not None and same_geometry and
                          stamp - self.previous_stamp <= self.max_scan_gap and
                          now - self.previous_at <= self.max_scan_gap)
        self.target = None
        self.target_stamp = None
        self.target_at = None
        if previous_fresh:
            candidates = []
            for index, distance in enumerate(scan.ranges):
                angle = scan.angle_min + index * scan.angle_increment
                if (not -self.forward_half_angle <= angle <= self.forward_half_angle or
                        not math.isfinite(distance) or
                        not max(scan.range_min, self.min_target_distance) <= distance <=
                        min(scan.range_max, self.max_target_distance)):
                    continue
                low = max(0, index - self.window_beams)
                high = min(len(self.previous), index + self.window_beams + 1)
                if any(math.isfinite(old) and
                       max(scan.range_min, self.min_target_distance) <= old <=
                       min(scan.range_max, self.max_target_distance) and
                       abs(old - distance) <= self.distance_tolerance
                       for old in self.previous[low:high]):
                    candidates.append((distance, abs(angle), angle))
            if candidates:
                distance, _, angle = min(candidates)
                self.target = (angle, distance)
                self.target_stamp = stamp
                self.target_at = now
        self.previous = tuple(scan.ranges)
        self.previous_stamp = stamp
        self.previous_at = now
        self.geometry = geometry

    def command(self, now, ros_now):
        command = Twist()
        if (self.target is None or self.target_at is None or
                not 0 <= now - self.target_at <= self.scan_timeout or
                not 0 <= ros_now - self.target_stamp <= self.scan_timeout):
            return command
        angle, distance = self.target
        if abs(angle) > self.angle_deadband:
            command.angular.z = max(-self.max_yaw_rate,
                                    min(self.max_yaw_rate, self.kp_yaw * angle))
        if (self.enable_linear_motion and abs(angle) <= self.linear_alignment_angle and
                abs(distance - self.target_distance) > self.distance_deadband):
            command.linear.x = max(-self.max_linear_speed,
                                   min(self.max_linear_speed,
                                       self.kp_linear * (distance - self.target_distance)))
        return command


class LidarFollower(Node):
    def __init__(self):
        super().__init__('lidar_follower')
        defaults = {
            'expected_frame': 'laser_link', 'scan_timeout': 0.4,
            'max_scan_gap': 0.3, 'forward_half_angle': math.pi / 4,
            'window_beams': 2, 'distance_tolerance': 0.15,
            'min_target_distance': 0.35, 'max_target_distance': 2.5,
            'target_distance': 1.0, 'distance_deadband': 0.1,
            'angle_deadband': 0.05, 'linear_alignment_angle': 0.15,
            'kp_linear': 0.6, 'kp_yaw': 0.7,
            'max_linear_speed': 0.10, 'max_yaw_rate': 0.25,
            'enable_linear_motion': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter('scan_topic', '/scan_filtered')
        self.policy = LidarFollowPolicy(**{
            name: self.get_parameter(name).value for name in defaults})
        self.create_subscription(LaserScan, str(self.get_parameter('scan_topic').value),
                                 self.on_scan, qos_profile_sensor_data)
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        self.command_pub = self.create_publisher(Twist, '/cmd_vel/tracking', command_qos)
        self.create_timer(0.05, self.publish_current,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_scan(self, scan):
        self.policy.receive_scan(scan, time.monotonic(),
                                 self.get_clock().now().nanoseconds * 1e-9)

    def publish_current(self):
        self.command_pub.publish(self.policy.command(
            time.monotonic(), self.get_clock().now().nanoseconds * 1e-9))


def main():
    rclpy.init()
    node = LidarFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.command_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
