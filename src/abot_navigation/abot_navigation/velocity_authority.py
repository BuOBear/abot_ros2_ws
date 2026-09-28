"""Explicit, sticky velocity ownership before twist_mux.

Only an explicit volatile /control/mode message grants a source control. The
current mode is echoed on transient-local /control/mode_state, including when
the same valid mode is requested again. A source timeout generates zero in the
same mux channel; it never changes mode. Wall monotonic time keeps the watchdog
effective if simulated time pauses.
"""

import math
import secrets
import time

import rclpy
from abot_control_interfaces.srv import AcquireNav, ReleaseNav
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String, UInt64
from tf2_ros import Buffer, TransformListener, TransformException


MODES = ('disabled', 'nav', 'tracking', 'teleop')


SCAN_QUALITY_DEFAULTS = {
    'min_beams': 180,
    'min_span_degrees': 350.0,
    'min_valid_fraction': 0.8,
    'max_blind_degrees': 20.0,
    'max_range_min': 0.2,
    'min_range_max': 1.0,
}


def scan_quality_settings(node):
    """Read conservative provisional coverage bounds shared by both guards."""
    for name, value in SCAN_QUALITY_DEFAULTS.items():
        node.declare_parameter('scan_' + name, value)
    settings = {name: node.get_parameter('scan_' + name).value
                for name in SCAN_QUALITY_DEFAULTS}
    if (not isinstance(settings['min_beams'], int) or settings['min_beams'] < 2 or
            not all(isinstance(value, (int, float)) and math.isfinite(value)
                    for name, value in settings.items() if name != 'min_beams') or
            not 0 < settings['min_span_degrees'] <= 360 or
            not 0 < settings['min_valid_fraction'] <= 1 or
            not 0 < settings['max_blind_degrees'] < 360 or
            not 0 <= settings['max_range_min'] < settings['min_range_max']):
        raise ValueError('invalid scan quality settings')
    return settings


def has_usable_scan_data(scan, *, min_beams=180, min_span_degrees=350.0,
                         min_valid_fraction=0.8, max_blind_degrees=20.0,
                         max_range_min=0.2, min_range_max=1.0):
    """Require a full, well formed scan with enough usable angular coverage."""
    count = len(scan.ranges)
    angles = (scan.angle_min, scan.angle_max, scan.angle_increment)
    if (count < min_beams or not all(math.isfinite(angle) for angle in angles) or
            scan.angle_increment <= 0 or scan.angle_max <= scan.angle_min):
        return False
    expected_max = scan.angle_min + (count - 1) * scan.angle_increment
    if (abs(scan.angle_max - expected_max) > 1.5 * scan.angle_increment or
            count * scan.angle_increment < math.radians(min_span_degrees) or
            count * scan.angle_increment > 2 * math.pi + 1.5 * scan.angle_increment):
        return False
    if (not math.isfinite(scan.range_min) or not math.isfinite(scan.range_max) or
            scan.range_min < 0 or scan.range_min > max_range_min or
            scan.range_max < min_range_max or scan.range_max <= scan.range_min):
        return False
    # +Inf is the usual no-return reading for a clear ray. NaN, -Inf and
    # finite readings outside the sensor's declared limits provide no evidence.
    valid = [value == math.inf or
             (math.isfinite(value) and scan.range_min <= value <= scan.range_max)
             for value in scan.ranges]
    if sum(valid) / count < min_valid_fraction:
        return False
    run = longest = 0
    for usable in valid:
        run = 0 if usable else run + 1
        longest = max(longest, run)
    leading = next((index for index, usable in enumerate(valid) if usable), count)
    trailing = next((index for index, usable in enumerate(reversed(valid)) if usable), count)
    # A partial scan's unobserved seam joins the blind rays at both ends.
    unobserved_angle = max(0.0, 2 * math.pi - count * scan.angle_increment)
    seam_blind_angle = (leading + trailing) * scan.angle_increment + unobserved_angle
    return max(longest * scan.angle_increment, seam_blind_angle) <= math.radians(max_blind_degrees)


class Authority:
    """The mode and command freshness policy; ROS callbacks only transport it."""

    def __init__(self, timeout):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        self.timeout = timeout
        self.mode = 'disabled'
        self.command = None
        self.received_at = None

    def set_mode(self, mode):
        if mode not in MODES:
            return False
        self.mode = mode
        self.clear_command()
        return True

    def clear_command(self):
        self.command = None
        self.received_at = None

    def receive(self, source, command, now):
        if source != self.mode:
            return False
        components = (command.linear.x, command.linear.y, command.linear.z,
                      command.angular.x, command.angular.y, command.angular.z)
        if not all(math.isfinite(value) for value in components):
            self.clear_command()
            return False
        self.command = command
        self.received_at = now
        return True

    def output(self, now):
        if (self.mode == 'disabled' or self.received_at is None or
                now - self.received_at > self.timeout):
            return Twist()
        return self.command


class ScanFreshness:
    """Fail closed on missing, delayed, future-dated, or interrupted scans."""

    def __init__(self, timeout):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('scan_timeout must be finite and positive')
        self.timeout = timeout
        self.expires_at = None
        self.stamp = None

    def invalidate(self, stamp):
        self.expires_at = None
        if self.stamp is None or stamp > self.stamp:
            # A previously valid scan cannot be replayed after bad sensor data.
            self.stamp = stamp

    def receive(self, stamp, ros_now, monotonic_now):
        age = ros_now - stamp
        if not 0 <= age <= self.timeout:
            self.expires_at = None
            return
        if self.stamp is not None and stamp <= self.stamp:
            # Duplicate samples cannot renew validity while ROS time is paused.
            # Older samples/clock resets fail closed until a newer stamp arrives.
            if stamp < self.stamp:
                self.expires_at = None
            return
        # A delayed scan only retains its remaining validity, even if ROS time pauses.
        self.expires_at = monotonic_now + self.timeout - age
        self.stamp = stamp

    def fresh(self, monotonic_now, ros_now):
        return (self.expires_at is not None and monotonic_now <= self.expires_at and
                0 <= ros_now - self.stamp <= self.timeout)


class VelocityAuthority(Node):
    def __init__(self):
        super().__init__('velocity_authority')
        self.declare_parameter('source_timeout', 0.25)
        timeout = self.get_parameter('source_timeout').value
        self.authority = Authority(float(timeout))
        self._nav_lease_id = None
        self._mode_epoch = 0
        self.declare_parameter('scan_timeout', 0.5)
        self.scan_freshness = ScanFreshness(float(self.get_parameter('scan_timeout').value))
        self.scan_quality = scan_quality_settings(self)
        self.tf_freshness = ScanFreshness(self.scan_freshness.timeout)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_frame = None
        self.scan_time = None
        self.create_subscription(LaserScan, '/scan_filtered', self.on_scan, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self._output_publishers = {
            mode: self.create_publisher(Twist, '/cmd_vel/authorized/' + mode, qos)
            for mode in MODES
        }
        mode_state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._mode_state_publisher = self.create_publisher(
            String, '/control/mode_state', mode_state_qos)
        self._mode_epoch_publisher = self.create_publisher(
            UInt64, '/control/mode_epoch', mode_state_qos)
        self.create_subscription(String, '/control/mode', self.on_mode, qos)
        # These callbacks and the mode subscription share the node's default
        # mutually exclusive callback group, even with a multithreaded executor.
        self.create_service(AcquireNav, '/control/acquire_nav', self.on_acquire_nav)
        self.create_service(ReleaseNav, '/control/release_nav', self.on_release_nav)
        self._input_subscriptions = [
            self.create_subscription(
                Twist, '/cmd_vel/' + source,
                lambda msg, source=source: self.on_command(source, msg), qos)
            for source in MODES if source != 'disabled'
        ]
        # A ROS-time timer would freeze if /clock is paused; the watchdog must not.
        self.create_timer(0.05, self.publish_current,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_control_state()

    def publish_control_state(self):
        self._mode_state_publisher.publish(String(data=self.authority.mode))
        self._mode_epoch_publisher.publish(UInt64(data=self._mode_epoch))

    def _transition_mode(self, mode):
        previous = self.authority.mode
        if not self.authority.set_mode(mode):
            raise ValueError('invalid internal control mode: ' + repr(mode))
        self._mode_epoch = (self._mode_epoch + 1) & ((1 << 64) - 1)
        # Clear the previous, possibly higher-priority mux input immediately.
        self._output_publishers[previous].publish(Twist())
        self.publish_current()
        self.publish_control_state()

    def on_mode(self, msg):
        if msg.data not in MODES:
            self.get_logger().warn('Ignoring invalid control mode: ' + repr(msg.data))
            return
        self._nav_lease_id = None
        self._transition_mode(msg.data)
        self.get_logger().info('Control mode: ' + msg.data)

    def on_acquire_nav(self, request, response):
        del request
        response.epoch = self._mode_epoch
        if self.authority.mode != 'disabled':
            response.message = 'control mode is ' + self.authority.mode
            return response
        # Zero is reserved for a rejected request. A fresh random ID also
        # makes stale releases from an earlier node process ineffective.
        lease_id = 0
        while lease_id == 0:
            lease_id = secrets.randbits(64)
        self._nav_lease_id = lease_id
        self._transition_mode('nav')
        response.success = True
        response.lease_id = lease_id
        response.epoch = self._mode_epoch
        response.message = 'nav acquired'
        return response

    def on_release_nav(self, request, response):
        if (self.authority.mode != 'nav' or self._nav_lease_id is None or
                request.lease_id != self._nav_lease_id):
            response.message = 'nav lease is no longer current'
            return response
        self._nav_lease_id = None
        self._transition_mode('disabled')
        response.success = True
        response.message = 'nav released'
        return response

    def on_command(self, source, msg):
        if source != self.authority.mode:
            return
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        if not (self.scan_freshness.fresh(now, ros_now) and
                self.transforms_ready(now, ros_now)):
            # A source command received during a sensor fault must not be
            # cached and played when observations recover.
            self.authority.clear_command()
            return
        if not self.authority.receive(source, msg, now):
            self.get_logger().warn('Rejected non-finite velocity command')

    def on_scan(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        was_fresh = self.scan_freshness.fresh(now, ros_now)
        if not has_usable_scan_data(msg, **self.scan_quality):
            self.scan_freshness.invalidate(stamp)
            self.scan_frame = None
            self.scan_time = None
            self.authority.clear_command()
            return
        if not was_fresh:
            # Also catch a gap if this callback runs before the watchdog timer.
            self.authority.clear_command()
        self.scan_frame = msg.header.frame_id
        self.scan_time = rclpy.time.Time.from_msg(msg.header.stamp)
        self.scan_freshness.receive(stamp, ros_now, now)

    def transforms_ready(self, now, ros_now):
        if not self.scan_frame or self.scan_time is None:
            return False
        try:
            # Collision monitor's base-shift correction depends on odometry TF.
            # Scan timestamps must be transformable and the latest dynamic TF
            # must remain fresh in both clock domains, even with a paused clock.
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            self.tf_freshness.receive(stamp, ros_now, now)
            return self.tf_freshness.fresh(now, ros_now) and self.tf_buffer.can_transform(
                'odom', self.scan_frame, self.scan_time)
        except TransformException:
            return False

    def publish_current(self):
        mode = self.authority.mode
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        ready = (self.scan_freshness.fresh(now, ros_now) and
                 self.transforms_ready(now, ros_now))
        if not ready:
            self.authority.clear_command()
        command = self.authority.output(now) if ready else Twist()
        self._output_publishers[mode].publish(command)


def main():
    rclpy.init()
    node = VelocityAuthority()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
