import math
import time

from abot_control_interfaces.srv import AcquireNav, ReleaseNav
from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, UInt64

from abot_navigation.velocity_authority import (
    Authority, VelocityAuthority, ScanFreshness, has_usable_scan_data)


def command(vx=0.0, vy=0.0, wz=0.0):
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = wz
    return msg


def test_teleop_loss_cannot_resume_old_navigation():
    authority = Authority(timeout=0.25)
    assert authority.set_mode('nav')
    assert authority.receive('nav', command(vx=0.1), now=0.0)
    assert authority.output(0.1).linear.x == 0.1

    assert authority.set_mode('teleop')
    assert authority.output(0.11).linear.x == 0.0
    assert not authority.receive('nav', command(vx=0.2), now=0.12)
    assert authority.receive('teleop', command(vy=-0.2), now=0.13)
    assert authority.output(0.2).linear.y == -0.2
    assert authority.output(0.4).linear.y == 0.0
    assert authority.mode == 'teleop'
    assert not authority.receive('nav', command(vx=0.2), now=0.41)
    assert authority.output(0.42).linear.x == 0.0

    assert authority.set_mode('nav')
    assert authority.output(0.43).linear.x == 0.0
    assert authority.receive('nav', command(vx=0.15), now=0.44)
    assert authority.output(0.45).linear.x == 0.15


def test_invalid_mode_and_non_finite_command_fail_closed():
    authority = Authority(timeout=0.25)
    assert not authority.set_mode('unknown')
    assert authority.mode == 'disabled'
    assert authority.set_mode('tracking')
    assert authority.receive('tracking', command(vy=0.1), now=1.0)
    assert not authority.receive('tracking', command(vy=float('nan')), now=1.1)
    assert authority.output(1.11).linear.y == 0.0
    assert authority.set_mode('disabled')
    assert not authority.receive('tracking', command(vy=0.1), now=1.2)
    assert authority.output(1.21).linear.y == 0.0


@pytest.mark.parametrize('timeout', [0.0, -1.0, float('nan'), float('inf')])
def test_invalid_timeout(timeout):
    with pytest.raises(ValueError):
        Authority(timeout)


def test_node_constructs_and_starts_disabled():
    rclpy.init()
    node = None
    try:
        node = VelocityAuthority()
        assert node.authority.mode == 'disabled'
        assert set(node._output_publishers) == {
            'disabled', 'nav', 'tracking', 'teleop'}
        assert len(node._input_subscriptions) == 3
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_mode_state_late_subscriber_and_command_echo():
    """A late observer sees startup state and one echo per valid command."""
    rclpy.init()
    authority = probe = executor = None
    try:
        authority = VelocityAuthority()
        probe = rclpy.create_node('mode_state_probe')
        executor = SingleThreadedExecutor()
        executor.add_node(authority)
        executor.add_node(probe)
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        states = []
        epochs = []
        probe.create_subscription(
            String, '/control/mode_state', lambda msg: states.append(msg.data), state_qos)
        probe.create_subscription(
            UInt64, '/control/mode_epoch', lambda msg: epochs.append(msg.data), state_qos)
        command_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        command_pub = probe.create_publisher(String, '/control/mode', command_qos)

        def until(predicate, timeout=8.0):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            assert predicate()

        # Subscription is created after VelocityAuthority published its first
        # state, so this requires the publisher's transient-local history.
        until(lambda: states == ['disabled'] and epochs == [0])
        until(lambda: command_pub.get_subscription_count() >= 1)
        command_pub.publish(String(data='nav'))
        until(lambda: states == ['disabled', 'nav'] and epochs == [0, 1])
        assert authority.authority.mode == 'nav'
        command_pub.publish(String(data='nav'))
        until(lambda: states == ['disabled', 'nav', 'nav'] and epochs == [0, 1, 2])
        assert authority.authority.mode == 'nav'
    finally:
        if executor is not None:
            executor.shutdown()
        if probe is not None:
            probe.destroy_node()
        if authority is not None:
            authority.destroy_node()
        rclpy.shutdown()


def test_invalid_mode_callback_leaves_state_and_publishes_nothing():
    rclpy.init()
    node = VelocityAuthority()
    original_publisher = node._mode_state_publisher
    original_epoch_publisher = node._mode_epoch_publisher
    try:
        node.on_mode(String(data='nav'))
        recorded = []
        node._mode_state_publisher = type(
            'Recorder', (), {'publish': lambda self, msg: recorded.append(msg.data)})()
        node._mode_epoch_publisher = type(
            'Recorder', (), {'publish': lambda self, msg: recorded.append(msg.data)})()
        node.on_mode(String(data='unknown'))
        assert node.authority.mode == 'nav'
        assert node._mode_epoch == 1
        assert recorded == []
    finally:
        node._mode_state_publisher = original_publisher
        node._mode_epoch_publisher = original_epoch_publisher
        node.destroy_node()
        rclpy.shutdown()


def test_nav_lease_services_reject_stale_release_and_busy_acquire():
    rclpy.init()
    authority = probe = executor = None
    try:
        authority = VelocityAuthority()
        probe = rclpy.create_node('nav_lease_probe')
        executor = SingleThreadedExecutor()
        executor.add_node(authority)
        executor.add_node(probe)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        mode_pub = probe.create_publisher(String, '/control/mode', qos)
        acquire = probe.create_client(AcquireNav, '/control/acquire_nav')
        release = probe.create_client(ReleaseNav, '/control/release_nav')

        def until(predicate, timeout=8.0):
            deadline = time.monotonic() + timeout
            while not predicate() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            assert predicate()

        def call(client, request):
            future = client.call_async(request)
            until(future.done)
            assert future.exception() is None
            return future.result()

        def set_external_mode(mode, expected_epoch):
            mode_pub.publish(String(data=mode))
            until(lambda: authority.authority.mode == mode and
                  authority._mode_epoch == expected_epoch)

        until(lambda: acquire.service_is_ready() and release.service_is_ready() and
              mode_pub.get_subscription_count() >= 1)
        grant = call(acquire, AcquireNav.Request())
        assert grant.success and grant.lease_id != 0 and grant.epoch == 1
        assert authority.authority.mode == 'nav'
        busy = call(acquire, AcquireNav.Request())
        assert not busy.success and busy.lease_id == 0 and busy.epoch == 1

        set_external_mode('teleop', 2)
        old_release = call(release, ReleaseNav.Request(lease_id=grant.lease_id))
        assert not old_release.success
        assert authority.authority.mode == 'teleop'
        busy = call(acquire, AcquireNav.Request())
        assert not busy.success and busy.lease_id == 0 and busy.epoch == 2

        set_external_mode('disabled', 3)
        set_external_mode('nav', 4)
        old_release = call(release, ReleaseNav.Request(lease_id=grant.lease_id))
        assert not old_release.success
        assert authority.authority.mode == 'nav' and authority._mode_epoch == 4

        set_external_mode('disabled', 5)
        grant2 = call(acquire, AcquireNav.Request())
        assert grant2.success and grant2.lease_id != grant.lease_id and grant2.epoch == 6
        set_external_mode('nav', 7)  # Repeating a valid mode also revokes a lease.
        assert not call(release, ReleaseNav.Request(lease_id=grant2.lease_id)).success
        assert authority.authority.mode == 'nav'

        set_external_mode('disabled', 8)
        grant3 = call(acquire, AcquireNav.Request())
        assert grant3.success and grant3.epoch == 9
        assert call(release, ReleaseNav.Request(lease_id=grant3.lease_id)).success
        assert authority.authority.mode == 'disabled' and authority._mode_epoch == 10
    finally:
        if executor is not None:
            executor.shutdown()
        if probe is not None:
            probe.destroy_node()
        if authority is not None:
            authority.destroy_node()
        rclpy.shutdown()


def test_scan_loss_and_replayed_timestamps_fail_closed():
    guard = ScanFreshness(0.5)
    assert not guard.fresh(10.0, 100.0)
    guard.receive(stamp=100.0, ros_now=100.1, monotonic_now=10.0)
    assert guard.fresh(10.3, 100.4)
    assert not guard.fresh(10.6, 100.1)  # Still expires when ROS time is paused.
    guard.receive(stamp=100.0, ros_now=101.0, monotonic_now=10.7)
    assert not guard.fresh(10.7, 101.0)  # Replayed stale scans do not refresh it.
    guard.receive(stamp=102.0, ros_now=101.0, monotonic_now=10.8)
    assert not guard.fresh(10.8, 101.0)
    guard.receive(stamp=101.0, ros_now=101.1, monotonic_now=10.9)
    assert guard.fresh(11.0, 101.2)


def test_delayed_scan_keeps_only_remaining_validity_and_clock_jumps_fail_closed():
    guard = ScanFreshness(0.5)
    guard.receive(stamp=100.0, ros_now=100.49, monotonic_now=10.0)
    assert guard.fresh(10.005, 100.49)
    assert not guard.fresh(10.02, 100.49)
    guard.receive(stamp=101.0, ros_now=101.0, monotonic_now=11.0)
    assert not guard.fresh(11.01, 102.0)
    assert not guard.fresh(11.01, 100.0)


@pytest.mark.parametrize('timeout', [0.0, -1.0, float('nan'), float('inf')])
def test_invalid_scan_timeout(timeout):
    with pytest.raises(ValueError):
        ScanFreshness(timeout)


def test_repeated_samples_cannot_extend_expiry_with_paused_ros_clock():
    guard = ScanFreshness(0.5)
    guard.receive(100.0, 100.0, 10.0)
    guard.receive(100.0, 100.0, 10.2)
    assert guard.fresh(10.3, 100.0)
    guard.receive(100.0, 100.0, 11.0)
    assert not guard.fresh(11.0, 100.0)
    guard.receive(100.1, 100.1, 11.1)
    assert guard.fresh(11.1, 100.1)


def test_clock_reset_requires_newer_sample_to_recover():
    guard = ScanFreshness(0.5)
    guard.receive(100.0, 100.0, 10.0)
    guard.receive(99.0, 99.0, 10.1)
    assert not guard.fresh(10.1, 99.0)
    guard.receive(100.0, 100.0, 10.2)
    assert not guard.fresh(10.2, 100.0)
    guard.receive(100.1, 100.1, 10.3)
    assert guard.fresh(10.3, 100.1)


@pytest.mark.parametrize('ranges,expected', [
    ([], False),
    ([math.nan] * 360, False),
    ([-math.inf] * 360, False),
    ([0.01, 11.0] * 180, False),
    ([math.inf] * 360, True),
    ([math.nan] * 359 + [math.inf], False),
    ([math.inf] * 100 + [math.nan] * 40 + [math.inf] * 220, False),
    ([math.inf] * 341 + [math.nan] * 19, True),
    ([0.05, 10.0] * 180, True),
])
def test_scan_data_requires_usable_ray(ranges, expected):
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = 2 * math.pi / 359
    scan.range_min = 0.05
    scan.range_max = 10.0
    scan.ranges = ranges
    assert has_usable_scan_data(scan) is expected


@pytest.mark.parametrize('range_min,range_max', [
    (0.0, 0.0), (-0.1, 10.0), (math.nan, 10.0), (0.05, math.inf),
    (2.0, 10.0), (0.05, 0.2),
])
def test_scan_data_rejects_invalid_limits(range_min, range_max):
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_max = math.pi
    scan.angle_increment = 2 * math.pi / 359
    scan.range_min = range_min
    scan.range_max = range_max
    scan.ranges = [math.inf] * 360
    assert not has_usable_scan_data(scan)


def test_scan_data_rejects_inconsistent_angles_and_narrow_field():
    scan = LaserScan()
    scan.range_min, scan.range_max = 0.05, 10.0
    scan.ranges = [math.inf] * 360
    scan.angle_min, scan.angle_max = -math.pi, math.pi
    scan.angle_increment = 0.0
    assert not has_usable_scan_data(scan)
    scan.angle_increment = math.pi / 359
    assert not has_usable_scan_data(scan)  # angle_max disagrees with beam count.
    scan.angle_max = 0.0
    assert not has_usable_scan_data(scan)


def test_unobserved_seam_and_end_blind_rays_count_together():
    scan = LaserScan()
    scan.range_min, scan.range_max = 0.05, 10.0
    scan.angle_min = -math.radians(175)
    scan.angle_increment = math.radians(1)
    scan.ranges = [math.inf] * 331 + [math.nan] * 19
    scan.angle_max = scan.angle_min + 349 * scan.angle_increment
    assert not has_usable_scan_data(scan)  # 19 blind degrees + 10 unobserved.
    scan.ranges = [math.inf] * 341 + [math.nan] * 9
    assert has_usable_scan_data(scan)


def test_invalid_scan_data_clears_freshness_and_blocks_command():
    rclpy.init()
    node = VelocityAuthority()
    try:
        assert node.authority.set_mode('nav')
        node.transforms_ready = lambda now, ros_now: True
        published = []
        node._output_publishers['nav'] = type(
            'Recorder', (), {'publish': lambda self, msg: published.append(msg)})()

        def send_scan(stamp_ns, ranges):
            scan = LaserScan()
            scan.header.frame_id = 'laser_link'
            scan.header.stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
            scan.range_min = 0.05
            scan.range_max = 10.0
            scan.angle_min = -math.pi
            scan.angle_max = math.pi
            scan.angle_increment = 2 * math.pi / 359
            scan.ranges = ranges
            node.on_scan(scan)

        now_ns = node.get_clock().now().nanoseconds
        for index, bad_ranges in enumerate(([], [math.nan] * 360)):
            old_stamp = now_ns - (300 - 100 * index) * 1_000_000
            send_scan(old_stamp, [math.inf] * 360)
            node.on_command('nav', command(vx=0.1))
            node.publish_current()
            assert published[-1].linear.x == 0.1  # Clear-space rays are valid.

            send_scan(old_stamp + 50_000_000, bad_ranges)
            node.on_command('nav', command(vx=0.2))
            node.publish_current()
            assert published[-1].linear.x == 0.0
            assert node.scan_frame is None
            send_scan(old_stamp, [0.5] * 360)
            node.publish_current()
            assert published[-1].linear.x == 0.0  # Older valid data cannot revive it.

        send_scan(node.get_clock().now().nanoseconds, [0.5] * 360)
        node.publish_current()
        assert published[-1].linear.x == 0.0  # No pre-fault command resumes.
        node.on_command('nav', command(vx=0.1))
        node.publish_current()
        assert published[-1].linear.x == 0.1

        node.transforms_ready = lambda now, ros_now: False
        node.on_command('nav', command(vx=0.2))
        node.publish_current()
        assert published[-1].linear.x == 0.0
        node.transforms_ready = lambda now, ros_now: True
        node.publish_current()
        assert published[-1].linear.x == 0.0  # TF recovery also needs a new command.
        node.on_command('nav', command(vx=0.1))
        node.publish_current()
        assert published[-1].linear.x == 0.1

        node.scan_freshness.expires_at = time.monotonic() - 1.0
        send_scan(node.get_clock().now().nanoseconds, [0.5] * 360)
        node.publish_current()
        assert published[-1].linear.x == 0.0  # Scan gap observed at recovery.
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_missing_stale_and_wrong_frame_tf_block_motion():
    from geometry_msgs.msg import TransformStamped
    rclpy.init()
    node = VelocityAuthority()
    try:
        node.scan_frame = 'laser_link'
        node.scan_time = rclpy.time.Time(seconds=100)
        assert not node.transforms_ready(10.0, 100.0)
        transform = TransformStamped()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.header.stamp = node.scan_time.to_msg()
        transform.transform.rotation.w = 1.0
        node.tf_buffer.set_transform(transform, 'test')
        assert not node.transforms_ready(10.0, 100.0)  # Laser TF is missing.
        transform.header.frame_id = 'base_footprint'
        transform.child_frame_id = 'laser_link'
        node.tf_buffer.set_transform_static(transform, 'test')
        assert node.transforms_ready(10.1, 100.1)
        assert not node.transforms_ready(10.6, 100.1)  # Frozen TF/ROS clock.
        assert not node.transforms_ready(10.7, 100.7)  # Advancing ROS clock.
    finally:
        node.destroy_node()
        rclpy.shutdown()
