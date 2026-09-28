import math

import pytest
from sensor_msgs.msg import LaserScan

from abot_tracking.lidar_follower import LidarFollowPolicy


def scan(stamp, returns=(), *, frame='laser_link', angle_min=-math.pi,
         angle_max=math.pi, count=361):
    msg = LaserScan()
    msg.header.frame_id = frame
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    msg.angle_min = angle_min
    msg.angle_max = angle_max
    msg.angle_increment = (angle_max - angle_min) / (count - 1)
    msg.range_min = 0.1
    msg.range_max = 8.0
    msg.ranges = [math.inf] * count
    for angle, distance in returns:
        index = round((angle - angle_min) / msg.angle_increment)
        msg.ranges[index] = distance
    return msg


def receive(policy, msg, at):
    policy.receive_scan(msg, at, at)
    return policy.command(at, at)


def test_two_scans_corrobate_closest_forward_return_and_default_to_yaw_only():
    policy = LidarFollowPolicy()
    assert receive(policy, scan(10.0, [(0.3, 1.6), (0.0, 2.0)]), 10.0).angular.z == 0
    command = receive(policy, scan(10.1, [(0.3, 1.55), (0.0, 2.0)]), 10.1)
    assert command.angular.z > 0
    assert command.angular.z <= 0.25
    assert command.linear.x == 0
    assert command.linear.y == 0
    assert command.angular.x == 0


def test_unconfirmed_noise_outside_sector_and_target_loss_stop():
    policy = LidarFollowPolicy()
    receive(policy, scan(10.0, [(0.25, 1.7), (2.0, 0.4)]), 10.0)
    assert receive(policy, scan(10.1, [(0.25, 1.65), (2.0, 0.4)]), 10.1).angular.z > 0
    assert receive(policy, scan(10.2, [(0.25, 0.45)]), 10.2).angular.z == 0
    assert receive(policy, scan(10.3), 10.3).angular.z == 0


def test_explicit_linear_enable_alignment_and_distance_bounds():
    policy = LidarFollowPolicy(enable_linear_motion=True)
    receive(policy, scan(10.0, [(0.0, 2.0)]), 10.0)
    command = receive(policy, scan(10.1, [(0.0, 2.0)]), 10.1)
    assert command.linear.x == 0.10
    receive(policy, scan(10.2, [(0.3, 2.0)]), 10.2)
    command = receive(policy, scan(10.3, [(0.3, 2.0)]), 10.3)
    assert command.linear.x == 0
    assert command.angular.z > 0
    receive(policy, scan(10.4, [(0.0, 0.7)]), 10.4)
    command = receive(policy, scan(10.5, [(0.0, 0.7)]), 10.5)
    assert -0.10 <= command.linear.x < 0
    receive(policy, scan(10.6, [(0.0, 1.05)]), 10.6)
    command = receive(policy, scan(10.7, [(0.0, 1.05)]), 10.7)
    assert command.linear.x == 0


def test_stale_receipt_stamp_and_scan_gap_zero_then_reacquire():
    policy = LidarFollowPolicy()
    receive(policy, scan(10.0, [(0.3, 1.5)]), 10.0)
    assert receive(policy, scan(10.1, [(0.3, 1.5)]), 10.1).angular.z > 0
    assert policy.command(10.51, 10.51).angular.z == 0
    # A paused ROS clock cannot preserve a received target indefinitely.
    assert policy.command(10.51, 10.1).angular.z == 0
    assert receive(policy, scan(10.2, [(0.3, 1.5)]), 10.7).angular.z == 0
    assert receive(policy, scan(10.8, [(0.3, 1.5)]), 10.8).angular.z == 0
    assert receive(policy, scan(10.9, [(0.3, 1.5)]), 10.9).angular.z > 0


def test_invalid_geometry_frame_range_and_replay_clear_target():
    policy = LidarFollowPolicy()
    receive(policy, scan(10.0, [(0.2, 1.5)]), 10.0)
    assert receive(policy, scan(10.1, [(0.2, 1.5)]), 10.1).angular.z > 0
    assert receive(policy, scan(10.2, [(0.2, 1.5)], frame='other'), 10.2).angular.z == 0
    assert receive(policy, scan(10.3, [(0.2, 1.5)]), 10.3).angular.z == 0
    assert receive(policy, scan(10.4, [(0.2, 1.5)]), 10.4).angular.z > 0
    assert receive(policy, scan(10.4, [(0.2, 1.5)]), 10.4).angular.z == 0
    partial = scan(10.5, [(0.2, 1.5)], angle_min=-0.2, angle_max=0.2)
    assert receive(policy, partial, 10.5).angular.z == 0
    bad = scan(10.6, [(0.2, 1.5)])
    for index in range(170, 195):
        bad.ranges[index] = math.nan
    assert receive(policy, bad, 10.6).angular.z == 0


def test_clock_rewind_requires_two_new_scans_and_future_stamps_fail():
    policy = LidarFollowPolicy()
    receive(policy, scan(10.0, [(0.2, 1.5)]), 10.0)
    assert receive(policy, scan(10.1, [(0.2, 1.5)]), 10.1).angular.z > 0
    assert policy.command(10.2, 1.0).angular.z == 0
    policy.receive_scan(scan(1.1, [(0.2, 1.5)]), 10.3, 1.1)
    assert policy.command(10.3, 1.1).angular.z == 0
    policy.receive_scan(scan(1.2, [(0.2, 1.5)]), 10.4, 1.2)
    assert policy.command(10.4, 1.2).angular.z > 0
    policy.receive_scan(scan(2.0, [(0.2, 1.5)]), 10.5, 1.3)
    assert policy.command(10.5, 1.3).angular.z == 0


@pytest.mark.parametrize('options', [
    {'enable_linear_motion': 1}, {'window_beams': -1},
    {'max_scan_gap': 0.5}, {'forward_half_angle': math.nan},
    {'target_distance': 3.0}, {'linear_alignment_angle': 0.8},
])
def test_bad_parameters_rejected(options):
    with pytest.raises(ValueError):
        LidarFollowPolicy(**options)
