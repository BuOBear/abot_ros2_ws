"""Exercise the installed lifecycle driver against a simulated serial board."""

import errno
import os
import select
import struct
import subprocess
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def frame(message_id, payload=b''):
    data = bytes((0x5A, message_id, len(payload))) + payload
    return data + bytes((sum(data) & 0xFF,))


class Board:
    def __init__(self, master):
        self.master = master
        self.feedback = True
        self.frames = []
        self.cv = threading.Condition()
        self.done = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def close(self):
        self.done.set()
        self.thread.join(timeout=2)

    def set_feedback(self, enabled):
        with self.cv:
            self.feedback = enabled

    def snapshot(self):
        with self.cv:
            return list(self.frames)

    def wait_for(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        with self.cv:
            while True:
                matches = [entry for entry in self.frames if predicate(entry)]
                if matches:
                    return matches[0]
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self.cv.wait(remaining)

    def _handle(self, message_id, payload):
        with self.cv:
            self.frames.append((time.monotonic(), message_id, payload))
            feedback = self.feedback
            self.cv.notify_all()
        if message_id == 0:
            reply = frame(0, b'v' * 32)
        elif message_id == 5 and feedback:
            # vx=0.12, vy=0.34, wz=-0.05, x=1.5, y=-2, yaw=0.25 (SI)
            reply = frame(5, struct.pack('<hhh i i h', 12, 34, -5, 150, -200, 25))
        elif message_id == 7 and feedback:
            reply = frame(7, struct.pack('<9f', 1, 2, 9.8, .1, .2, .3, 100, 200, 300))
        else:
            return
        try:
            os.write(self.master, reply)
        except OSError:
            pass

    def _run(self):
        pending = bytearray()
        while not self.done.is_set():
            readable, _, _ = select.select([self.master], [], [], .02)
            if not readable:
                continue
            try:
                chunk = os.read(self.master, 512)
            except OSError as exc:
                # Linux PTY masters report EIO while no slave is open.
                if exc.errno == errno.EIO:
                    time.sleep(.01)
                    continue
                return
            pending.extend(chunk)
            while pending:
                if pending[0] != 0x5A:
                    del pending[0]
                    continue
                if len(pending) < 4:
                    break
                length = pending[2]
                if length > 64:
                    del pending[0]
                    continue
                size = 4 + length
                if len(pending) < size:
                    break
                incoming = bytes(pending[:size])
                del pending[:size]
                if (sum(incoming[:-1]) & 0xFF) == incoming[-1]:
                    self._handle(incoming[1], incoming[3:-1])


def velocity(entry):
    return struct.unpack('<hhh', entry[2]) if entry[1] == 4 and len(entry[2]) == 6 else None


def wait_ros(node, condition, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
        if condition():
            return True
    return False


def change_state(node, client, transition_id):
    request = ChangeState.Request()
    request.transition.id = transition_id
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5)
    assert future.done(), f'lifecycle transition {transition_id} timed out'
    assert future.result() is not None and future.result().success, transition_id


def publish_command(publisher, x=0.0, y=0.0, yaw=0.0):
    command = Twist()
    command.linear.x = x
    command.linear.y = y
    command.angular.z = yaw
    publisher.publish(command)


def test_lifecycle_watchdog_and_reconnect(tmp_path):
    driver = os.environ['ABOT_BASE_DRIVER']
    master, slave = os.openpty()
    port = os.ttyname(slave)
    os.close(slave)
    board = Board(master)
    log_path = tmp_path / 'base_driver.log'
    log_file = log_path.open('w+')
    process = None
    node = None
    prior_domain = os.environ.get('ROS_DOMAIN_ID')
    prior_localhost = os.environ.get('ROS_LOCALHOST_ONLY')
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    try:
        process = subprocess.Popen([
            driver, '--ros-args', '-p', f'port:={port}',
            '-p', 'response_timeout_ms:=60',
            '-p', 'command_period_ms:=30',
            '-p', 'odom_period_ms:=30',
            '-p', 'imu_period_ms:=30',
            '-p', 'command_timeout_ms:=1500',
            '-p', 'reconnect_delay_ms:=100',
            '-p', 'odom_pose_x_stddev_m:=0.2',
            '-p', 'odom_pose_y_stddev_m:=0.5',
            '-p', 'odom_twist_vx_stddev_mps:=0.1',
            '-p', 'odom_twist_vy_stddev_mps:=0.3',
        ], stdout=log_file, stderr=subprocess.STDOUT)
        rclpy.init()
        node = rclpy.create_node('abot_hardware_pty_test')
        lifecycle = node.create_client(ChangeState, '/abot_hardware/change_state')
        assert lifecycle.wait_for_service(timeout_sec=8), 'lifecycle service unavailable'
        odom = []
        imu = []
        node.create_subscription(Odometry, '/wheel_odom', odom.append, 10)
        node.create_subscription(Imu, '/imu/data_raw', imu.append, qos_profile_sensor_data)
        command_pub = node.create_publisher(Twist, '/cmd_vel', 1)

        change_state(node, lifecycle, Transition.TRANSITION_CONFIGURE)
        assert wait_ros(node, lambda: command_pub.get_subscription_count() > 0)
        change_state(node, lifecycle, Transition.TRANSITION_ACTIVATE)
        assert board.wait_for(lambda e: e[1] == 4 and velocity(e) == (0, 0, 0), 3)
        assert wait_ros(node, lambda: bool(odom) and bool(imu))
        assert odom[-1].twist.twist.linear.y == pytest.approx(.34)
        assert odom[-1].pose.pose.position.y == pytest.approx(-2)
        assert odom[-1].pose.covariance[0] == pytest.approx(.04)
        assert odom[-1].pose.covariance[7] == pytest.approx(.25)
        assert odom[-1].twist.covariance[0] == pytest.approx(.01)
        assert odom[-1].twist.covariance[7] == pytest.approx(.09)
        assert imu[-1].orientation_covariance[0] == -1
        assert imu[-1].angular_velocity.z == pytest.approx(.3)

        # A physical base must reject a runtime switch to simulated time.
        parameters = node.create_client(SetParameters, '/abot_hardware/set_parameters')
        assert parameters.wait_for_service(timeout_sec=3)
        request = SetParameters.Request()
        request.parameters = [Parameter('use_sim_time', value=True).to_parameter_msg()]
        future = parameters.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        assert future.done() and not future.result().results[0].successful

        # Expired input reaches zero even while feedback requests run.
        after = time.monotonic()
        publish_command(command_pub, x=.12, y=.35)
        moving = board.wait_for(lambda e: e[0] >= after and velocity(e) == (12, 35, 0), 3)
        assert moving, 'no nonzero velocity reached the simulated board'
        stopped = board.wait_for(lambda e: e[0] > moving[0] and velocity(e) == (0, 0, 0), 2)
        assert stopped and stopped[0] - moving[0] < 1.8, 'watchdog did not stop the board'

        # An excessive command clears the previously accepted velocity.
        after = time.monotonic()
        publish_command(command_pub, y=.2)
        moving = board.wait_for(lambda e: e[0] >= after and velocity(e) == (0, 20, 0), 3)
        assert moving
        publish_command(command_pub, y=3.0)
        assert board.wait_for(lambda e: e[0] > moving[0] and velocity(e) == (0, 0, 0), 1)

        # Deactivation stops, and reactivation cannot replay its prior Twist.
        after = time.monotonic()
        publish_command(command_pub, y=.2)
        moving = board.wait_for(lambda e: e[0] >= after and velocity(e) == (0, 20, 0), 3)
        assert moving
        change_state(node, lifecycle, Transition.TRANSITION_DEACTIVATE)
        stopped = board.wait_for(lambda e: e[0] > moving[0] and velocity(e) == (0, 0, 0), 1)
        assert stopped
        change_state(node, lifecycle, Transition.TRANSITION_ACTIVATE)
        assert board.wait_for(lambda e: e[0] > stopped[0] and e[1] == 0, 3)
        start = time.monotonic()
        while time.monotonic() - start < .25:
            rclpy.spin_once(node, timeout_sec=.02)
        assert not any(e[0] >= stopped[0] and velocity(e) == (0, 20, 0)
                       for e in board.snapshot())

        # Drop feedback until the worker closes and reopens the same PTY.
        after = time.monotonic()
        publish_command(command_pub, y=.21)
        moving = board.wait_for(lambda e: e[0] >= after and velocity(e) == (0, 21, 0), 3)
        assert moving
        board.set_feedback(False)
        reconnect = board.wait_for(
            lambda e: e[1] == 0 and e[0] > moving[0], 3)
        assert reconnect, 'serial worker did not reopen the PTY'
        assert any(moving[0] < e[0] < reconnect[0] and velocity(e) == (0, 0, 0)
                   for e in board.snapshot()), 'disconnect did not attempt a zero frame'
        assert time.monotonic() - after < 1.5, 'old command expired before reconnect test'
        # Drain any feedback already queued when replies were disabled. While
        # the board stays silent, the driver must not republish old samples.
        start = time.monotonic()
        while time.monotonic() - start < .1:
            rclpy.spin_once(node, timeout_sec=.02)
        feedback_count = len(odom), len(imu)
        start = time.monotonic()
        while time.monotonic() - start < .25:
            rclpy.spin_once(node, timeout_sec=.02)
        assert (len(odom), len(imu)) == feedback_count, 'stale feedback was published'
        board.set_feedback(True)
        assert board.wait_for(lambda e: e[1] == 5 and e[0] > reconnect[0], 3)
        start = time.monotonic()
        while time.monotonic() - start < .25:
            rclpy.spin_once(node, timeout_sec=.02)
        assert not any(e[0] >= reconnect[0] and velocity(e) == (0, 21, 0)
                       for e in board.snapshot()), 'old command replayed after reconnect'
        after = time.monotonic()
        publish_command(command_pub, y=.22)
        assert board.wait_for(lambda e: e[0] >= after and velocity(e) == (0, 22, 0), 3)
        change_state(node, lifecycle, Transition.TRANSITION_DEACTIVATE)
        assert all(e[1] in (0, 4, 5, 7) for e in board.snapshot()), \
            'driver sent a firmware parameter or odometry-reset frame'
    except Exception as exc:
        log_file.flush()
        raise AssertionError(f'{exc}\ndriver log:\n{log_path.read_text()}') from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        board.close()
        os.close(master)
        log_file.close()
        if prior_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = prior_domain
        if prior_localhost is None:
            os.environ.pop('ROS_LOCALHOST_ONLY', None)
        else:
            os.environ['ROS_LOCALHOST_ONLY'] = prior_localhost
