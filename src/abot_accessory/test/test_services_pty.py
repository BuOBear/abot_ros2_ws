import os
import pty
import select
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from std_srvs.srv import Trigger

os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
os.environ["ROS_LOCALHOST_ONLY"] = "1"
_fastdds_profile = Path(__file__).resolve().parents[3] / "deployment/fastdds-local-test.xml"
if _fastdds_profile.is_file():
    os.environ.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE", str(_fastdds_profile))


SHOOT = bytes((0x55, 0x01, 0x12, 0x00, 0x00, 0x00, 0x01, 0x69))
STOP = bytes((0x55, 0x01, 0x11, 0x00, 0x00, 0x00, 0x01, 0x68))


def _read_exact(fd, count, timeout=2.0):
    data = bytearray()
    end = time.monotonic() + timeout
    while len(data) < count:
        remaining = end - time.monotonic()
        if remaining <= 0:
            break
        readable, _, _ = select.select([fd], [], [], remaining)
        if not readable:
            break
        data.extend(os.read(fd, count - len(data)))
    return bytes(data)


def _call(node, service_name):
    client = node.create_client(Trigger, service_name)
    assert client.wait_for_service(timeout_sec=5.0), f"service unavailable: {service_name}"
    future = client.call_async(Trigger.Request())
    end = time.monotonic() + 5.0
    while not future.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert future.done(), f"service timed out: {service_name}"
    response = future.result()
    node.destroy_client(client)
    return response


def _start_node(args):
    executable = os.environ["ABOT_ACCESSORY_DRIVER"]
    return subprocess.Popen(
        [executable, "--ros-args", *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
    )


def _stop_node(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        return process.communicate(timeout=5.0)[0]
    except subprocess.TimeoutExpired:
        process.kill()
        return process.communicate()[0]


def test_disabled_by_default_sends_no_frame_and_services_fail_clearly():
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    process = _start_node(["-p", f"port:={path}"])
    rclpy.init()
    node = rclpy.create_node("accessory_service_test_disabled")
    try:
        response = _call(node, "/accessory/shoot")
        assert not response.success
        assert "disabled" in response.message
        ready, _, _ = select.select([master], [], [], 0.15)
        assert not ready, "disabled node unexpectedly wrote to its configured PTY"
    finally:
        node.destroy_node()
        rclpy.shutdown()
        output = _stop_node(process)
        os.close(master)
        os.close(slave)
    assert "no serial port opened and no frame sent" in output


def test_explicit_services_send_exact_frames_and_shutdown_stops_active_accessory():
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    process = _start_node(["-p", "enabled:=true", "-p", f"port:={path}"])
    rclpy.init()
    node = rclpy.create_node("accessory_service_test_enabled")
    try:
        shoot = _call(node, "/accessory/shoot")
        assert shoot.success, shoot.message
        assert _read_exact(master, 8) == SHOOT

        stop = _call(node, "/accessory/stop")
        assert stop.success, stop.message
        assert _read_exact(master, 8) == STOP

        shoot = _call(node, "/accessory/shoot")
        assert shoot.success, shoot.message
        assert _read_exact(master, 8) == SHOOT
    finally:
        node.destroy_node()
        rclpy.shutdown()
        output = _stop_node(process)
    try:
        assert "sent stop frame before closing accessory port" in output
        assert _read_exact(master, 8, timeout=0.1) == STOP
    finally:
        os.close(master)
        os.close(slave)


def test_disconnect_is_reported_in_service_response():
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    process = _start_node(["-p", "enabled:=true", "-p", f"port:={path}"])
    rclpy.init()
    node = rclpy.create_node("accessory_service_test_disconnect")
    try:
        client = node.create_client(Trigger, "/accessory/shoot")
        assert client.wait_for_service(timeout_sec=5.0)
        os.close(master)
        master = -1
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert future.done()
        response = future.result()
        assert not response.success
        assert "write failed" in response.message
        assert response.message
    finally:
        node.destroy_node()
        rclpy.shutdown()
        output = _stop_node(process)
        if master >= 0:
            os.close(master)
        os.close(slave)
    assert "shoot frame write failed" in output
