"""Real rclpy service and pyserial on a PTY, without any physical device."""
import os
import select
import yaml
import rclpy
from rclpy.executors import SingleThreadedExecutor
from jy_real_interfaces.srv import SetServoAngle
from jy_real_robot.arm_node import Arm
from jy_real_robot.servo import ACTUATORS


def test_ros_service_reports_unverified_and_no_joint_states(tmp_path):
    master, slave = os.openpty()
    profile = tmp_path / 'profile.yaml'
    profile.write_text(yaml.safe_dump({'calibration_verified': True, 'actuators': {
        name: {'channel': chr(65 + i), 'min_deg': 40, 'max_deg': 120}
        for i, name in enumerate(ACTUATORS)}}))
    rclpy.init(args=['--ros-args', '-p', f'profile:={profile}', '-p', f'port:={os.ttyname(slave)}'])
    arm = client_node = executor = None
    try:
        arm = Arm()
        client_node = rclpy.create_node('test_arm_client')
        executor = SingleThreadedExecutor()
        executor.add_node(arm)
        executor.add_node(client_node)
        client = client_node.create_client(SetServoAngle, '/arm/set_servo_angle')
        assert client.wait_for_service(timeout_sec=3)
        assert not select.select([master], [], [], 0.02)[0]
        request = SetServoAngle.Request(actuator='gripper', board_angle_deg=100)
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=3)
        assert future.done()
        assert future.result().sent_unverified
        assert 'UNVERIFIED' in future.result().message
        assert select.select([master], [], [], 0.5)[0]
        assert os.read(master, 100) == b'$F100#'
        request.board_angle_deg = 180
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=3)
        assert future.done() and not future.result().sent_unverified
        assert not select.select([master], [], [], 0.02)[0]
        assert not any(name == '/joint_states' for name, _ in arm.get_topic_names_and_types())
    finally:
        if executor is not None:
            executor.shutdown()
        if client_node is not None:
            client_node.destroy_node()
        if arm is not None:
            arm.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)


def test_ros_service_returns_failure_and_keeps_fault_latched(tmp_path):
    master, slave = os.openpty()
    profile = tmp_path / 'profile.yaml'
    profile.write_text(yaml.safe_dump({'calibration_verified': True, 'actuators': {
        name: {'channel': chr(65 + i), 'min_deg': 40, 'max_deg': 120}
        for i, name in enumerate(ACTUATORS)}}))
    rclpy.init(args=['--ros-args', '-p', f'profile:={profile}', '-p', f'port:={os.ttyname(slave)}'])
    arm = client_node = executor = None
    try:
        arm = Arm()
        os.close(master)
        client_node = rclpy.create_node('test_arm_fault_client')
        executor = SingleThreadedExecutor()
        executor.add_node(arm)
        executor.add_node(client_node)
        client = client_node.create_client(SetServoAngle, '/arm/set_servo_angle')
        assert client.wait_for_service(timeout_sec=3)

        request = SetServoAngle.Request(actuator='joint_1', board_angle_deg=90)
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=3)
        assert future.done() and not future.result().sent_unverified
        assert 'write failed' in future.result().message
        assert arm.transport.fault

        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=3)
        assert future.done() and not future.result().sent_unverified
        assert 'serial fault latched' in future.result().message
    finally:
        if executor is not None:
            executor.shutdown()
        if client_node is not None:
            client_node.destroy_node()
        if arm is not None:
            arm.destroy_node()
        rclpy.shutdown()
        try:
            os.close(master)
        except OSError:
            pass
        os.close(slave)
