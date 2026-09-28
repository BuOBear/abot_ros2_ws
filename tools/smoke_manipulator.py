#!/usr/bin/env python3
"""Inspect the installed, device-free real robot launch in an isolated ROS domain.

Source the ROS and unified ABOT workspace and select an unused ROS_DOMAIN_ID before running.
No hardware, camera, detector, or arm option is enabled by this probe.
"""

import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage


def observe(mode, timeout, log_file, bond_timeout):
    command = ['ros2', 'launch', 'jy_real_robot', 'real.launch.py']
    if mode == 'mapping':
        command += [
            'enable_state_estimation:=true', 'enable_velocity:=true', 'mode:=mapping',
            f'bond_timeout:={bond_timeout}',
        ]
    elif mode != 'default':
        raise ValueError(mode)

    child_frames = set()
    dynamic_edges = set()
    static_publishers = set()
    started = time.monotonic()
    sensor_stop = threading.Event()
    sensor_thread = None
    sensor_node = None
    sensor_error = []
    sensor_count = [0]
    with open(log_file, 'w', encoding='utf-8') as log:
        # Initialize the observer first so an invalid RMW/log directory cannot
        # leave an unobserved ROS launch running.
        rclpy.init()
        observer = rclpy.create_node('jy_install_observer')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        if mode == 'mapping':
            # Only this isolated-domain test publishes synthetic inputs. The
            # production launch has no fake odom/IMU/scan fallback.
            sensor_node = rclpy.create_node('jy_synthetic_inputs')
            odom_pub = sensor_node.create_publisher(Odometry, '/wheel_odom',
                                                    qos_profile_sensor_data)
            imu_pub = sensor_node.create_publisher(Imu, '/imu/data_raw',
                                                   qos_profile_sensor_data)
            scan_pub = sensor_node.create_publisher(LaserScan, '/scan_filtered',
                                                    qos_profile_sensor_data)

            def publish_sensors():
                try:
                    while not sensor_stop.is_set():
                        stamp = sensor_node.get_clock().now().to_msg()
                        odom = Odometry()
                        odom.header.stamp = stamp
                        odom.header.frame_id = 'odom'
                        odom.child_frame_id = 'base_footprint'
                        odom.pose.pose.orientation.w = 1.0
                        for index in (0, 7, 14, 21, 28, 35):
                            odom.pose.covariance[index] = 0.1
                            odom.twist.covariance[index] = 0.01
                        odom_pub.publish(odom)

                        imu = Imu()
                        imu.header.stamp = stamp
                        imu.header.frame_id = 'imu_link'
                        imu.orientation_covariance[0] = -1.0
                        imu.linear_acceleration.z = 9.80665
                        for index in (0, 4, 8):
                            imu.angular_velocity_covariance[index] = 0.01
                            imu.linear_acceleration_covariance[index] = 0.01
                        imu_pub.publish(imu)

                        scan = LaserScan()
                        scan.header.stamp = stamp
                        scan.header.frame_id = 'laser_link'
                        scan.angle_min, scan.angle_max = -math.pi, math.pi
                        scan.angle_increment = 2 * math.pi / 359
                        scan.range_min, scan.range_max, scan.scan_time = 0.05, 10.0, 0.1
                        scan.ranges = [
                            4.0 / max(abs(math.sin(-math.pi + i * scan.angle_increment)),
                                      abs(math.cos(-math.pi + i * scan.angle_increment)))
                            for i in range(360)
                        ]
                        scan_pub.publish(scan)
                        sensor_count[0] += 1
                        sensor_stop.wait(0.1)
                except Exception as exc:
                    sensor_error.append(str(exc))

            sensor_thread = threading.Thread(target=publish_sensors, daemon=True)
            sensor_thread.start()
        qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)

        def on_tf(message):
            child_frames.update(item.child_frame_id for item in message.transforms)

        observer.create_subscription(TFMessage, '/tf_static', on_tf, qos)
        observer.create_subscription(
            TFMessage, '/tf',
            lambda message: dynamic_edges.update(
                (item.header.frame_id, item.child_frame_id)
                for item in message.transforms), 100)
        report = None
        try:
            required_nodes = {'robot_state_publisher'}
            if mode == 'mapping':
                required_nodes |= {'velocity_gate', 'velocity_authority', 'twist_mux',
                                   'ekf_filter_node', 'slam_toolbox'}
            while time.monotonic() - started < timeout:
                if process.poll() is not None:
                    raise RuntimeError(f'launch exited early: {process.returncode}')
                if sensor_error:
                    raise RuntimeError('synthetic publisher failed: ' + sensor_error[0])
                rclpy.spin_once(observer, timeout_sec=0.2)
                nodes = set(observer.get_node_names()) - {'jy_install_observer'}
                static_publishers = {
                    item.node_name for item in observer.get_publishers_info_by_topic('/tf_static')
                }
                final_publishers = {
                    item.node_name for item in observer.get_publishers_info_by_topic('/cmd_vel')
                }
                if (required_nodes <= nodes and {'base_link', 'imu_link', 'laser_link'} <= child_frames
                        and static_publishers == {'robot_state_publisher'}
                        and final_publishers == ({'velocity_gate'} if mode == 'mapping' else set())):
                    break
            else:
                raise RuntimeError('required installed graph/TF did not become ready')

            if {'camera_link', 'camera_optical_frame', 'wrist_camera_optical_frame'} & child_frames:
                raise AssertionError('unmeasured camera TF was published')
            forbidden_topics = ('/joint_states', '/camera/image_raw', '/scan')
            for topic in forbidden_topics:
                if observer.get_publishers_info_by_topic(topic):
                    raise AssertionError(f'unrequested hardware/state publisher on {topic}')
            if any(name == '/arm/set_servo_angle'
                   for name, _ in observer.get_service_names_and_types()):
                raise AssertionError('arm service started without enable_arm')
            active = []
            shutdown = []
            if mode == 'mapping':
                # A graph appearance is earlier than Nav2 lifecycle readiness. Wait for
                # activation before stopping the launch, then ask managers to shut down.
                for name in ('velocity_smoother', 'collision_monitor', 'map_saver',
                             'controller_server', 'planner_server', 'smoother_server',
                             'behavior_server', 'bt_navigator', 'waypoint_follower'):
                    client = observer.create_client(GetState, f'/{name}/get_state')
                    try:
                        deadline = time.monotonic() + timeout
                        while time.monotonic() < deadline:
                            if process.poll() is not None:
                                raise RuntimeError(f'launch exited while activating {name}')
                            if client.service_is_ready():
                                future = client.call_async(GetState.Request())
                                rclpy.spin_until_future_complete(observer, future, timeout_sec=1.0)
                                if future.done() and future.result().current_state.id == 3:
                                    active.append(name)
                                    break
                            rclpy.spin_once(observer, timeout_sec=0.1)
                        else:
                            raise RuntimeError(f'{name} did not become active')
                    finally:
                        observer.destroy_client(client)
                deadline = time.monotonic() + timeout
                expected = {('odom', 'base_footprint'), ('map', 'odom')}
                while not expected <= dynamic_edges and time.monotonic() < deadline:
                    rclpy.spin_once(observer, timeout_sec=0.1)
                if not expected <= dynamic_edges:
                    raise RuntimeError('synthetic inputs did not produce map/odom TF chain')
                for name in ('lifecycle_manager_navigation', 'lifecycle_manager_mapping',
                             'lifecycle_manager_velocity'):
                    client = observer.create_client(ManageLifecycleNodes,
                                                    f'/{name}/manage_nodes')
                    try:
                        if not client.wait_for_service(timeout_sec=3.0):
                            raise RuntimeError(f'{name} management service unavailable')
                        request = ManageLifecycleNodes.Request()
                        request.command = ManageLifecycleNodes.Request.SHUTDOWN
                        future = client.call_async(request)
                        rclpy.spin_until_future_complete(observer, future, timeout_sec=45.0)
                        if not future.done() or not future.result().success:
                            raise RuntimeError(f'{name} did not shut down cleanly')
                        shutdown.append(name)
                    finally:
                        observer.destroy_client(client)
            report = {
                'mode': mode, 'passed': True, 'nodes': sorted(nodes),
                'static_tf_children': sorted(child_frames),
                'dynamic_tf_edges': sorted([list(edge) for edge in dynamic_edges]),
                'static_tf_publishers': sorted(static_publishers),
                'final_cmd_vel_publishers': sorted(final_publishers),
                'hardware_and_arm_disabled': True,
                'active_lifecycle_nodes': active,
                'orderly_lifecycle_shutdown': shutdown,
                'synthetic_input_cycles': sensor_count[0],
            }
            return report
        finally:
            sensor_stop.set()
            if sensor_thread is not None:
                sensor_thread.join(timeout=3)
            if sensor_node is not None:
                sensor_node.destroy_node()
            observer.destroy_node()
            rclpy.shutdown()
            if process.poll() is None:
                # ros2 launch forwards SIGINT to children. Signal its parent only.
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
                raise RuntimeError('launch required forced process-group cleanup')
            if report is not None:
                report['launch_returncode'] = process.returncode
                if process.returncode != 0 or sensor_error or (sensor_thread is not None
                                                               and sensor_thread.is_alive()):
                    report['passed'] = False
                    report['error'] = 'launch or synthetic publisher did not stop cleanly'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('default', 'mapping'), default='default')
    parser.add_argument('--timeout', type=float, default=25.0)
    parser.add_argument('--bond-timeout', type=float, default=12.0,
                        help='test-only WSL override; production launch defaults to 4 seconds')
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args()
    if not os.getenv('ROS_DOMAIN_ID'):
        parser.error('set an isolated ROS_DOMAIN_ID')
    if os.getenv('ROS_LOCALHOST_ONLY') != '1':
        parser.error('set ROS_LOCALHOST_ONLY=1 to isolate synthetic test inputs')
    options.output.mkdir(parents=True, exist_ok=True)
    ros_home = options.output / 'ros-home'
    ros_home.mkdir(exist_ok=True)
    os.environ['ROS_HOME'] = str(ros_home.resolve())
    os.environ['ROS_LOG_DIR'] = str((ros_home / 'log').resolve())
    try:
        result = observe(options.mode, options.timeout, options.output / 'launch.log',
                         options.bond_timeout)
    except Exception as exc:
        result = {'mode': options.mode, 'passed': False, 'error': str(exc)}
    if result['passed']:
        log = (options.output / 'launch.log').read_text(errors='replace')
        if re.search(r'\[ERROR\]|Traceback|SIGKILL|process has died', log):
            result['passed'] = False
            result['error'] = 'launch log contains an error, traceback or forced exit'
    (options.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
