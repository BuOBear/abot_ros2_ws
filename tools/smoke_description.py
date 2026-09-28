#!/usr/bin/env python3
"""Offline smoke test of the installed default abot bringup.

Run after sourcing ROS 2 Humble and this workspace's install/setup.bash:
    python3 tools/smoke_description.py

This launches the installed bringup in a private localhost ROS domain, reads
latched /tf_static directly with rclpy, then shuts down the whole launch group.
No serial device, lidar, camera, navigation or wheel feedback is needed.
"""

import argparse
import os
import pathlib
import random
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from collections import defaultdict


REQUIRED_FIXED_EDGES = {
    'base_link': 'base_footprint',
    'laser_link': 'base_link',
    'imu_link': 'base_link',
    'camera_link': 'base_link',
    'camera_optical_frame': 'camera_link',
}
WHEEL_CHILDREN = {'link_left_w', 'link_left_s', 'link_right_w', 'link_right_s'}
SMOKE_NODE = '_abot_description_smoke'


def stop_group(process):
    """Stop launch and all of its children even after a check fails."""
    if process.poll() is not None:
        return
    for sig, grace in ((signal.SIGINT, 4), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=grace)
            return
        except subprocess.TimeoutExpired:
            pass


def log_tail(stream, lines=30):
    stream.flush()
    stream.seek(0)
    return '\n'.join(stream.read().splitlines()[-lines:])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=20.0,
                        help='Seconds allowed for ROS discovery and checks (default: 20)')
    parser.add_argument('--domain-id', type=int,
                        help='ROS domain to isolate this run (default: random 100-232)')
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('--timeout must be positive')
    domain_id = args.domain_id if args.domain_id is not None else random.SystemRandom().randint(100, 232)
    if not 0 <= domain_id <= 232:
        parser.error('--domain-id must be between 0 and 232')

    if shutil.which('ros2') is None:
        raise RuntimeError('ros2 is not on PATH; source /opt/ros/humble/setup.bash')
    # Set the same isolated DDS environment before importing rclpy and before
    # spawning launch; either import may load an RMW implementation.
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.pop('ROS_AUTOMATIC_DISCOVERY_RANGE', None)
    os.environ.pop('ROS_STATIC_PEERS', None)
    print(f'Smoke domain: ROS_DOMAIN_ID={domain_id}, ROS_LOCALHOST_ONLY=1', flush=True)
    try:
        from ament_index_python.packages import get_package_share_directory
        import rclpy
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from tf2_msgs.msg import TFMessage
    except ImportError as exc:
        raise RuntimeError('ROS 2 Python packages unavailable; source Humble and the workspace install') from exc
    for package in ('abot_bringup', 'abot_description'):
        path = pathlib.Path(get_package_share_directory(package))
        if not path.is_dir():
            raise RuntimeError(f'{package} installed share not found')

    process = None
    node = None
    initialized = False
    with tempfile.TemporaryFile(mode='w+t', encoding='utf-8') as launch_log:
        try:
            process = subprocess.Popen(
                ['ros2', 'launch', 'abot_bringup', 'bringup.launch.py'],
                stdin=subprocess.DEVNULL, stdout=launch_log, stderr=subprocess.STDOUT,
                start_new_session=True, env=os.environ.copy(),
            )
            rclpy.init(domain_id=domain_id)
            initialized = True
            node = rclpy.create_node(SMOKE_NODE)
            edges = defaultdict(set)

            def on_static(message):
                for transform in message.transforms:
                    child = transform.child_frame_id
                    parent = transform.header.frame_id
                    edges[child].add(parent)

            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(TFMessage, '/tf_static', on_static, qos)
            client = node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
            future = None
            deadline = time.monotonic() + args.timeout
            stable_since = None
            last_nodes = set()
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f'bringup exited early with code {process.returncode}')
                rclpy.spin_once(node, timeout_sec=0.1)
                last_nodes = {f'{namespace.rstrip("/")}/{name}' for name, namespace
                              in node.get_node_names_and_namespaces()}
                if future is None and client.service_is_ready():
                    future = client.call_async(GetParameters.Request(names=['use_sim_time']))
                parameter_ok = False
                if future is not None and future.done():
                    response = future.result()
                    if response is None or len(response.values) != 1:
                        raise RuntimeError('robot_state_publisher did not return use_sim_time')
                    parameter = response.values[0]
                    if parameter.type != ParameterType.PARAMETER_BOOL or parameter.bool_value:
                        raise RuntimeError('default bringup use_sim_time is not false')
                    parameter_ok = True

                fixed_ok = all(edges.get(child) == {parent}
                               for child, parent in REQUIRED_FIXED_EDGES.items())
                expected_nodes = {'/robot_state_publisher', f'/{SMOKE_NODE}'}
                static_publishers = node.get_publishers_info_by_topic('/tf_static')
                ready = (fixed_ok and parameter_ok and last_nodes == expected_nodes
                         and len(static_publishers) == 1)
                if ready:
                    if stable_since is None:
                        stable_since = time.monotonic()
                    if time.monotonic() - stable_since >= 0.5:
                        break
                else:
                    stable_since = None
            else:
                raise RuntimeError(
                    f'timed out after {args.timeout:g}s; observed nodes={sorted(last_nodes)}, '
                    f'fixed edges={dict(edges)}, static_publishers='
                    f'{len(node.get_publishers_info_by_topic("/tf_static"))}, '
                    f'parameter_ready={future is not None and future.done()}'
                )

            for child, parent in REQUIRED_FIXED_EDGES.items():
                if edges[child] != {parent}:
                    raise RuntimeError(f'{child} has invalid parents: {edges[child]}')
            if len(node.get_publishers_info_by_topic('/tf_static')) != 1:
                raise RuntimeError('/tf_static must have exactly one publisher')
            if set(edges) != set(REQUIRED_FIXED_EDGES):
                raise RuntimeError(f'unexpected static frames: {sorted(set(edges) - set(REQUIRED_FIXED_EDGES))}')
            if WHEEL_CHILDREN & set(edges):
                raise RuntimeError('wheel frames were published as fixed transforms')

            print('PASS: required fixed TF tree; one publisher per child; use_sim_time=false', flush=True)
            print('PASS: only robot_state_publisher is running; no hardware or sensor nodes', flush=True)
            print('Wheel joints remain dynamic; no wheel feedback was fabricated.', flush=True)
            return 0
        except Exception as exc:
            print(f'FAIL: {exc}', file=sys.stderr)
            print('Launch output (last 30 lines):', file=sys.stderr)
            print(log_tail(launch_log), file=sys.stderr)
            return 1
        finally:
            if process is not None:
                stop_group(process)
            if node is not None:
                node.destroy_node()
            if initialized:
                rclpy.shutdown()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print(f'FAIL: {error}', file=sys.stderr)
        sys.exit(1)
