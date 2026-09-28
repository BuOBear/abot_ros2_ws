#!/usr/bin/env python3
"""Exercise installed P0 nodes with synthetic inputs; never opens physical devices.

Run after sourcing Humble, required dependencies and this workspace's install.
Uses a dedicated localhost DDS domain. This is software integration evidence only.
"""
import argparse
import faulthandler
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time


def process_start_time(pid):
    """Read Linux process birth time so a watchdog never signals a reused PID."""
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return fields[19] if fields[0] != 'Z' else None  # starttime is field 22
    except (OSError, IndexError):
        return None


def process_group_members(group_id):
    """Snapshot only live members of a process group created by this run."""
    members = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            if fields[0] != 'Z' and int(fields[2]) == group_id:
                members[int(entry.name)] = fields[19]
        except (OSError, IndexError, ValueError):
            continue
    return members


def write_json_atomic(path, value):
    temporary = path.with_name(path.name + f'.{os.getpid()}.tmp')
    temporary.write_text(json.dumps(value, indent=2) + '\n')
    temporary.replace(path)


def run_watchdog(payload):
    """Bound native ROS calls throughout the run and clean verified child groups."""
    parent_pid = payload['parent_pid']
    parent_start = payload['parent_start']
    registry_path = Path(payload['registry_path'])
    observed_path = Path(payload['observed_path'])
    owned = {}
    retired = set()
    last_observed = {}
    parent_exited = False
    phase = 'run'
    deadline = payload['initial_deadline_monotonic']

    def observe(registry):
        for child in registry['children']:
            pid = child['pid']
            if pid in retired:
                continue
            entry = owned.setdefault(pid, {'start': child['start'], 'name': child['name'],
                                           'members': {}})
            entry['members'].update({int(member_pid): start
                                     for member_pid, start in child['members'].items()})
            live_members = process_group_members(pid)
            if any(live_members.get(member_pid) == start
                   for member_pid, start in entry['members'].items()):
                # Add descendants while at least one previously verified member
                # still holds the original group ID. Never reacquire after loss.
                entry['members'].update(live_members)
            else:
                retired.add(pid)

    while True:
        if process_start_time(parent_pid) != parent_start:
            parent_exited = True
            break
        try:
            registry = json.loads(registry_path.read_text())
        except (OSError, json.JSONDecodeError):
            if time.monotonic() >= deadline:
                break
            time.sleep(0.25)
            continue
        phase = registry['phase']
        if phase == 'disarmed':
            return
        deadline = registry['deadline_monotonic']
        observe(registry)
        observed = {str(pid): entry['members'] for pid, entry in owned.items()}
        if observed != last_observed:
            write_json_atomic(observed_path, observed)
            last_observed = {pid: members.copy() for pid, members in observed.items()}
        if time.monotonic() >= deadline:
            break
        time.sleep(0.5)

    # Capture a group registered just before the parent exited or timed out.
    try:
        latest = json.loads(registry_path.read_text())
        if latest['phase'] == 'disarmed':
            return
        observe(latest)
        write_json_atomic(observed_path,
                          {str(pid): entry['members'] for pid, entry in owned.items()})
    except (OSError, json.JSONDecodeError):
        pass

    try:
        if json.loads(Path(payload['result_path']).read_text()).get('overall_pass') is True:
            return
    except (OSError, json.JSONDecodeError):
        pass

    # The parent may be stuck inside a native DDS destroy call, where Python
    # signal handlers and thread timeouts cannot make progress.
    parent_exited = parent_exited or process_start_time(parent_pid) != parent_start
    if not parent_exited:
        if process_start_time(parent_pid) != parent_start:
            parent_exited = True
        else:
            try:
                os.kill(parent_pid, signal.SIGKILL)
            except ProcessLookupError:
                parent_exited = True
    child_groups = []
    for pid, child in owned.items():
        if pid in retired:
            continue
        original_members = child['members']
        live_members = process_group_members(pid)
        if any(live_members.get(member_pid) == start
               for member_pid, start in original_members.items()):
            child_groups.append((pid, child, original_members))
            try:
                if process_start_time(pid) == child['start']:
                    os.kill(pid, signal.SIGINT)  # ros2 launch forwards this.
                else:
                    # The launch parent exited, but an original child still
                    # owns the group. Ask that group to exit directly.
                    current_members = process_group_members(pid)
                    if any(current_members.get(member_pid) == start
                           for member_pid, start in original_members.items()):
                        os.killpg(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    time.sleep(8.0)
    forced_groups = []
    for group_id, child, original_members in child_groups:
        # A numerical process group ID can be reused after every original
        # member exits. Never signal a group with no matching live member.
        live_members = process_group_members(group_id)
        if not any(live_members.get(pid) == start
                   for pid, start in original_members.items()):
            continue
        # Recheck the parent identity immediately before group SIGKILL. If it
        # is gone, matching original group members still prove group identity.
        parent_start = process_start_time(group_id)
        if parent_start is not None and parent_start != child['start']:
            continue
        try:
            os.killpg(group_id, signal.SIGKILL)
        except ProcessLookupError:
            continue
        forced_groups.append(child['name'])

    path = Path(payload['result_path'])
    try:
        results = json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        results = {}
    reason = ('Harness exited before watchdog was disarmed'
              if parent_exited else
              f'{phase} watchdog deadline exceeded; killed the stuck harness')
    if results.get('failure') in (None, 'Run did not complete'):
        results['failure'] = reason
    results.setdefault('cleanup_failure', []).append(reason)
    results['watchdog_triggered'] = True
    results['watchdog_phase'] = phase
    results['watchdog_forced_child_groups'] = forced_groups
    results['no forced process kills'] = False
    results['overall_pass'] = False
    write_json_atomic(path, results)


def sweep_verified_groups(registry, observed_path):
    """Report and clean children that outlived their registered launch parents."""
    observed = json.loads(observed_path.read_text())
    orphaned = []
    forced = []
    survivors = []
    unverified = []
    for child in registry['children']:
        group_id = child['pid']
        known = {int(pid): start for pid, start in child['members'].items()}
        known.update({int(pid): start for pid, start in
                      observed.get(str(group_id), {}).items()})

        def verified_live():
            live = process_group_members(group_id)
            if any(live.get(pid) == start for pid, start in known.items()):
                known.update(live)
                return live
            return {}

        if not verified_live():
            # A nonempty group whose original members were never observed is
            # ambiguous after PID reuse. Fail the run without signaling it.
            if process_group_members(group_id):
                unverified.append(child['name'])
            continue
        orphaned.append(child['name'])
        try:
            if process_start_time(group_id) == child['start']:
                os.kill(group_id, signal.SIGINT)
            elif verified_live():
                os.killpg(group_id, signal.SIGINT)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and verified_live():
            time.sleep(0.1)
        if verified_live():
            # Check the original member identity immediately before a
            # group-wide forced signal; a reused group ID is never enough.
            current_parent = process_start_time(group_id)
            if (current_parent is None or current_parent == child['start']) and verified_live():
                try:
                    os.killpg(group_id, signal.SIGKILL)
                    forced.append(child['name'])
                except ProcessLookupError:
                    pass
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline and verified_live():
                    time.sleep(0.1)
        if verified_live():
            survivors.append(child['name'])
    return orphaned, forced, survivors, unverified


def self_test_owned_sweep():
    """Reproduce a launch parent exiting while its child remains in the group."""
    with tempfile.TemporaryDirectory(prefix='p0-owned-sweep-') as directory:
        pid_path = Path(directory) / 'child.pid'
        observed_path = Path(directory) / 'observed.json'
        leader_code = (
            'import subprocess,sys,time; '
            'child=subprocess.Popen(["sleep","30"]); '
            'open(sys.argv[1],"w").write(str(child.pid)); time.sleep(0.3)'
        )
        leader = subprocess.Popen([sys.executable, '-c', leader_code, str(pid_path)],
                                  start_new_session=True)
        unrelated = subprocess.Popen(['sleep', '30'], start_new_session=True)
        child_pid = None
        child_start = None
        try:
            deadline = time.monotonic() + 3.0
            while not pid_path.exists() and time.monotonic() < deadline:
                time.sleep(0.02)
            child_pid = int(pid_path.read_text())
            members = process_group_members(leader.pid)
            child_start = members[child_pid]
            registry = {'children': [{'pid': leader.pid,
                                     'start': process_start_time(leader.pid),
                                     'members': members, 'name': 'orphan_test'}]}
            write_json_atomic(observed_path, {str(leader.pid): members})
            leader.wait(timeout=3)
            assert process_start_time(child_pid) == child_start
            orphaned, forced, survivors, unverified = sweep_verified_groups(
                registry, observed_path)
            assert orphaned == ['orphan_test'] and not forced and not survivors and not unverified
            assert process_start_time(child_pid) is None
            assert unrelated.poll() is None
            print('PASS: exited launch parent and surviving child fail the final sweep')
        finally:
            if leader.poll() is None:
                leader.kill()
            leader.wait(timeout=5)
            if child_pid is not None and process_start_time(child_pid) == child_start:
                os.kill(child_pid, signal.SIGKILL)
            if unrelated.poll() is None:
                unrelated.kill()
            unrelated.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=87)
    parser.add_argument('--bond-timeout', type=float, default=4.0,
                        help='Test-only Nav2 lifecycle bond timeout in seconds')
    parser.add_argument('--synthetic-async', action='store_true',
                        help='Use asynchronous Fast DDS publication only for this synthetic probe')
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).resolve().parents[1] / '.validation' / 'p0-smoke')
    args = parser.parse_args()
    faulthandler.register(signal.SIGUSR1)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    if args.synthetic_async:
        # WSL/TCP has blocked this probe's synthetic /wheel_odom publish().
        # Child ROS launches retain their normal production publication mode.
        os.environ['RMW_FASTRTPS_PUBLICATION_MODE'] = 'ASYNCHRONOUS'
    if not math.isfinite(args.bond_timeout) or args.bond_timeout <= 0:
        parser.error('--bond-timeout must be finite and positive')
    test_bond_timeout = str(args.bond_timeout)
    args.output.mkdir(parents=True, exist_ok=True)
    results = {'overall_pass': False, 'failure': 'Run did not complete'}
    results['synthetic asynchronous publication'] = args.synthetic_async
    result_path = args.output / 'results.json'
    result_path.write_text(json.dumps(results, indent=2) + '\n')
    registry_path = args.output / 'watchdog-registry.json'
    observed_path = args.output / 'watchdog-observed.json'
    registry = {'phase': 'run', 'deadline_monotonic': time.monotonic() + 600,
                'children': []}
    write_json_atomic(registry_path, registry)
    write_json_atomic(observed_path, {})
    watchdog_payload = {'parent_pid': os.getpid(),
                        'parent_start': process_start_time(os.getpid()),
                        'initial_deadline_monotonic': registry['deadline_monotonic'],
                        'registry_path': str(registry_path.resolve()),
                        'observed_path': str(observed_path.resolve()),
                        'result_path': str(result_path.resolve())}
    watchdog = subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), '--run-watchdog',
         json.dumps(watchdog_payload)],
        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, start_new_session=True)
    if watchdog.poll() is not None:
        raise RuntimeError('Run watchdog exited during startup')

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from sensor_msgs.msg import Imu, LaserScan
    from nav_msgs.msg import Odometry, OccupancyGrid
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.srv import SaveMap, ManageLifecycleNodes
    from lifecycle_msgs.srv import GetState
    from std_msgs.msg import String
    from action_msgs.msg import GoalStatus
    from tf2_ros import Buffer, TransformListener
    from ament_index_python.packages import get_package_prefix, get_package_share_directory

    rclpy.init()
    node = rclpy.create_node('p0_integration_probe')
    # Keep synthetic inputs running while the probe waits for services/actions.
    # A separate publisher-only node avoids sharing the probe's spin thread.
    sensor_node = rclpy.create_node('p0_synthetic_sensors')
    processes = []
    process_names = {}
    forced_kills = []
    shutdown_completed = set()
    handles = []
    last = {'odom': None, 'imu': None, 'cmd': None, 'map': None, 'cmd_at': 0.0,
            'nav_cmd': None, 'nav_cmd_at': 0.0}
    cmd_history = []
    tf_edges = {}
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    sensor_state = {'scans': True, 'scan_mode': 'room', 'blocked': False, 'vy': 0.0,
                    'nav': None, 'teleop': None, 'mode': None}
    sensor_lock = threading.Lock()
    current_stage = 'starting synthetic sensors'
    def set_sensor(**changes):
        with sensor_lock:
            sensor_state.update(changes)

    # robot_localization subscribes with SensorDataQoS. On this WSL/TCP test
    # transport a synthetic Reliable writer can block in publish() while
    # Nav2 discovers peers, starving every other input in this probe process.
    # The production hardware publisher remains Reliable.
    odom_pub = sensor_node.create_publisher(Odometry, '/wheel_odom', qos_profile_sensor_data)
    imu_pub = sensor_node.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
    scan_pub = sensor_node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    nav_pub = sensor_node.create_publisher(Twist, '/cmd_vel/nav', 1)
    teleop_pub = sensor_node.create_publisher(Twist, '/cmd_vel/teleop', 1)
    mode_pub = sensor_node.create_publisher(String, '/control/mode', 1)
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def store(name, msg):
        last[name] = msg
        if name in ('cmd', 'nav_cmd'):
            last[name + '_at'] = time.monotonic()
        if name == 'cmd':
            cmd_history.append((last['cmd_at'], msg))

    signal_probe = {}

    def record_command(topic, msg):
        sample = signal_probe.setdefault(topic, {'count': 0, 'max_abs_y': 0.0})
        sample['count'] += 1
        sample['last_y'] = msg.linear.y
        sample['max_abs_y'] = max(sample['max_abs_y'], abs(msg.linear.y))

    def record_filtered_scan(msg):
        sample = signal_probe.setdefault('/scan_filtered', {'count': 0})
        sample['count'] += 1
        sample['last_received_at'] = time.monotonic()
        sample['last_beams'] = len(msg.ranges)
        sample['last_stamp_age_s'] = round(
            node.get_clock().now().nanoseconds * 1e-9 -
            (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9), 3)

    # Humble rclpy omits publisher GIDs even from take_message metadata.
    # A small rclcpp observer records the actual DDS writer for each TF child.
    probe_source = Path(__file__).resolve().parent / 'tf_probe'
    probe_build = args.output / 'tf_probe_build'
    subprocess.run(['cmake', '-S', str(probe_source), '-B', str(probe_build)], check=True)
    subprocess.run(['cmake', '--build', str(probe_build), '--parallel', '1'], check=True)
    tf_observations = None

    def drain_tf():
        if tf_observations is not None:
            while True:
                offset = tf_observations.tell()
                line = tf_observations.readline()
                if not line:
                    break
                if not line.endswith('\n'):
                    # The probe writes concurrently; read this record again
                    # once its complete DDS writer ID has reached the file.
                    tf_observations.seek(offset)
                    break
                parts = line.split()
                if (len(parts) != 3 or len(parts[2]) != 48 or
                        any(char not in '0123456789abcdef' for char in parts[2])):
                    # DDS/ROS diagnostics can also appear in the probe log.
                    continue
                child, parent, gid = parts
                tf_edges.setdefault(child, set()).add((parent, gid))

    subscriptions = [
        node.create_subscription(Odometry, '/odom', lambda m: store('odom', m), 10),
        node.create_subscription(Imu, '/imu/data', lambda m: store('imu', m), qos_profile_sensor_data),
        node.create_subscription(Twist, '/cmd_vel', lambda m: store('cmd', m), 10),
        node.create_subscription(Twist, '/cmd_vel/nav', lambda m: store('nav_cmd', m), 10),
        node.create_subscription(OccupancyGrid, '/map', lambda m: store('map', m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(LaserScan, '/scan_filtered', record_filtered_scan,
                                 qos_profile_sensor_data),
        *[node.create_subscription(Twist, topic,
                                   lambda m, topic=topic: record_command(topic, m), 10)
          for topic in ('/cmd_vel/authorized/nav', '/cmd_vel/muxed',
                        '/cmd_vel/smoothed', '/cmd_vel/collision_checked')],
    ]

    sensor_report = {'error': None, 'iterations': 0, 'max_gap_s': 0.0,
                     'max_gap_stage': None, 'max_publish_s': 0.0,
                     'max_publish_topic': None, 'max_publish_stage': None,
                     'publishing_topic': None, 'publishing_stage': None,
                     'publishing_since': None}

    def timed_publish(topic, publisher, message):
        started = time.monotonic()
        publish_stage = current_stage
        sensor_report['publishing_topic'] = topic
        sensor_report['publishing_stage'] = publish_stage
        sensor_report['publishing_since'] = started
        try:
            publisher.publish(message)
        finally:
            duration = time.monotonic() - started
            if duration > sensor_report['max_publish_s']:
                sensor_report['max_publish_s'] = duration
                sensor_report['max_publish_topic'] = topic
                sensor_report['max_publish_stage'] = publish_stage
            sensor_report['publishing_topic'] = None
            sensor_report['publishing_stage'] = None
            sensor_report['publishing_since'] = None

    def sensors():
        with sensor_lock:
            state = sensor_state.copy()
            sensor_state['mode'] = None
        stamp = sensor_node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_footprint'
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.y = state['vy']
        for index in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[index] = 0.1
            odom.twist.covariance[index] = 0.01
        timed_publish('/wheel_odom', odom_pub, odom)
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, 'imu_link'
        imu.orientation_covariance[0] = -1.0
        imu.linear_acceleration.z = 9.80665
        for index in (0, 4, 8):
            imu.angular_velocity_covariance[index] = 0.01
            imu.linear_acceleration_covariance[index] = 0.01
        timed_publish('/imu/data_raw', imu_pub, imu)
        if state['scans']:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'laser_link'
            scan.angle_min, scan.angle_max = -math.pi, math.pi
            scan.angle_increment = 2 * math.pi / 359
            scan.range_min, scan.range_max, scan.scan_time = 0.05, 10.0, 0.1
            # A fixed 8 x 8 m square room; ample clearance around the origin.
            scan.ranges = [4.0 / max(abs(math.sin(-math.pi + i * scan.angle_increment)),
                                     abs(math.cos(-math.pi + i * scan.angle_increment))) for i in range(360)]
            if state['scan_mode'] == 'empty':
                scan.ranges = []
            elif state['scan_mode'] == 'nan':
                scan.ranges = [math.nan] * 360
            elif state['scan_mode'] == 'clear':
                scan.ranges = [math.inf] * 360
            elif state['scan_mode'] == 'sparse':
                scan.ranges = [math.nan] * 359 + [math.inf]
            elif state['scan_mode'] == 'blind_wedge':
                ranges = list(scan.ranges)
                ranges[240:280] = [math.nan] * 40
                scan.ranges = ranges
            elif state['scan_mode'] == 'near_blind':
                scan.range_min = 2.0
            elif state['scan_mode'] == 'far_blind':
                scan.range_max = 0.2
                scan.ranges = [math.inf] * 360
            if state['blocked'] and state['scan_mode'] == 'room':
                for index in range(264, 276):
                    scan.ranges[index] = 0.28  # Lateral obstacle outside the body box.
            timed_publish('/scan', scan_pub, scan)
        for key, pub in [('nav', nav_pub), ('teleop', teleop_pub)]:
            if state[key] is not None:
                msg = Twist()
                msg.linear.x, msg.linear.y = state[key]
                timed_publish('/cmd_vel/' + key, pub, msg)
        if state['mode'] is not None:
            timed_publish('/control/mode', mode_pub, String(data=state['mode']))

    sensor_stop = threading.Event()

    def publish_sensors():
        last_started = None
        while not sensor_stop.is_set():
            started = time.monotonic()
            if last_started is not None:
                gap = started - last_started
                if gap > sensor_report['max_gap_s']:
                    sensor_report['max_gap_s'] = gap
                    sensor_report['max_gap_stage'] = current_stage
            last_started = started
            try:
                sensors()
                sensor_report['iterations'] += 1
            except BaseException as exc:
                sensor_report['error'] = f'{type(exc).__name__}: {exc}'
                return
            sensor_stop.wait(max(0.0, 0.1 - (time.monotonic() - started)))

    sensor_thread = threading.Thread(target=publish_sensors, name='p0-synthetic-sensors', daemon=True)
    sensor_thread.start()

    def moving(msg):
        return msg is not None and max(abs(msg.linear.x), abs(msg.linear.y), abs(msg.angular.z)) > 0.01

    def zero(msg):
        if msg is None:
            return False
        components = (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z)
        return all(value == 0.0 for value in components)

    def sustained_zero(since, duration=0.75):
        now = time.monotonic()
        samples = [(stamp, msg) for stamp, msg in cmd_history if stamp > since]
        last_motion = max((stamp for stamp, msg in samples if not zero(msg)), default=since)
        zero_samples = [(stamp, msg) for stamp, msg in samples if stamp > last_motion]
        stamps = [stamp for stamp, _ in zero_samples]
        return (len(stamps) >= 5 and stamps[-1] - stamps[0] >= duration and
                now - stamps[-1] < 0.2 and
                all(right - left <= 0.25 for left, right in zip(stamps, stamps[1:])))

    def fresh_zero(since):
        msg = last['cmd']
        return last['cmd_at'] > since and zero(msg)

    def fault_stop(since, label, budget):
        until(lambda: fresh_zero(since), label, budget)
        first_zero = min(stamp for stamp, msg in cmd_history if stamp > since and zero(msg))
        latency = first_zero - since
        assert latency <= budget, f'{label} took {latency:.3f}s, budget {budget:.3f}s'
        results[label + ' latency_s'] = round(latency, 3)
        return first_zero

    def assert_zero_continuity(since, label):
        samples = [(stamp, msg) for stamp, msg in cmd_history if stamp >= since]
        assert len(samples) >= 5 and all(zero(msg) for _, msg in samples), label
        assert all(right[0] - left[0] <= 0.25
                   for left, right in zip(samples, samples[1:])), label + ' output gap'

    def check_tf(mode):
        expected = {'base_link': 'base_footprint', 'laser_link': 'base_link',
                    'imu_link': 'base_link', 'camera_link': 'base_link',
                    'camera_optical_frame': 'camera_link', 'base_footprint': 'odom', 'odom': 'map'}
        until(lambda: all(child in tf_edges for child in expected), mode + ' required TF edges')
        for child, parent in expected.items():
            assert len(tf_edges[child]) == 1, (child, tf_edges[child])
            assert next(iter(tf_edges[child]))[0] == parent
        until(lambda: buffer.can_transform('map', 'camera_optical_frame', rclpy.time.Time()),
              mode + ' complete TF chain')
        results[mode + ' unique TF publishers per child'] = True

    def start(name, command):
        log = (args.output / (name + '.log')).open('w')
        handles.append(log)
        child_env = os.environ.copy()
        if args.synthetic_async:
            child_env.pop('RMW_FASTRTPS_PUBLICATION_MODE', None)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True, env=child_env)
        processes.append(process)
        process_names[process.pid] = name
        registry['children'].append({
            'pid': process.pid, 'start': process_start_time(process.pid),
            'members': process_group_members(process.pid), 'name': name,
        })
        write_json_atomic(registry_path, registry)
        return process

    def stop(process):
        nonlocal current_stage
        if process.poll() is None:
            current_stage = 'stopping ' + process_names.get(process.pid, str(process.pid))
            child = next((entry for entry in registry['children']
                          if entry['pid'] == process.pid), None)
            if child is not None and process_start_time(process.pid) == child['start']:
                # Capture descendants while the original group leader still
                # proves ownership. They may outlive ros2 launch itself.
                child['members'].update(process_group_members(process.pid))
                write_json_atomic(registry_path, registry)
            # ros2 launch forwards SIGINT to its children. Signaling the whole
            # group here delivers it twice and can interrupt their cleanup.
            process.send_signal(signal.SIGINT)
            # Retain a group kill only as the final bounded cleanup fallback.
            deadline = time.monotonic() + 25.0
            while process.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                drain_tf()
            if process.poll() is None:
                forced_kills.append(process_names.get(process.pid, str(process.pid)))
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

    def shutdown_managers(names):
        nonlocal current_stage
        for name in names:
            if name in shutdown_completed:
                continue
            current_stage = 'shutting down ' + name
            client = node.create_client(ManageLifecycleNodes, '/' + name + '/manage_nodes')
            try:
                key = name + ' orderly shutdown'
                if client.wait_for_service(timeout_sec=1.0):
                    request = ManageLifecycleNodes.Request()
                    request.command = ManageLifecycleNodes.Request.SHUTDOWN
                    future = client.call_async(request)
                    # Nav2 can take more than 10 s to shut down all managed nodes.
                    # Require the manager's completed, successful response.
                    deadline = time.monotonic() + 45.0
                    while not future.done() and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    success = future.done() and future.result().success
                else:
                    success = False
                results[key] = results.get(key, True) and bool(success)
                if success:
                    shutdown_completed.add(name)
                print('SHUTDOWN:', name, bool(success), flush=True)
                # Keep the manager response if client destruction blocks next.
                result_path.write_text(json.dumps(results, indent=2) + '\n')
            finally:
                node.destroy_client(client)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            drain_tf()

    def until(predicate, label, timeout=30.0):
        nonlocal current_stage
        current_stage = label
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            drain_tf()
            if sensor_report['error'] is not None:
                raise RuntimeError('Synthetic sensor publisher failed: ' + sensor_report['error'])
            if sensor_report['max_gap_s'] > 0.5:
                raise RuntimeError('Synthetic sensor publisher exceeded 0.5 s scan timeout '
                                   f'during {sensor_report["max_gap_stage"]}: '
                                   f'{sensor_report["max_gap_s"]:.3f} s')
            if predicate():
                print('PASS:', label, flush=True)
                results[label] = True
                return
            failed = [p.returncode for p in processes if p.poll() is not None and p.returncode not in (0, -2)]
            if failed:
                raise RuntimeError(f'Launch exited {failed}; inspect {args.output}')
        raise RuntimeError(f'Timed out: {label}; inspect {args.output}')

    def call(client, request, label, timeout=20.0):
        until(lambda: client.service_is_ready(), label + ' available', timeout)
        future = client.call_async(request)
        until(future.done, label + ' returned', timeout)
        return future.result()

    def active(names):
        for name in names:
            client = node.create_client(GetState, '/' + name + '/get_state')
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline:
                if client.service_is_ready():
                    future = client.call_async(GetState.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
                    if future.done() and future.result().current_state.id == 3:
                        results[name + ' active'] = True
                        print('PASS:', name, 'active', flush=True)
                        break
                spin(0.25)
            else:
                raise RuntimeError(name + ' did not become active')
            node.destroy_client(client)

    try:
        start('tf_publishers', [str(probe_build / 'tf_publisher_probe')])
        tf_observations = (args.output / 'tf_publishers.log').open()
        start('laser_filter', [str(Path(get_package_prefix('laser_filters')) / 'lib' /
                                   'laser_filters' / 'scan_to_scan_filter_chain'),
                              '--ros-args', '--params-file', str(Path(
                                  get_package_share_directory('abot_bringup')) / 'config' / 'laser_filter.yaml')])
        start('base', ['ros2', 'launch', 'abot_bringup', 'bringup.launch.py',
                       'enable_state_estimation:=false', 'enable_velocity:=true',
                       'bond_timeout:=' + test_bond_timeout])
        estimation = start('state_estimation', ['ros2', 'launch', 'abot_navigation',
                                                'localization.launch.py'])
        until(lambda: last['odom'] is not None and last['imu'] is not None, 'EKF and IMU output')
        active(['velocity_smoother', 'collision_monitor'])
        set_sensor(vy=0.12)
        until(lambda: last['odom'].twist.twist.linear.y > 0.08, 'EKF preserves measured lateral velocity')
        set_sensor(vy=0.0)
        until(lambda: abs(last['odom'].twist.twist.linear.y) < 0.01, 'EKF returns to stationary')
        until(lambda: mode_pub.get_subscription_count() == 1, 'one velocity authority')
        set_sensor(mode='nav', nav=(0.0, 0.12))
        until(lambda: last['cmd'] is not None and last['cmd'].linear.y > 0.05, 'lateral velocity reaches final cmd_vel')
        stopped_at = time.monotonic()
        set_sensor(blocked=True)
        until(lambda: fresh_zero(stopped_at), 'lateral obstacle stops final velocity', 5.0)
        set_sensor(blocked=False)
        until(lambda: last['cmd'].linear.y > 0.05, 'cleared lateral obstacle allows selected source')
        set_sensor(mode='teleop', teleop=(0.10, 0.0))
        until(lambda: last['cmd'].linear.x > 0.05, 'teleop owns final velocity')
        stopped_at = time.monotonic()
        set_sensor(teleop=None)
        until(lambda: fresh_zero(stopped_at), 'teleop timeout produces fresh zero', 3.0)
        until(lambda: sustained_zero(stopped_at),
              'teleop timeout settles to sustained zero', 3.0)
        settled_at = time.monotonic()
        spin(0.5)
        assert all(zero(msg) for stamp, msg in cmd_history if stamp > settled_at), (
            'nonzero velocity after teleop timeout settled',
            [(stamp - settled_at, msg.linear.x, msg.linear.y, msg.angular.z)
             for stamp, msg in cmd_history if stamp > settled_at and not zero(msg)])
        results['teleop timeout holds zero while nav continues'] = True
        set_sensor(mode='nav')
        until(lambda: last['cmd'].linear.y > 0.05, 'explicit nav regrant')
        stopped_at = time.monotonic()
        set_sensor(scan_mode='empty')
        empty_zero = fault_stop(stopped_at, 'empty scan stops final velocity', 0.75)
        until(lambda: sustained_zero(stopped_at), 'empty scan holds final zero', 5.0)
        assert_zero_continuity(empty_zero, 'empty scan')
        set_sensor(scan_mode='clear')
        until(lambda: last['cmd'].linear.y > 0.05,
              'all-NaN scan precondition motion', 5.0)
        stopped_at = time.monotonic()
        set_sensor(scan_mode='nan')
        nan_zero = fault_stop(stopped_at, 'all-NaN scan holds final zero', 0.75)
        until(lambda: sustained_zero(stopped_at), 'all-NaN scan sustained zero', 5.0)
        assert_zero_continuity(nan_zero, 'all-NaN scan')
        for scan_mode in ('sparse', 'blind_wedge', 'near_blind', 'far_blind'):
            set_sensor(scan_mode='clear')
            until(lambda: last['cmd'].linear.y > 0.05,
                  scan_mode + ' precondition motion', 5.0)
            stopped_at = time.monotonic()
            set_sensor(scan_mode=scan_mode)
            first_zero = fault_stop(stopped_at, scan_mode + ' stops final velocity', 0.75)
            until(lambda: sustained_zero(stopped_at), scan_mode + ' sustained zero', 5.0)
            assert_zero_continuity(first_zero, scan_mode)
        set_sensor(scan_mode='clear')
        until(lambda: last['cmd'].linear.y > 0.05,
              'positive-infinity clear scan allows motion', 5.0)
        set_sensor(scan_mode='room')
        stopped_at = time.monotonic()
        set_sensor(scans=False)
        # A transient zero while the last scan is still fresh is not evidence
        # of a stale-scan stop. Wait for the most recently received filtered
        # scan to exceed the production 0.5 s freshness limit.
        until(lambda: (time.monotonic() - signal_probe['/scan_filtered']['last_received_at']
                       >= 0.5), 'filtered scan actually stale', 2.0)
        scan_expired_at = signal_probe['/scan_filtered']['last_received_at'] + 0.5
        stale_zero = fault_stop(scan_expired_at, 'stale scan stops final velocity', 0.5)
        assert stale_zero - stopped_at <= 1.0, 'stale scan total stop latency'
        results['stale scan silence to zero latency_s'] = round(stale_zero - stopped_at, 3)
        until(lambda: sustained_zero(scan_expired_at), 'stale scan sustained zero', timeout=5)
        assert_zero_continuity(stale_zero, 'stale scan')
        set_sensor(scans=True, mode='disabled', nav=None)
        spin(1.0)
        publishers = node.get_publishers_info_by_topic('/cmd_vel')
        assert len(publishers) == 1 and publishers[0].node_name == 'velocity_gate', publishers
        results['only velocity_gate publishes cmd_vel'] = True

        mapping = start('mapping', ['ros2', 'launch', 'abot_navigation', 'navigation.launch.py',
                                    'mode:=mapping', 'bond_timeout:=' + test_bond_timeout])
        until(lambda: last['map'] is not None and len(last['map'].data) > 0, 'SLAM publishes synthetic map', 45)
        nav_nodes = ['controller_server', 'planner_server', 'smoother_server',
                     'behavior_server', 'bt_navigator', 'waypoint_follower']
        active(nav_nodes + ['map_saver'])
        check_tf('mapping')
        save = node.create_client(SaveMap, '/map_saver/save_map')
        request = SaveMap.Request()
        request.map_topic = '/map'
        request.map_url = str(args.output / 'synthetic_map')
        request.image_format, request.map_mode = 'pgm', 'trinary'
        request.free_thresh, request.occupied_thresh = 0.25, 0.65
        assert call(save, request, 'save synthetic map').result
        results['map saved'] = True
        shutdown_managers(['lifecycle_manager_navigation', 'lifecycle_manager_mapping'])
        assert {'lifecycle_manager_navigation', 'lifecycle_manager_mapping'} <= shutdown_completed
        stop(mapping)
        processes.remove(mapping)
        spin(2.0)
        tf_edges.pop('odom', None)  # New map->odom owner for the mutually exclusive mode.
        last['map'] = None
        start('localization', ['ros2', 'launch', 'abot_navigation', 'navigation.launch.py',
                              'mode:=localization', 'map:=' + str(args.output / 'synthetic_map.yaml'),
                              'bond_timeout:=' + test_bond_timeout])
        active(['map_server', 'amcl'])
        until(lambda: last['map'] is not None, 'saved map reloaded')
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[0] = pose.pose.covariance[7] = 0.1
        pose.pose.covariance[35] = 0.05
        for _ in range(5):
            pose.header.stamp = node.get_clock().now().to_msg()
            initial_pub.publish(pose)
            spin(0.5)
        active(nav_nodes + ['velocity_smoother', 'collision_monitor'])
        check_tf('localization')
        navigator = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        until(navigator.server_is_ready, 'NavigateToPose action available')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = 1.0
        goal.pose.pose.orientation.w = 1.0
        set_sensor(mode='nav')
        goal_started_at = time.monotonic()
        sent = navigator.send_goal_async(goal)
        until(sent.done, 'navigation goal response')
        handle = sent.result()
        assert handle.accepted, 'NavigateToPose goal was rejected'
        results['navigation goal accepted'] = True
        until(lambda: last['nav_cmd_at'] > goal_started_at and moving(last['nav_cmd']) and
              last['cmd_at'] > goal_started_at and moving(last['cmd']),
              'real Nav2 velocity traverses final command chain', 15.0)
        canceled_at = time.monotonic()
        canceled = handle.cancel_goal_async()
        until(canceled.done, 'navigation cancel response')
        assert canceled.result().return_code == 0
        assert any(bytes(info.goal_id.uuid) == bytes(handle.goal_id.uuid)
                   for info in canceled.result().goals_canceling)
        outcome = handle.get_result_async()
        until(outcome.done, 'navigation goal terminal result')
        assert outcome.result().status == GoalStatus.STATUS_CANCELED
        until(lambda: fresh_zero(canceled_at), 'canceled Nav2 goal stops final command', 3.0)
        until(lambda: sustained_zero(canceled_at), 'canceled goal sustained exact zero', 5.0)
        results['specific navigation goal canceled'] = True
        results['canceled goal holds final zero'] = True
        set_sensor(mode='teleop', teleop=(0.10, 0.0))
        until(lambda: last['cmd'].linear.x > 0.05, 'teleop before odometry TF loss')
        stopped_at = time.monotonic()
        stop(estimation)
        processes.remove(estimation)
        def odometry_tf_is_stale():
            transform = buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            return node.get_clock().now().nanoseconds * 1e-9 - stamp > 0.5

        until(odometry_tf_is_stale, 'odometry TF actually stale while scans and teleop continue', 5.0)
        stopped_at = time.monotonic()
        tf_zero = fault_stop(stopped_at, 'missing odometry TF stops fresh-scan teleop', 0.75)
        until(lambda: sustained_zero(stopped_at, duration=1.5),
              'stale TF sustained exact zero past monitor timeout', 5.0)
        assert_zero_continuity(tf_zero, 'stale TF')
        results['stale odometry TF holds final zero with fresh scans and teleop'] = True
        results['failure'] = None
    except BaseException as exc:
        results['failure'] = f'{type(exc).__name__}: {exc}'
        results['signal_probe'] = signal_probe
        results['tf_children_observed'] = sorted(tf_edges)
        results['last_cmd_y'] = last['cmd'].linear.y if last['cmd'] is not None else None
        results['last_nav_y'] = last['nav_cmd'].linear.y if last['nav_cmd'] is not None else None
        # Preserve the actual failure if DDS teardown itself hangs or the run
        # is interrupted before the final result can be written.
        result_path.write_text(json.dumps(results, indent=2) + '\n')
        raise
    finally:
        cleanup_errors = []
        try:
            # Renew the deadline for the bounded cleanup phase. The functional
            # phase has already had its own 600 s limit since run startup.
            registry['phase'] = 'cleanup'
            registry['deadline_monotonic'] = time.monotonic() + 420
            write_json_atomic(registry_path, registry)
        except BaseException as exc:
            cleanup_errors.append(f'Cleanup watchdog could not be armed: {type(exc).__name__}: {exc}')

        def cleanup(action):
            try:
                action()
            except BaseException as exc:
                cleanup_errors.append(f'{type(exc).__name__}: {exc}')

        set_sensor(mode='disabled', nav=None, teleop=None, scan_mode='room')
        cleanup(lambda: spin(0.2))  # Let the authority receive disabled.
        sensor_stop.set()  # Stop test publishers before tearing down DDS readers.
        sensor_thread.join(timeout=5)
        if sensor_thread.is_alive():
            cleanup_errors.append('Synthetic sensor publisher did not stop')
        results['synthetic sensor publish iterations'] = sensor_report['iterations']
        results['synthetic sensor max publish gap_s'] = round(sensor_report['max_gap_s'], 3)
        results['synthetic sensor max publish gap stage'] = sensor_report['max_gap_stage']
        results['synthetic sensor max publish call_s'] = round(sensor_report['max_publish_s'], 3)
        results['synthetic sensor max publish call topic'] = sensor_report['max_publish_topic']
        results['synthetic sensor max publish call stage'] = sensor_report['max_publish_stage']
        results['synthetic sensor in-flight publish topic'] = sensor_report['publishing_topic']
        results['synthetic sensor in-flight publish stage'] = sensor_report['publishing_stage']
        results['synthetic sensor in-flight publish elapsed_s'] = (
            round(time.monotonic() - sensor_report['publishing_since'], 3)
            if sensor_report['publishing_since'] is not None else None)
        # The safety chain uses a 0.5 s scan timeout. A longer gap means the
        # synthetic stream cannot validate normal continuous-input behavior.
        cadence_stable = sensor_report['max_gap_s'] <= 0.5
        results['synthetic sensor cadence within scan timeout'] = cadence_stable
        if not cadence_stable:
            cleanup_errors.append('Synthetic sensor publisher exceeded 0.5 s scan timeout')
        if sensor_report['error'] is not None:
            cleanup_errors.append('Synthetic sensor publisher failed: ' + sensor_report['error'])
        # On an already failed run, service shutdown may block in DDS client
        # teardown; stop the launch parents directly instead.
        if results['failure'] is None and not cleanup_errors:
            for manager in ['lifecycle_manager_navigation', 'lifecycle_manager_localization',
                            'lifecycle_manager_mapping', 'lifecycle_manager_velocity']:
                cleanup(lambda manager=manager: shutdown_managers([manager]))
        for process in reversed(processes):
            cleanup(lambda process=process: stop(process))
        try:
            orphaned, forced_orphans, orphan_survivors, unverified_groups = sweep_verified_groups(
                registry, observed_path)
            results['no orphaned child process groups'] = not orphaned and not unverified_groups
            results['orphaned child process groups'] = orphaned
            results['orphaned child groups surviving cleanup'] = orphan_survivors
            results['unverified live process groups'] = unverified_groups
            forced_kills.extend('orphan group ' + name for name in forced_orphans)
            if orphaned:
                cleanup_errors.append('Launch children survived their parent in: ' +
                                      ', '.join(orphaned))
            if orphan_survivors:
                cleanup_errors.append('Launch children survived bounded cleanup in: ' +
                                      ', '.join(orphan_survivors))
            if unverified_groups:
                cleanup_errors.append('Registered process groups remain live without a verified '
                                      'member; left untouched: ' + ', '.join(unverified_groups))
        except BaseException as exc:
            results['no orphaned child process groups'] = False
            cleanup_errors.append(f'Owned child sweep failed: {type(exc).__name__}: {exc}')
        if not sensor_thread.is_alive():
            cleanup(sensor_node.destroy_node)
        cleanup(node.destroy_node)
        cleanup(rclpy.shutdown)
        if tf_observations is not None:
            cleanup(tf_observations.close)
        for log in handles:
            cleanup(log.close)
        # A manager can reset and recover between the active() snapshots.
        # Such a run is not stable acceptance evidence even if later checks pass.
        bond_failures = []
        launch_child_kills = []
        for name in ('base', 'mapping', 'localization'):
            path = args.output / (name + '.log')
            if not path.exists():
                continue
            log_text = path.read_text()
            if ('CRITICAL FAILURE: SERVER ' in log_text or
                    'Have not received a heartbeat' in log_text):
                bond_failures.append(name)
            if "escalating to 'SIGKILL'" in log_text:
                launch_child_kills.append(name)
        results['no lifecycle bond failures'] = not bond_failures
        if bond_failures:
            cleanup_errors.append('Lifecycle bond failure in: ' + ', '.join(bond_failures))
        results['no launch child SIGKILL'] = not launch_child_kills
        if launch_child_kills:
            cleanup_errors.append('Launch force-killed children in: ' +
                                  ', '.join(launch_child_kills))
        results['no forced process kills'] = not forced_kills
        if forced_kills:
            cleanup_errors.append('Forced process kills: ' + ', '.join(forced_kills))
        if any(value is False for key, value in results.items()
               if key.endswith(' orderly shutdown')):
            cleanup_errors.append('A lifecycle manager failed orderly shutdown')
        if watchdog.poll() is not None:
            cleanup_errors.append('Run watchdog exited before teardown completed')
        else:
            # Stop it before publishing the final result, so it cannot
            # overwrite a completed successful run at its deadline.
            try:
                registry['phase'] = 'disarmed'
                write_json_atomic(registry_path, registry)
                watchdog.wait(timeout=5)
            except subprocess.TimeoutExpired:
                watchdog.terminate()
                try:
                    watchdog.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    watchdog.kill()
                    watchdog.wait(timeout=5)
                    cleanup_errors.append('Run watchdog required SIGKILL to stop')
            except BaseException as exc:
                cleanup_errors.append(f'Run watchdog could not be disarmed: {type(exc).__name__}: {exc}')
                watchdog.terminate()
                watchdog.wait(timeout=5)
        if cleanup_errors:
            results['cleanup_failure'] = cleanup_errors
            if results['failure'] is None:
                results['failure'] = '; '.join(cleanup_errors)
        results['overall_pass'] = results['failure'] is None
        result_path.write_text(json.dumps(results, indent=2) + '\n')
        print(json.dumps(results, indent=2), flush=True)
        if cleanup_errors:
            raise RuntimeError('; '.join(cleanup_errors))



if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == '--run-watchdog':
        run_watchdog(json.loads(sys.argv[2]))
    elif len(sys.argv) == 2 and sys.argv[1] == '--self-test-owned-sweep':
        self_test_owned_sweep()
    else:
        main()
