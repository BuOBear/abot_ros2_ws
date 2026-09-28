"""Real DDS service/topic contract between authority and mission nodes."""

from concurrent.futures import Future
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'abot_navigation'))
from abot_navigation.velocity_authority import VelocityAuthority  # noqa: E402
from abot_mission.mission_node import MissionNode  # noqa: E402


ROUTE = '''
map_id: graph_test_map
frame_id: map
angle_unit: radians
steps:
  - {type: navigate, name: one, pose: {x: 0.0, y: 0.0, yaw: 0.0}, timeout_sec: 10.0}
'''


class FakeAction:
    def __init__(self):
        self.goals = []
        self.requests = []

    def send_goal_async(self, goal):
        future = Future()
        self.goals.append(goal)
        self.requests.append(future)
        return future


class FakeHandle:
    accepted = True

    def __init__(self):
        self.goal_id = SimpleNamespace(uuid=bytes([8] * 16))
        self.result = Future()
        self.cancel_reply = Future()
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_reply


def until(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_authority_lease_graph_prevents_stale_release_over_teleop(tmp_path):
    route = tmp_path / 'route.yaml'
    route.write_text(ROUTE)
    old_domain = os.environ.get('ROS_DOMAIN_ID')
    os.environ['ROS_DOMAIN_ID'] = '183'
    rclpy.init(args=['--ros-args', '-p', f'route_file:={route}',
                     '-p', 'map_id:=graph_test_map'])
    executor = MultiThreadedExecutor(num_threads=3)
    authority = mission = operator = None
    thread = None
    try:
        authority = VelocityAuthority()
        mission = MissionNode()
        mission._poll_lifecycle = lambda now: None
        mission._ready = lambda now: True
        action = FakeAction()
        mission.nav = action
        operator = Node('mission_graph_test_operator')
        start = operator.create_client(Trigger, '/mission/start')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        mode = operator.create_publisher(String, '/control/mode', qos)
        for node in (authority, mission, operator):
            executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        assert until(lambda: mission.last_mode == 'disabled' and mission.last_epoch == 0)
        assert until(lambda: start.service_is_ready() and mode.get_subscription_count() > 0)
        start_future = start.call_async(Trigger.Request())
        assert until(start_future.done) and start_future.result().success
        assert until(lambda: len(action.goals) == 1)
        assert mission.lease_confirmed and authority.authority.mode == 'nav'
        handle = FakeHandle()
        action.requests[0].set_result(handle)
        assert until(lambda: mission.goal_handle is handle)
        # Direct operator mode command revokes the lease and increments epoch.
        mode.publish(String(data='teleop'))
        assert until(lambda: authority.authority.mode == 'teleop')
        assert until(lambda: handle.cancel_calls == 1)
        assert until(lambda: mission.release_blocked)
        assert authority.authority.mode == 'teleop'
        handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
        assert until(lambda: mission.machine.phase == 'failed')
        assert len(action.goals) == 1
    finally:
        executor.shutdown(timeout_sec=2.0)
        if thread is not None:
            thread.join(timeout=2.0)
        for node in (operator, mission, authority):
            if node is not None:
                node.destroy_node()
        rclpy.shutdown()
        if old_domain is None:
            os.environ.pop('ROS_DOMAIN_ID', None)
        else:
            os.environ['ROS_DOMAIN_ID'] = old_domain
