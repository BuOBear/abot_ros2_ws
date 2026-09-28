"""Drive node callbacks with controllable Action and lease Service futures."""

from concurrent.futures import Future
from types import SimpleNamespace
import time

import pytest
import rclpy
from abot_control_interfaces.srv import AcquireNav, ReleaseNav
from action_msgs.msg import GoalStatus
from std_msgs.msg import String, UInt64
from std_srvs.srv import Trigger

from abot_mission.mission_node import MissionNode


ROUTE = '''
map_id: test_map
frame_id: map
angle_unit: radians
steps:
  - {type: navigate, name: first, pose: {x: 1.0, y: 2.0, yaw: 0.0}, timeout_sec: 5.0}
  - {type: navigate, name: second, pose: {x: 3.0, y: 4.0, yaw: 1.0}, timeout_sec: 5.0}
'''


class FakeService:
    def __init__(self):
        self.requests = []
        self.futures = []
        self.raise_call = False

    def service_is_ready(self):
        return True

    def call_async(self, request):
        if self.raise_call:
            raise RuntimeError('service transport failed')
        future = Future()
        self.requests.append(request)
        self.futures.append(future)
        return future


class FakeAction:
    def __init__(self):
        self.goals = []
        self.requests = []
        self.raise_send = False

    def send_goal_async(self, goal):
        if self.raise_send:
            raise RuntimeError('action transport failed')
        future = Future()
        self.goals.append(goal)
        self.requests.append(future)
        return future


class FakeGoalHandle:
    accepted = True

    def __init__(self, goal_number, raise_result=False, raise_cancel=False):
        self.goal_id = SimpleNamespace(uuid=bytes([goal_number] * 16))
        self.result = Future()
        self.cancel_reply = Future()
        self.cancel_calls = 0
        self.raise_result = raise_result
        self.raise_cancel = raise_cancel

    def get_result_async(self):
        if self.raise_result:
            raise RuntimeError('result transport failed')
        return self.result

    def cancel_goal_async(self):
        if self.raise_cancel:
            raise RuntimeError('cancel transport failed')
        self.cancel_calls += 1
        return self.cancel_reply


@pytest.fixture
def node(tmp_path):
    route = tmp_path / 'route.yaml'
    route.write_text(ROUTE)
    rclpy.init(args=['--ros-args', '-p', f'route_file:={route}', '-p', 'map_id:=test_map'])
    mission = MissionNode()
    action = FakeAction()
    acquire = FakeService()
    release = FakeService()
    mission.nav = action
    mission.acquire_client = acquire
    mission.release_client = release
    mission._poll_lifecycle = lambda now: None
    mission._ready = lambda now: True
    mission._on_mode_state(String(data='disabled'))
    mission._on_mode_epoch(UInt64(data=0))
    yield mission, action, acquire, release
    mission.destroy_node()
    rclpy.shutdown()


def acquire_lease(mission, action, acquire, lease_id=11, epoch=1, expect_goal=True):
    assert mission._start(None, Trigger.Response()).success
    mission._tick()
    assert len(acquire.requests) == 1 and not action.goals
    # Old latched nav/epoch data alone cannot send a goal.
    assert mission.last_mode == 'disabled' and mission.last_epoch == 0
    acquire.futures[-1].set_result(AcquireNav.Response(
        success=True, lease_id=lease_id, epoch=epoch, message='nav acquired'))
    assert not action.goals
    mission._on_mode_state(String(data='nav'))
    assert not action.goals
    mission._on_mode_epoch(UInt64(data=epoch))
    assert len(action.goals) == (1 if expect_goal else 0)


def test_bound_success_waits_for_release_before_next_goal(node):
    mission, action, acquire, release = node
    acquire_lease(mission, action, acquire)
    first = FakeGoalHandle(1)
    action.requests[0].set_result(first)
    first.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert mission.machine.phase == 'preparing' and mission.machine.index == 1
    assert release.requests[0].lease_id == 11
    # Authority publishes disabled and incremented epoch inside ReleaseNav,
    # before its Service response reaches this node.
    mission._on_mode_state(String(data='disabled'))
    mission._on_mode_epoch(UInt64(data=2))
    assert mission.machine.phase == 'preparing' and mission.machine.index == 1
    mission._tick()
    assert len(action.goals) == 1 and len(acquire.requests) == 1
    release.futures[0].set_result(ReleaseNav.Response(success=True, message='released'))
    mission._tick()
    assert len(acquire.requests) == 2
    acquire.futures[1].set_result(AcquireNav.Response(
        success=True, lease_id=12, epoch=3, message='nav acquired'))
    mission._on_mode_state(String(data='nav'))
    mission._on_mode_epoch(UInt64(data=3))
    assert len(action.goals) == 2
    action.requests[1].set_result(SimpleNamespace(accepted=False))
    assert mission.machine.phase == 'failed' and mission.machine.index == 1
    assert release.requests[-1].lease_id == 12


def test_release_response_before_state_echo_does_not_reacquire_early(node):
    mission, action, acquire, release = node
    acquire_lease(mission, action, acquire)
    first = FakeGoalHandle(7)
    action.requests[0].set_result(first)
    first.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    assert mission.machine.phase == 'preparing'

    # DDS delivery is independent across the Service response and the two
    # state topics. A successful reply alone cannot open the next step.
    release.futures[0].set_result(ReleaseNav.Response(success=True, message='released'))
    mission._tick()
    assert len(acquire.requests) == 1 and mission.lease_id == 11
    mission._on_mode_state(String(data='disabled'))
    mission._tick()
    assert len(acquire.requests) == 1 and mission.lease_id == 11
    mission._on_mode_epoch(UInt64(data=2))
    assert mission.lease_id is None
    mission._tick()
    assert len(acquire.requests) == 2


def test_external_mode_after_release_reply_blocks_next_acquire(node):
    mission, action, acquire, release = node
    acquire_lease(mission, action, acquire)
    first = FakeGoalHandle(8)
    action.requests[0].set_result(first)
    first.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
    release.futures[0].set_result(ReleaseNav.Response(success=True, message='released'))
    mission._on_mode_state(String(data='teleop'))
    mission._tick()
    assert mission.release_blocked and mission.machine.phase == 'failed'
    assert len(acquire.requests) == 1


def test_operator_cancel_checks_exact_goal_result(node):
    mission, action, acquire, release = node
    acquire_lease(mission, action, acquire)
    handle = FakeGoalHandle(2)
    action.requests[0].set_result(handle)
    assert mission._cancel(None, Trigger.Response()).success
    assert release.requests[0].lease_id == 11
    assert handle.cancel_calls == 1
    assert mission.machine.phase == 'canceling'
    assert not mission._start(None, Trigger.Response()).success
    handle.cancel_reply.set_result(SimpleNamespace(
        goals_canceling=[SimpleNamespace(goal_id=handle.goal_id)]))
    assert mission.machine.phase == 'canceling'
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert mission.machine.phase == 'canceled' and mission.machine.index == 0


def test_external_epoch_change_cancels_and_rejected_release_blocks(node):
    mission, action, acquire, release = node
    acquire_lease(mission, action, acquire)
    handle = FakeGoalHandle(3)
    action.requests[0].set_result(handle)
    # An external repeated nav request can leave mode_state=nav while revoking
    # this lease. The incremented epoch must cancel this specific goal.
    mission._on_mode_epoch(UInt64(data=2))
    assert handle.cancel_calls == 1 and release.requests[0].lease_id == 11
    release.futures[0].set_result(ReleaseNav.Response(
        success=False, message='nav lease is no longer current'))
    assert mission.release_blocked and not mission._start(None, Trigger.Response()).success
    handle.result.set_result(SimpleNamespace(status=GoalStatus.STATUS_CANCELED))
    assert mission.machine.phase == 'failed' and mission.machine.index == 0


def test_teleop_wins_before_acquire_reply(node):
    mission, action, acquire, release = node
    assert mission._start(None, Trigger.Response()).success
    mission._tick()
    # Authority has accepted teleop, but its state reply has not reached the
    # mission. Atomic AcquireNav rejects, so mission never claims control.
    acquire.futures[0].set_result(AcquireNav.Response(
        success=False, lease_id=0, epoch=1, message='control mode is teleop'))
    mission._on_mode_state(String(data='teleop'))
    assert mission.machine.phase == 'failed' and not action.goals
    assert not release.requests
    assert not hasattr(mission, 'mode_publisher')


def test_late_acquire_success_is_released_after_cancel_or_timeout(node):
    mission, action, acquire, release = node
    assert mission._start(None, Trigger.Response()).success
    mission._tick()
    assert mission._cancel(None, Trigger.Response()).success
    acquire.futures[0].set_result(AcquireNav.Response(
        success=True, lease_id=99, epoch=1, message='nav acquired'))
    assert not action.goals and release.requests[0].lease_id == 99
    assert not mission._start(None, Trigger.Response()).success
    release.futures[0].set_result(ReleaseNav.Response(success=True, message='released'))
    assert mission.lease_id == 99  # The Service reply is not the state barrier.
    mission._on_mode_state(String(data='disabled'))
    mission._on_mode_epoch(UInt64(data=2))
    assert mission.lease_id is None


def test_acquire_timeout_late_success_is_released(node):
    mission, action, acquire, release = node
    assert mission._start(None, Trigger.Response()).success
    mission._tick()
    mission.acquire_requested_at = time.monotonic() - 3.0
    mission._tick()
    assert mission.machine.phase == 'failed' and not action.goals
    acquire.futures[0].set_result(AcquireNav.Response(
        success=True, lease_id=44, epoch=1, message='late'))
    assert release.requests[0].lease_id == 44


def test_fast_epoch_change_before_goal_ack_cancels_lease(node):
    mission, action, acquire, release = node
    assert mission._start(None, Trigger.Response()).success
    mission._tick()
    acquire.futures[0].set_result(AcquireNav.Response(
        success=True, lease_id=45, epoch=1, message='granted'))
    mission._on_mode_state(String(data='nav'))
    mission._on_mode_epoch(UInt64(data=2))
    assert not action.goals
    assert release.requests[0].lease_id == 45
    assert mission.machine.phase == 'failed'


def test_preparing_timeout_when_nav2_not_ready(node):
    mission, action, acquire, release = node
    mission._ready = lambda now: False
    assert mission._start(None, Trigger.Response()).success
    mission.preparing_since = time.monotonic() - mission.preparing_timeout - 1.0
    mission._tick()
    assert mission.machine.phase == 'failed'
    assert mission.machine.detail == 'preparing_timeout'
    assert not acquire.requests and not action.goals and not release.requests


def test_transport_exceptions_keep_accepted_handle_reserved(node):
    mission, action, acquire, release = node
    acquire_lease(mission, action, acquire)
    handle = FakeGoalHandle(4, raise_result=True, raise_cancel=True)
    action.requests[0].set_result(handle)
    assert mission.machine.active_token is not None and mission.goal_handle is handle
    assert len(release.requests) == 1
    assert not mission._start(None, Trigger.Response()).success


def test_send_exception_releases_lease_and_fails(node):
    mission, action, acquire, release = node
    action.raise_send = True
    acquire_lease(mission, action, acquire, expect_goal=False)
    assert mission.machine.phase == 'failed'
    assert mission.machine.active_token is None
    assert release.requests[0].lease_id == 11
