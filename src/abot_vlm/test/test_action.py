import time
from threading import Event, Thread

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from abot_vlm_interfaces.action import AnalyzeImage
from abot_vlm.vlm_action import VlmAction


def await_future(future, timeout=3):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert future.done(), 'ROS future did not complete'
    return future.result()


def frame():
    image = Image()
    image.width = 4
    image.height = 3
    image.encoding = 'rgb8'
    image.step = 12
    image.data = bytes([100] * 36)
    return image


@pytest.fixture
def action_pair():
    rclpy.init()
    class FakeBackend:
        enabled = True
        started = Event()
        release = Event()
        blocking = False
        fail = False

        def available(self):
            return self.enabled

        def analyze(self, prompt, jpeg, timeout_sec):
            self.started.set()
            if self.blocking:
                assert self.release.wait(2)
            if self.fail:
                raise RuntimeError('fake backend failed')
            return 'reply:' + prompt

    backend = FakeBackend()
    server = VlmAction(backend=backend)
    client_node = Node('vlm_test_client')
    client = ActionClient(client_node, AnalyzeImage, '/vlm/analyze')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client_node)
    thread = Thread(target=executor.spin, daemon=True)
    thread.start()
    assert client.wait_for_server(timeout_sec=3)
    try:
        yield server, client, backend
    finally:
        backend.release.set()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        client.destroy()
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown()


def send(client, prompt='count', max_age=0.0, timeout=0.0):
    goal = AnalyzeImage.Goal()
    goal.prompt = prompt
    goal.max_frame_age_sec = float(max_age)
    goal.timeout_sec = float(timeout)
    return await_future(client.send_goal_async(goal))


def result(handle):
    return await_future(handle.get_result_async())


def test_action_preflight_and_success(action_pair):
    server, client, backend = action_pair
    assert not send(client, prompt='').accepted
    for bad in (-1.0, float('nan'), float('inf')):
        assert not send(client, max_age=bad).accepted
        assert not send(client, timeout=bad).accepted
    assert not send(client, max_age=5.01).accepted
    no_frame = result(send(client))
    assert no_frame.status == GoalStatus.STATUS_ABORTED
    assert no_frame.result.code == AnalyzeImage.Result.NO_FRAME

    server._frames.update(frame())
    server._frames._latest = server._frames._latest.__class__(
        server._frames._latest.image, time.monotonic() - 2)
    stale = result(send(client, max_age=0.1))
    assert stale.result.code == AnalyzeImage.Result.STALE_FRAME

    server._frames.update(frame())
    backend.enabled = False
    unavailable = result(send(client))
    assert unavailable.result.code == AnalyzeImage.Result.UNAVAILABLE
    backend.enabled = True
    ok = result(send(client, prompt='apple'))
    assert ok.status == GoalStatus.STATUS_SUCCEEDED
    assert ok.result.code == AnalyzeImage.Result.OK
    assert ok.result.text == 'reply:apple'
    assert result(send(client, max_age=5.0)).result.code == AnalyzeImage.Result.OK

    backend.fail = True
    failed = result(send(client))
    assert failed.status == GoalStatus.STATUS_ABORTED
    assert failed.result.code == AnalyzeImage.Result.BACKEND_ERROR
    assert failed.result.text == ''
    backend.fail = False
    bad_image = frame()
    bad_image.encoding = 'yuyv'
    server._frames.update(bad_image)
    invalid = result(send(client))
    assert invalid.status == GoalStatus.STATUS_ABORTED
    assert invalid.result.code == AnalyzeImage.Result.INVALID_IMAGE


def test_cancel_timeout_and_late_result_not_reused(action_pair):
    server, client, backend = action_pair
    server._frames.update(frame())
    backend.blocking = True
    first = send(client, timeout=0.1)
    assert first.accepted and backend.started.wait(1)
    timeout_result = result(first)
    assert timeout_result.status == GoalStatus.STATUS_ABORTED
    assert timeout_result.result.code == AnalyzeImage.Result.TIMEOUT
    assert not send(client).accepted
    backend.release.set()
    deadline = time.monotonic() + 1
    while server._worker.current_token() is not None and time.monotonic() < deadline:
        time.sleep(0.01)
    assert server._worker.current_token() is None
    backend.blocking = False
    second = result(send(client, prompt='new'))
    assert second.result.code == AnalyzeImage.Result.OK
    assert second.result.text == 'reply:new'

    backend.started.clear()
    backend.release.clear()
    backend.blocking = True
    canceled_handle = send(client, prompt='cancel', timeout=1)
    assert backend.started.wait(1)
    cancel_response = await_future(canceled_handle.cancel_goal_async())
    assert cancel_response.goals_canceling
    canceled = result(canceled_handle)
    assert canceled.status == GoalStatus.STATUS_CANCELED
    assert canceled.result.code == AnalyzeImage.Result.CANCELED
    assert not send(client).accepted


def test_default_camera_topic_receives_frame(action_pair):
    server, client, _backend = action_pair
    assert server.get_parameter('image_topic').value == '/camera/image_raw'
    assert server._frames.snapshot() is None
    publisher = server.create_publisher(Image, '/camera/image_raw', 10)
    try:
        deadline = time.monotonic() + 2
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert publisher.get_subscription_count() > 0
        publisher.publish(frame())
        while server._frames.snapshot() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert server._frames.snapshot() is not None
        assert result(send(client)).result.code == AnalyzeImage.Result.OK
    finally:
        server.destroy_publisher(publisher)
