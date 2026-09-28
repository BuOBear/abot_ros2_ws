from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Event, Thread
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from abot_vlm_interfaces.action import AnalyzeImage
from abot_vlm.worker import ArkBackend
from abot_vlm.vlm_action import VlmAction


@pytest.fixture
def trickle_server(monkeypatch):
    received = Event()
    stop = Event()

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            length = int(self.headers['Content-Length'])
            self.rfile.read(length)
            received.set()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            while not stop.is_set():
                try:
                    self.wfile.write(b' ')
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    break
                time.sleep(0.02)

        def log_message(self, _format, *_args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    server.daemon_threads = True
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    monkeypatch.setenv('ABOT_ARK_API_KEY', 'fake-local-test-key')
    monkeypatch.setenv('ABOT_ARK_MODEL', 'fake-local-test-model')
    monkeypatch.setenv('NO_PROXY', '127.0.0.1,localhost')
    monkeypatch.setenv('no_proxy', '127.0.0.1,localhost')
    try:
        yield f'http://127.0.0.1:{server.server_port}/', received
    finally:
        stop.set()
        server.shutdown()
        server.server_close()
        thread.join(timeout=1)


def test_trickle_response_obeys_total_wall_clock_deadline(trickle_server):
    endpoint, received = trickle_server
    backend = ArkBackend(endpoint=endpoint)
    backend.prepare()
    start = time.monotonic()
    with pytest.raises(TimeoutError):
        backend.analyze('count', b'fake-jpeg', 0.35)
    elapsed = time.monotonic() - start
    assert received.is_set()
    assert elapsed < 1.2


def test_abort_kills_inflight_request_and_releases_worker(trickle_server):
    endpoint, received = trickle_server
    backend = ArkBackend(endpoint=endpoint)
    backend.prepare()
    with ThreadPoolExecutor(max_workers=1) as executor:
        future = executor.submit(backend.analyze, 'count', b'fake-jpeg', 5.0)
        assert received.wait(2)
        start = time.monotonic()
        backend.abort()
        with pytest.raises(TimeoutError):
            future.result(timeout=1)
        assert time.monotonic() - start < 1.0


def test_action_timeout_kills_child_then_accepts_next_goal(trickle_server):
    endpoint, received = trickle_server
    rclpy.init()
    server = VlmAction(backend=ArkBackend(endpoint=endpoint))
    client_node = Node('vlm_ark_deadline_client')
    client = ActionClient(client_node, AnalyzeImage, '/vlm/analyze')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(client_node)
    spin = Thread(target=executor.spin, daemon=True)
    spin.start()

    def await_ros(future, timeout=3):
        limit = time.monotonic() + timeout
        while not future.done() and time.monotonic() < limit:
            time.sleep(0.01)
        assert future.done()
        return future.result()

    def send_goal():
        image = Image()
        image.width = 4
        image.height = 3
        image.encoding = 'rgb8'
        image.step = 12
        image.data = bytes([100] * 36)
        server._frames.update(image)
        request = AnalyzeImage.Goal()
        request.prompt = 'count'
        request.timeout_sec = 0.35
        return await_ros(client.send_goal_async(request))

    try:
        assert client.wait_for_server(timeout_sec=3)
        first = send_goal()
        assert first.accepted
        assert received.wait(2)
        first_result = await_ros(first.get_result_async())
        assert first_result.status == GoalStatus.STATUS_ABORTED
        assert first_result.result.code == AnalyzeImage.Result.TIMEOUT
        limit = time.monotonic() + 1
        while server._worker.current_token() is not None and time.monotonic() < limit:
            time.sleep(0.01)
        assert time.monotonic() < limit, 'worker lease remained held after process deadline'
        second = send_goal()
        assert second.accepted
        second_result = await_ros(second.get_result_async())
        assert second_result.result.code == AnalyzeImage.Result.TIMEOUT
    finally:
        executor.shutdown(timeout_sec=3)
        spin.join(timeout=3)
        client.destroy()
        client_node.destroy_node()
        server.destroy_node()
        rclpy.shutdown()
