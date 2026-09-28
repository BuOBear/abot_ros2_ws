"""Opt-in ROS 2 adapter for bounded local speech announcements."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .espeak_ng_backend import EspeakNgBackend
from .speech_output_worker import (
    MAX_TEXT_BYTES,
    MAX_TEXT_CHARS,
    SpeechOutputEvent,
    SpeechOutputWorker,
    speech_text_error,
)
from .speech_worker import valid_request_id


MAX_REQUEST_BYTES = 4096


def _unique_json_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate JSON object key')
        result[key] = value
    return result


class VoiceAnnouncer(Node):
    """Accept explicit JSON announcements; audio remains off by default."""

    def __init__(self) -> None:
        super().__init__('voice_announcer')
        self.declare_parameter('enabled', False)
        self.declare_parameter('executable', 'espeak-ng')
        self.declare_parameter('voice', 'zh')
        self.declare_parameter('queue_size', 1)
        self.declare_parameter('request_timeout_sec', 20.0)

        self.worker: SpeechOutputWorker | None = None
        if self.get_parameter('enabled').value:
            backend = EspeakNgBackend(
                self.get_parameter('executable').value,
                voice=self.get_parameter('voice').value,
            )
            self.worker = SpeechOutputWorker(
                backend,
                queue_size=self.get_parameter('queue_size').value,
                timeout_sec=self.get_parameter('request_timeout_sec').value,
            )
            self.get_logger().warning(
                'Local speech output is enabled. Accepted /voice/speak/request '
                'messages may play audio through the system audio device.')
        else:
            self.get_logger().info(
                'Local speech output is disabled; no synthesizer or audio device is opened. '
                'Set enabled:=true to accept explicit announcement requests.')

        self.status_publisher = self.create_publisher(String, '/voice/speak/status', 10)
        self.request_subscription = self.create_subscription(
            String, '/voice/speak/request', self.on_request, 10)
        self._drain_timer = self.create_timer(0.05, self.publish_worker_events)

    def on_request(self, message: String) -> None:
        if not isinstance(message.data, str) or len(message.data) > MAX_REQUEST_BYTES:
            self.publish_status('', 'rejected', 'request exceeds the 4096-byte limit')
            return
        try:
            encoded_size = len(message.data.encode('utf-8'))
        except (AttributeError, UnicodeEncodeError):
            self.publish_status('', 'rejected', 'request must be valid UTF-8 text')
            return
        if encoded_size > MAX_REQUEST_BYTES:
            self.publish_status('', 'rejected', 'request exceeds the 4096-byte limit')
            return
        try:
            request = json.loads(message.data, object_pairs_hook=_unique_json_object)
        except (json.JSONDecodeError, TypeError, ValueError):
            self.publish_status('', 'rejected', 'request must be a JSON object')
            return
        if not isinstance(request, dict):
            self.publish_status('', 'rejected', 'request must be a JSON object')
            return

        request_id = request.get('id', '')
        if not valid_request_id(request_id):
            self.publish_status('', 'rejected', 'id must contain 1-64 safe ASCII characters')
            return
        if set(request) != {'id', 'text'}:
            self.publish_status(request_id, 'rejected', 'request fields must be exactly id and text')
            return
        text = request.get('text')
        text_error = speech_text_error(text)
        if text_error:
            self.publish_status(request_id, 'rejected', text_error)
            return

        if self.worker is None:
            self.publish_status(request_id, 'rejected', 'speech output is disabled')
            return
        accepted, detail = self.worker.submit(request_id, text)
        if not accepted:
            self.publish_status(request_id, 'rejected', detail)

    def publish_worker_events(self) -> None:
        if self.worker is None:
            return
        for event in self.worker.drain_events():
            self.publish_event(event)

    def publish_event(self, event: SpeechOutputEvent) -> None:
        self.publish_status(event.request_id, event.state, event.detail)
        if event.state in {'failed', 'timed_out'}:
            self.get_logger().warning(
                f'Speech request {event.request_id} ended as {event.state}: {event.detail}')

    def publish_status(self, request_id: str, state: str, detail: str = '') -> None:
        message = String()
        message.data = json.dumps(
            {'id': request_id, 'state': state, 'detail': detail}, ensure_ascii=False)
        self.status_publisher.publish(message)

    def destroy_node(self):
        if self.worker is not None:
            self.worker.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceAnnouncer()
    try:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
