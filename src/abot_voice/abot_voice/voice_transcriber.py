"""ROS 2 adapter for bounded, local file-based speech recognition."""

from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .faster_whisper_backend import FasterWhisperBackend
from .speech_worker import SpeechEvent, SpeechWorker, valid_audio_filename, valid_request_id


class VoiceTranscriber(Node):
    """Accept allow-listed WAV requests and publish text/status only."""

    def __init__(self) -> None:
        super().__init__('voice_transcriber')
        self.declare_parameter(
            'audio_root', str(Path.home() / '.ros' / 'abot_voice' / 'inbox'))
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('compute_type', 'int8')
        self.declare_parameter('language', 'zh')
        self.declare_parameter('beam_size', 5)
        self.declare_parameter('queue_size', 1)
        self.declare_parameter('request_timeout_sec', 45.0)
        self.declare_parameter('max_audio_bytes', 5_000_000)
        self.declare_parameter('max_duration_sec', 60.0)

        audio_root = self.get_parameter('audio_root').value
        backend = FasterWhisperBackend(
            self.get_parameter('model_path').value,
            device=self.get_parameter('device').value,
            compute_type=self.get_parameter('compute_type').value,
            language=self.get_parameter('language').value,
            beam_size=self.get_parameter('beam_size').value,
        )
        self.worker = SpeechWorker(
            audio_root,
            backend.transcribe,
            queue_size=self.get_parameter('queue_size').value,
            timeout_sec=self.get_parameter('request_timeout_sec').value,
            max_file_bytes=self.get_parameter('max_audio_bytes').value,
            max_duration_sec=self.get_parameter('max_duration_sec').value,
        )

        self.text_publisher = self.create_publisher(String, '/voice/text', 10)
        self.status_publisher = self.create_publisher(String, '/voice/transcribe/status', 10)
        self.request_subscription = self.create_subscription(
            String, '/voice/transcribe/request', self.on_request, 10)
        self._drain_timer = self.create_timer(0.05, self.publish_worker_events)
        self.get_logger().info(
            f'Listening for WAV requests; inbox={audio_root!s}. '
            'Microphone capture and motion control are disabled.')

    def on_request(self, message: String) -> None:
        try:
            request = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.publish_status('', 'rejected', 'request must be a JSON object')
            return
        if not isinstance(request, dict):
            self.publish_status('', 'rejected', 'request must be a JSON object')
            return

        request_id = request.get('id', '')
        if not valid_request_id(request_id):
            self.publish_status('', 'rejected', 'id must contain 1-64 letters, digits, dots, underscores, or hyphens')
            return
        if set(request) != {'id', 'file'}:
            self.publish_status(request_id, 'rejected', 'request fields must be exactly id and file')
            return
        filename = request.get('file')
        if not valid_audio_filename(filename):
            self.publish_status(request_id, 'rejected', 'file must be a WAV basename inside the configured inbox')
            return

        accepted, detail = self.worker.submit(request_id, filename)
        if not accepted:
            self.publish_status(request_id, 'rejected', detail)

    def publish_worker_events(self) -> None:
        for event in self.worker.drain_events():
            self.publish_event(event)

    def publish_event(self, event: SpeechEvent) -> None:
        if event.state == 'completed':
            text_message = String()
            text_message.data = event.text
            self.text_publisher.publish(text_message)
        self.publish_status(event.request_id, event.state, event.detail)
        if event.state in {'failed', 'timed_out'}:
            self.get_logger().warning(
                f'Voice request {event.request_id} ended as {event.state}: {event.detail}')

    def publish_status(self, request_id: str, state: str, detail: str = '') -> None:
        message = String()
        message.data = json.dumps(
            {'id': request_id, 'state': state, 'detail': detail}, ensure_ascii=False)
        self.status_publisher.publish(message)

    def destroy_node(self):
        self.worker.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceTranscriber()
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
