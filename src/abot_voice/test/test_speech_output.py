import json
from pathlib import Path
import threading
import time

import pytest

from abot_voice.espeak_ng_backend import EspeakNgBackend
from abot_voice.speech_output_worker import (
    MAX_TEXT_CHARS,
    SpeechOutputTimeout,
    SpeechOutputUnavailable,
    SpeechOutputWorker,
    speech_text_error,
)


def wait_for_states(worker, wanted, timeout=1.0):
    deadline = time.monotonic() + timeout
    seen = {}
    while time.monotonic() < deadline:
        for event in worker.drain_events():
            if event.state in wanted.get(event.request_id, set()):
                seen[event.request_id] = event
        if all(request_id in seen for request_id in wanted):
            return seen
        time.sleep(0.005)
    raise AssertionError(f'timed out waiting for {wanted}; saw {seen}')


class FakeBackend:
    def __init__(self):
        self.calls = []
        self.started = threading.Event()
        self.release = threading.Event()
        self.stopped = False

    def speak(self, text, _timeout_sec):
        self.calls.append(text)
        self.started.set()
        self.release.wait(1.0)

    def stop(self):
        self.stopped = True
        self.release.set()


def test_speech_text_is_single_line_valid_utf8_and_bounded():
    assert speech_text_error('你好，机器人。') == ''
    assert speech_text_error('  okay  ') == ''
    assert speech_text_error('') == 'text must not be empty'
    assert speech_text_error('one\ntwo') == 'text must be a single line without control characters'
    assert speech_text_error('announcement\n') == 'text must be a single line without control characters'
    assert speech_text_error('x' * (MAX_TEXT_CHARS + 1)).startswith('text must be at most')
    assert speech_text_error('bad\ud800') == 'text must be valid UTF-8'
    assert speech_text_error(123) == 'text must be a string'


def test_worker_serializes_requests_bounds_queue_and_reports_status():
    backend = FakeBackend()
    worker = SpeechOutputWorker(backend, queue_size=1, timeout_sec=2.0)
    try:
        assert worker.submit('first', '先说第一句') == (True, '')
        assert backend.started.wait(0.5), 'backend did not start on the worker thread'
        assert worker.submit('second', '然后说第二句') == (True, '')
        accepted, reason = worker.submit('third', 'queue must stay bounded')
        assert not accepted and reason == 'request queue is full'
        assert worker.submit('invalid', 'two\nlines')[0] is False

        backend.release.set()
        events = wait_for_states(worker, {
            'first': {'completed'},
            'second': {'completed'},
        })
        assert events['first'].state == 'completed'
        assert events['second'].state == 'completed'
        assert backend.calls == ['先说第一句', '然后说第二句']
    finally:
        worker.close()
    assert backend.stopped


def test_worker_reports_deadline_and_discards_late_completion():
    backend = FakeBackend()
    worker = SpeechOutputWorker(backend, timeout_sec=0.05)
    try:
        assert worker.submit('slow', '慢一点') == (True, '')
        assert backend.started.wait(0.5)
        event = wait_for_states(worker, {'slow': {'timed_out'}}, timeout=0.3)['slow']
        assert event.detail == 'request deadline exceeded; late result ignored'
        backend.release.set()
        time.sleep(0.03)
        assert all(item.state != 'completed' for item in worker.drain_events())
    finally:
        worker.close()


def test_espeak_adapter_uses_fixed_argv_and_sends_text_on_stdin(tmp_path):
    executable = tmp_path / 'fake-espeak-ng'
    args_file = tmp_path / 'args.json'
    text_file = tmp_path / 'text.txt'
    executable.write_text(
        '#!/usr/bin/env python3\n'
        'import json, pathlib, sys\n'
        f'pathlib.Path({str(args_file)!r}).write_text(json.dumps(sys.argv[1:]))\n'
        f'pathlib.Path({str(text_file)!r}).write_text(sys.stdin.read())\n',
        encoding='utf-8',
    )
    executable.chmod(0o755)

    sentinel = tmp_path / 'shell-injection'
    user_text = f'announcement; $(touch {sentinel})'
    EspeakNgBackend(str(executable)).speak(user_text, timeout_sec=2.0)

    assert json.loads(args_file.read_text(encoding='utf-8')) == ['--stdin', '-v', 'zh']
    assert text_file.read_text(encoding='utf-8') == user_text
    assert not sentinel.exists()


def test_espeak_adapter_kills_process_when_timeout_expires(tmp_path):
    executable = tmp_path / 'slow-espeak-ng'
    executable.write_text('#!/usr/bin/env python3\nimport time\ntime.sleep(5)\n', encoding='utf-8')
    executable.chmod(0o755)

    with pytest.raises(SpeechOutputTimeout):
        EspeakNgBackend(str(executable)).speak('short announcement', timeout_sec=0.05)


def test_missing_local_synthesizer_is_reported_on_request_without_download(tmp_path, monkeypatch):
    monkeypatch.setattr('abot_voice.espeak_ng_backend.shutil.which', lambda _name: None)
    backend = EspeakNgBackend()
    with pytest.raises(SpeechOutputUnavailable, match='not installed'):
        backend.speak('hello', timeout_sec=1.0)
    backend.stop()
