import threading
import time
import wave

import pytest

from abot_voice.faster_whisper_backend import FasterWhisperBackend
from abot_voice.speech_worker import (
    AudioInputError,
    BackendUnavailable,
    SpeechWorker,
    resolve_wav_file,
    valid_audio_filename,
    valid_request_id,
)


def write_wav(path, *, sample_rate=16000, channels=1, sample_width=2, duration=0.1):
    frames = int(sample_rate * duration)
    with wave.open(str(path), 'wb') as wav_file:
        wav_file.setnchannels(channels)
        wav_file.setsampwidth(sample_width)
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(b'\0' * frames * channels * sample_width)
    return path


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


def test_request_contract_and_wav_inbox_reject_paths_symlinks_and_wrong_format(tmp_path):
    inbox = tmp_path / 'inbox'
    inbox.mkdir()
    wav_path = write_wav(inbox / 'sample.wav', channels=2)

    assert valid_request_id('request.1-okay')
    assert not valid_request_id('bad/id')
    assert valid_audio_filename('sample.wav')
    for unsafe_name in ('../sample.wav', '/tmp/sample.wav', 'sub/sample.wav', 'clip.mp3'):
        assert not valid_audio_filename(unsafe_name)
    assert resolve_wav_file(inbox, 'sample.wav') == wav_path.resolve()

    outside = write_wav(tmp_path / 'outside.wav')
    (inbox / 'link.wav').symlink_to(outside)
    with pytest.raises(AudioInputError, match='regular, non-symlink'):
        resolve_wav_file(inbox, 'link.wav')

    write_wav(inbox / 'wrong-rate.wav', sample_rate=8000)
    with pytest.raises(AudioInputError, match='16000 Hz'):
        resolve_wav_file(inbox, 'wrong-rate.wav')


def test_worker_uses_fake_backend_off_executor_and_bounds_pending_requests(tmp_path):
    inbox = tmp_path / 'inbox'
    inbox.mkdir()
    write_wav(inbox / 'first.wav')
    write_wav(inbox / 'second.wav')
    backend_started = threading.Event()
    release_backend = threading.Event()
    called = []

    def fake_backend(path):
        called.append(path.name)
        if path.name == 'first.wav':
            backend_started.set()
            assert release_backend.wait(1.0)
        return f'text from {path.stem}'

    worker = SpeechWorker(inbox, fake_backend, queue_size=1, timeout_sec=2.0)
    try:
        assert worker.submit('first', 'first.wav') == (True, '')
        assert backend_started.wait(0.5), 'backend did not start on the worker thread'
        assert worker.submit('second', 'second.wav') == (True, '')
        accepted, reason = worker.submit('third', 'second.wav')
        assert not accepted and reason == 'request queue is full'

        release_backend.set()
        events = wait_for_states(worker, {
            'first': {'completed'},
            'second': {'completed'},
        })
        assert events['first'].text == 'text from first'
        assert events['second'].text == 'text from second'
        assert called == ['first.wav', 'second.wav']
    finally:
        release_backend.set()
        worker.close()


def test_timeout_is_reported_promptly_and_discards_late_backend_result(tmp_path):
    inbox = tmp_path / 'inbox'
    inbox.mkdir()
    write_wav(inbox / 'slow.wav')
    backend_started = threading.Event()
    backend_finished = threading.Event()

    def slow_backend(_path):
        backend_started.set()
        time.sleep(0.2)
        backend_finished.set()
        return 'late text must not escape'

    worker = SpeechWorker(inbox, slow_backend, timeout_sec=0.05)
    try:
        assert worker.submit('slow', 'slow.wav')[0]
        assert backend_started.wait(0.5)
        timed_out = wait_for_states(worker, {'slow': {'timed_out'}}, timeout=0.3)['slow']
        assert timed_out.state == 'timed_out'
        assert backend_finished.wait(0.5)
        time.sleep(0.02)
        assert all(event.state != 'completed' for event in worker.drain_events())
    finally:
        worker.close()


def test_missing_or_nonlocal_model_is_reported_without_download_attempt(tmp_path):
    with pytest.raises(BackendUnavailable, match='model_path is unset'):
        FasterWhisperBackend().transcribe(tmp_path / 'unused.wav')

    with pytest.raises(BackendUnavailable, match='model downloads are disabled'):
        FasterWhisperBackend(str(tmp_path / 'missing-model')).transcribe(tmp_path / 'unused.wav')


def test_missing_file_fails_as_request_status_without_calling_backend(tmp_path):
    inbox = tmp_path / 'inbox'
    inbox.mkdir()
    called = []
    worker = SpeechWorker(inbox, lambda path: called.append(path) or 'unexpected')
    try:
        assert worker.submit('missing', 'missing.wav')[0]
        events = wait_for_states(worker, {'missing': {'failed'}})
        assert events['missing'].detail == 'audio inbox or requested file is unavailable'
        assert called == []
    finally:
        worker.close()
