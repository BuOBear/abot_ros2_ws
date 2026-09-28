from threading import Event
import time

import numpy as np
import pytest
from sensor_msgs.msg import Image

from abot_vlm.worker import FrameCache, InvalidImage, SingleFlightWorker, encode_jpeg


def image(encoding='rgb8', width=4, height=3):
    message = Image()
    message.width = width
    message.height = height
    message.encoding = encoding
    channels = 1 if encoding == 'mono8' else 3
    message.step = width * channels
    message.data = bytes([80] * message.step * height)
    return message


def test_frame_cache_and_encoding():
    cache = FrameCache()
    assert cache.snapshot() is None
    frame = image()
    cache.update(frame)
    assert cache.snapshot().image is frame
    assert encode_jpeg(frame).startswith(b'\xff\xd8')
    assert encode_jpeg(image('mono8')).startswith(b'\xff\xd8')
    with pytest.raises(InvalidImage):
        encode_jpeg(image('yuyv'))
    frame.data = b'0'
    with pytest.raises(InvalidImage):
        encode_jpeg(frame)


def test_late_worker_keeps_single_flight_lease_until_actual_completion():
    started = Event()
    release = Event()

    class Backend:
        abort_calls = 0

        def available(self):
            return True

        def analyze(self, prompt, jpeg, timeout_sec):
            started.set()
            assert release.wait(2)
            return 'answer'

        def abort(self):
            self.abort_calls += 1

    backend = Backend()
    worker = SingleFlightWorker(backend)
    try:
        token = worker.reserve()
        assert token is not None
        future = worker.submit(token, 'question', image(), 1.0)
        assert started.wait(1)
        assert worker.reserve() is None
        release.set()
        assert future.result(timeout=1) == 'answer'
        # The backend is finished, but the corresponding Action has not yet
        # processed its terminal result. A second goal must still be rejected.
        assert worker.reserve() is None
        worker.finish(token)
        limit = time.monotonic() + 1
        new_token = None
        while new_token is None and time.monotonic() < limit:
            new_token = worker.reserve()
            time.sleep(0.01)
        assert new_token is not None and new_token != token
        worker.abort_active(token)
        assert backend.abort_calls == 0
        worker.finish(new_token)
    finally:
        release.set()
        worker.close()
