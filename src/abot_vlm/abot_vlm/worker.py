"""One in-flight VLM request with a bounded network worker.

An expired or canceled request keeps its lease until the backend call returns.
That prevents worker queue buildup if a remote call ignores cancellation.
"""

from __future__ import annotations

import base64
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
import json
import os
import subprocess
import sys
from threading import Event, Lock
import time
from typing import Protocol

import cv2
import numpy as np
from sensor_msgs.msg import Image


class BackendUnavailable(Exception):
    pass


class InvalidImage(Exception):
    pass


class Backend(Protocol):
    def available(self) -> bool: ...
    def analyze(self, prompt: str, jpeg: bytes, timeout_sec: float) -> str: ...


@dataclass(frozen=True)
class FrameSnapshot:
    image: Image
    received_at: float


class FrameCache:
    def __init__(self) -> None:
        self._lock = Lock()
        self._latest: FrameSnapshot | None = None

    def update(self, image: Image) -> None:
        # The subscriber callback deliberately performs no conversion or I/O.
        with self._lock:
            self._latest = FrameSnapshot(image, time.monotonic())

    def snapshot(self) -> FrameSnapshot | None:
        with self._lock:
            return self._latest


def encode_jpeg(message: Image, max_pixels: int = 4_000_000) -> bytes:
    if message.width < 1 or message.height < 1 or message.width * message.height > max_pixels:
        raise InvalidImage('invalid image dimensions')
    channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3}.get(message.encoding)
    if channels is None:
        raise InvalidImage('unsupported image encoding')
    row_bytes = message.width * channels
    if message.step < row_bytes or len(message.data) < message.step * message.height:
        raise InvalidImage('truncated image')
    raw = np.frombuffer(message.data, dtype=np.uint8, count=message.step * message.height)
    rows = raw.reshape(message.height, message.step)[:, :row_bytes]
    if channels == 1:
        pixels = rows.reshape(message.height, message.width)
    else:
        pixels = rows.reshape(message.height, message.width, channels)
        if message.encoding == 'rgb8':
            pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode('.jpg', pixels)
    if not ok:
        raise InvalidImage('JPEG encoding failed')
    return encoded.tobytes()


class ArkBackend:
    """Optional Ark backend with a killable, one-request child process."""

    def __init__(self, endpoint: str = 'https://ark.cn-beijing.volces.com/api/v3/chat/completions'):
        self._endpoint = endpoint
        self._lock = Lock()
        self._abort = Event()
        self._process: subprocess.Popen | None = None

    def available(self) -> bool:
        return bool(os.environ.get('ABOT_ARK_API_KEY') and os.environ.get('ABOT_ARK_MODEL'))

    def prepare(self) -> None:
        # Called before the worker is submitted, so an immediate Action cancel
        # cannot be lost while JPEG encoding or child startup is in progress.
        self._abort.clear()

    def abort(self) -> None:
        self._abort.set()
        with self._lock:
            process = self._process
        if process is not None:
            try:
                process.kill()
            except ProcessLookupError:
                pass

    def analyze(self, prompt: str, jpeg: bytes, timeout_sec: float) -> str:
        if not self.available():
            raise BackendUnavailable('ABOT_ARK_API_KEY and ABOT_ARK_MODEL are required')
        if self._abort.is_set():
            raise TimeoutError('request stopped')
        deadline = time.monotonic() + timeout_sec
        payload = json.dumps({
            'endpoint': self._endpoint,
            'prompt': prompt,
            'jpeg': base64.b64encode(jpeg).decode('ascii'),
            'timeout_sec': timeout_sec,
        }).encode('utf-8')
        process = subprocess.Popen(
            [sys.executable, '-m', 'abot_vlm.ark_http_child'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        with self._lock:
            self._process = process
            stopped = self._abort.is_set()
        if stopped:
            try:
                process.kill()
            except ProcessLookupError:
                pass
        try:
            try:
                output, _ = process.communicate(
                    input=payload, timeout=max(0.01, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate(timeout=1.0)
                raise TimeoutError('request deadline elapsed')
        finally:
            with self._lock:
                if self._process is process:
                    self._process = None
        if self._abort.is_set():
            raise TimeoutError('request stopped')
        if process.returncode != 0 or len(output) > 1_000_000:
            raise RuntimeError('backend request failed')
        parsed = json.loads(output)
        content = parsed['text']
        if not isinstance(content, str):
            raise ValueError('backend response content is not text')
        return content


class SingleFlightWorker:
    def __init__(self, backend: Backend) -> None:
        self.backend = backend
        self._lock = Lock()
        self._next_token = 0
        self._token: int | None = None
        self._action_done = False
        self._worker_done = True
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='abot-vlm')

    def reserve(self) -> int | None:
        with self._lock:
            if self._token is not None:
                return None
            self._next_token += 1
            self._token = self._next_token
            self._action_done = False
            self._worker_done = True
            return self._token

    def current_token(self) -> int | None:
        with self._lock:
            return self._token

    def submit(self, token: int, prompt: str, frame: Image, timeout_sec: float) -> Future[str]:
        # The lease covers both the backend call and the ROS Action terminal
        # transition. A completed Future alone cannot admit a new goal.
        with self._lock:
            if token != self._token or self._action_done or not self._worker_done:
                raise RuntimeError('worker lease is not active')
            prepare = getattr(self.backend, 'prepare', None)
            if prepare is not None:
                prepare()
            self._worker_done = False
            try:
                future = self._executor.submit(self._analyze, prompt, frame, timeout_sec)
            except Exception:
                self._worker_done = True
                raise
        future.add_done_callback(lambda _: self._mark_worker_done(token))
        return future

    def _mark_worker_done(self, token: int) -> None:
        with self._lock:
            if token != self._token:
                return
            self._worker_done = True
            self._maybe_release()

    def finish(self, token: int) -> None:
        """Call only after the matching Action has entered a terminal state."""
        with self._lock:
            if token != self._token:
                return
            self._action_done = True
            self._maybe_release()

    def _maybe_release(self) -> None:
        # Caller owns _lock.
        if self._action_done and self._worker_done:
            self._token = None

    def abort_active(self, token: int) -> None:
        # Keep the lease lock through abort(). Otherwise A could release, B
        # could start, and A's delayed abort could kill B's subprocess.
        with self._lock:
            if token != self._token:
                return
            abort = getattr(self.backend, 'abort', None)
            if abort is not None:
                abort()

    def _analyze(self, prompt: str, frame: Image, timeout_sec: float) -> str:
        jpeg = encode_jpeg(frame)
        return self.backend.analyze(prompt, jpeg, timeout_sec)

    def close(self) -> None:
        with self._lock:
            token = self._token
        if token is not None:
            self.abort_active(token)
        self._executor.shutdown(wait=False, cancel_futures=True)
