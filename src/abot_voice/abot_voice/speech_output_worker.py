"""Bounded, ROS-independent worker for local text-to-speech announcements."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import re
import threading
import time
import unicodedata
from typing import Protocol

from .speech_worker import valid_request_id


MAX_TEXT_CHARS = 240
MAX_TEXT_BYTES = 1024
_VOICE_NAME = re.compile(r'^[A-Za-z][A-Za-z0-9_-]{0,31}$')


class SpeechOutputError(RuntimeError):
    """Base error for local synthesis failures."""


class SpeechOutputUnavailable(SpeechOutputError):
    """The configured local synthesizer cannot be used."""


class SpeechOutputTimeout(SpeechOutputError):
    """The local synthesizer exceeded its request deadline."""


class SpeechOutputBackend(Protocol):
    def speak(self, text: str, timeout_sec: float) -> None:
        """Synthesize and play a single bounded announcement."""

    def stop(self) -> None:
        """Stop active playback and reject any later work after shutdown."""


@dataclass(frozen=True)
class SpeechOutputEvent:
    request_id: str
    state: str
    detail: str = ''


@dataclass
class _Job:
    request_id: str
    text: str
    deadline: float
    timed_out: bool = False


def speech_text_error(value: object) -> str:
    """Return a short validation reason for a bounded, single-line utterance."""
    if not isinstance(value, str):
        return 'text must be a string'
    if len(value) > MAX_TEXT_CHARS:
        return f'text must be at most {MAX_TEXT_CHARS} characters'
    try:
        encoded = value.encode('utf-8')
    except UnicodeEncodeError:
        return 'text must be valid UTF-8'
    if len(encoded) > MAX_TEXT_BYTES:
        return f'text must be at most {MAX_TEXT_BYTES} UTF-8 bytes'
    if any(unicodedata.category(char) in {'Cc', 'Cs'} for char in value):
        return 'text must be a single line without control characters'
    if not value.strip():
        return 'text must not be empty'
    return ''


class SpeechOutputWorker:
    """Run one local synthesis request at a time with bounded admission/status.

    The backend receives the remaining time for each job and is responsible for
    stopping its child process at that deadline. The worker also reports queue
    deadlines promptly and discards any late completion from injected backends.
    """

    def __init__(
        self,
        backend: SpeechOutputBackend,
        *,
        queue_size: int = 1,
        timeout_sec: float = 20.0,
    ) -> None:
        if isinstance(queue_size, bool) or not isinstance(queue_size, int) or queue_size < 1:
            raise ValueError('queue_size must be an integer of at least one')
        if not math.isfinite(timeout_sec) or timeout_sec <= 0 or timeout_sec > 120:
            raise ValueError('timeout_sec must be finite and in the range (0, 120]')
        self.backend = backend
        self.queue_size = queue_size
        self.timeout_sec = timeout_sec

        self._condition = threading.Condition()
        self._pending: deque[_Job] = deque()
        self._jobs: dict[str, _Job] = {}
        self._events: dict[str, SpeechOutputEvent] = {}
        self._active: _Job | None = None
        self._closed = False
        self._worker_thread = threading.Thread(
            target=self._run, name='abot-voice-output', daemon=True)
        self._deadline_thread = threading.Thread(
            target=self._watch_deadlines, name='abot-voice-output-deadlines', daemon=True)
        self._worker_thread.start()
        self._deadline_thread.start()

    def submit(self, request_id: object, text: object) -> tuple[bool, str]:
        if not valid_request_id(request_id):
            return False, 'id must contain 1-64 letters, digits, dots, underscores, or hyphens'
        error = speech_text_error(text)
        if error:
            return False, error
        assert isinstance(text, str)
        text = text.strip()

        with self._condition:
            if self._closed:
                return False, 'worker is shutting down'
            if request_id in self._jobs or request_id in self._events:
                return False, 'request ID is already in use'
            if len(self._pending) >= self.queue_size:
                return False, 'request queue is full'
            terminal_events = sum(key not in self._jobs for key in self._events)
            if len(self._jobs) + terminal_events >= self.queue_size + 1:
                return False, 'status mailbox is full; retry after it is drained'

            job = _Job(request_id, text, time.monotonic() + self.timeout_sec)
            self._jobs[request_id] = job
            self._pending.append(job)
            self._events[request_id] = SpeechOutputEvent(request_id, 'queued')
            self._condition.notify_all()
            return True, ''

    def drain_events(self) -> list[SpeechOutputEvent]:
        with self._condition:
            events = list(self._events.values())
            self._events.clear()
            self._condition.notify_all()
            return events

    def close(self, join_timeout_sec: float = 1.0) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
            for job in self._pending:
                self._events[job.request_id] = SpeechOutputEvent(
                    job.request_id, 'cancelled', 'worker is shutting down')
                self._jobs.pop(job.request_id, None)
            self._pending.clear()
            self._condition.notify_all()

        # The eSpeak adapter terminates its active child and prevents a race
        # where another child could start after shutdown begins.
        try:
            self.backend.stop()
        except Exception:
            pass
        self._worker_thread.join(timeout=max(0.0, join_timeout_sec))
        self._deadline_thread.join(timeout=max(0.0, join_timeout_sec))

    def _emit_timeout_locked(self, job: _Job) -> None:
        if job.timed_out:
            return
        job.timed_out = True
        self._events[job.request_id] = SpeechOutputEvent(
            job.request_id, 'timed_out', 'request deadline exceeded; late result ignored')

    def _watch_deadlines(self) -> None:
        with self._condition:
            while not self._closed:
                now = time.monotonic()
                for job in list(self._jobs.values()):
                    if job.deadline <= now:
                        self._emit_timeout_locked(job)
                        if job is not self._active:
                            try:
                                self._pending.remove(job)
                            except ValueError:
                                pass
                            self._jobs.pop(job.request_id, None)
                deadlines = [job.deadline for job in self._jobs.values() if not job.timed_out]
                wait_sec = max(0.0, min(deadlines) - time.monotonic()) if deadlines else None
                self._condition.wait(timeout=wait_sec)

    def _run(self) -> None:
        while True:
            with self._condition:
                while not self._pending and not self._closed:
                    self._condition.wait()
                if self._closed and not self._pending:
                    return
                job = self._pending.popleft()
                if self._jobs.get(job.request_id) is not job:
                    continue
                remaining = job.deadline - time.monotonic()
                if remaining <= 0:
                    self._emit_timeout_locked(job)
                    self._jobs.pop(job.request_id, None)
                    self._condition.notify_all()
                    continue
                self._active = job
                self._events[job.request_id] = SpeechOutputEvent(job.request_id, 'running')
                self._condition.notify_all()

            error: Exception | None = None
            try:
                self.backend.speak(job.text, remaining)
            except Exception as exc:
                error = exc

            with self._condition:
                if time.monotonic() >= job.deadline:
                    self._emit_timeout_locked(job)
                elif not job.timed_out:
                    if error is None:
                        event = SpeechOutputEvent(job.request_id, 'completed')
                    elif isinstance(error, SpeechOutputTimeout):
                        event = SpeechOutputEvent(
                            job.request_id, 'timed_out', 'local synthesizer exceeded the deadline')
                    elif isinstance(error, SpeechOutputUnavailable):
                        event = SpeechOutputEvent(job.request_id, 'failed', str(error)[:240])
                    else:
                        event = SpeechOutputEvent(
                            job.request_id, 'failed', f'speech output failed ({type(error).__name__})')
                    self._events[job.request_id] = event
                self._jobs.pop(job.request_id, None)
                if self._active is job:
                    self._active = None
                self._condition.notify_all()
