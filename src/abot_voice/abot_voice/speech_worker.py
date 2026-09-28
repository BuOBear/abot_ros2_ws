"""Bounded, ROS-independent speech request worker and WAV inbox validation."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import os
from pathlib import Path
import re
import stat
import threading
import time
import wave
from typing import Callable


_REQUEST_ID = re.compile(r'^[A-Za-z0-9_.-]{1,64}$')


class AudioInputError(ValueError):
    """An audio request violates the configured inbox contract."""


class BackendUnavailable(RuntimeError):
    """The optional local speech backend cannot be used as configured."""


@dataclass(frozen=True)
class SpeechEvent:
    request_id: str
    state: str
    detail: str = ''
    text: str = ''


@dataclass
class _Job:
    request_id: str
    filename: str
    deadline: float
    timed_out: bool = False


def valid_request_id(value: object) -> bool:
    return isinstance(value, str) and _REQUEST_ID.fullmatch(value) is not None


def valid_audio_filename(value: object) -> bool:
    """Allow only one WAV filename, never an arbitrary path from a ROS message."""
    if not isinstance(value, str) or not value or value in {'.', '..'}:
        return False
    if '\x00' in value or '/' in value or '\\' in value:
        return False
    return Path(value).name == value and Path(value).suffix.lower() == '.wav'


def resolve_wav_file(
    audio_root: str | os.PathLike[str],
    filename: str,
    *,
    max_file_bytes: int = 5_000_000,
    max_duration_sec: float = 60.0,
) -> Path:
    """Resolve and validate a PCM WAV directly inside a configured inbox."""
    if not valid_audio_filename(filename):
        raise AudioInputError('file must be a WAV basename inside the configured inbox')
    if max_file_bytes <= 0 or not math.isfinite(max_duration_sec) or max_duration_sec <= 0:
        raise ValueError('audio limits must be finite positive values')

    root = Path(audio_root).expanduser().resolve(strict=False)
    candidate = root / filename
    try:
        root_stat = root.stat()
        file_stat = candidate.lstat()
    except OSError as exc:
        raise AudioInputError('audio inbox or requested file is unavailable') from exc
    if not stat.S_ISDIR(root_stat.st_mode):
        raise AudioInputError('configured audio inbox is not a directory')
    if stat.S_ISLNK(file_stat.st_mode) or not stat.S_ISREG(file_stat.st_mode):
        raise AudioInputError('requested audio must be a regular, non-symlink file')
    if file_stat.st_size <= 0 or file_stat.st_size > max_file_bytes:
        raise AudioInputError('WAV file is empty or exceeds the configured byte limit')

    resolved = candidate.resolve(strict=True)
    if resolved.parent != root:
        raise AudioInputError('requested audio resolves outside the configured inbox')
    try:
        with wave.open(str(resolved), 'rb') as wav_file:
            channels = wav_file.getnchannels()
            sample_width = wav_file.getsampwidth()
            sample_rate = wav_file.getframerate()
            frames = wav_file.getnframes()
            compression = wav_file.getcomptype()
    except (OSError, EOFError, wave.Error) as exc:
        raise AudioInputError('requested file is not a readable PCM WAV') from exc

    if compression != 'NONE' or channels not in (1, 2) or sample_width != 2 or sample_rate != 16000:
        raise AudioInputError('WAV must be uncompressed signed 16-bit PCM at 16000 Hz, mono or stereo')
    duration = frames / sample_rate
    if frames <= 0 or duration > max_duration_sec:
        raise AudioInputError('WAV duration is empty or exceeds the configured limit')
    return resolved


class SpeechWorker:
    """Run one transcription at a time with a bounded pending queue and deadlines.

    A timed-out backend call cannot be forcibly interrupted safely. Its timeout
    is reported promptly and its late text is discarded; the worker remains
    single-threaded until that call returns, so backend calls cannot accumulate.
    """

    def __init__(
        self,
        audio_root: str | os.PathLike[str],
        transcribe: Callable[[Path], str],
        *,
        queue_size: int = 1,
        timeout_sec: float = 45.0,
        max_file_bytes: int = 5_000_000,
        max_duration_sec: float = 60.0,
    ) -> None:
        if queue_size < 1:
            raise ValueError('queue_size must be at least one')
        if not math.isfinite(timeout_sec) or timeout_sec <= 0:
            raise ValueError('timeout_sec must be finite and positive')
        self.audio_root = Path(audio_root).expanduser()
        self.transcribe = transcribe
        self.queue_size = queue_size
        self.timeout_sec = timeout_sec
        self.max_file_bytes = max_file_bytes
        self.max_duration_sec = max_duration_sec

        self._condition = threading.Condition()
        self._pending: deque[_Job] = deque()
        self._jobs: dict[str, _Job] = {}
        # Keep only the latest event per ID. This queue is bounded by admission
        # control even if the ROS timer is temporarily unable to drain it.
        self._events: dict[str, SpeechEvent] = {}
        self._active: _Job | None = None
        self._closed = False
        self._worker_thread = threading.Thread(
            target=self._run, name='abot-voice-worker', daemon=True)
        self._deadline_thread = threading.Thread(
            target=self._watch_deadlines, name='abot-voice-deadlines', daemon=True)
        self._worker_thread.start()
        self._deadline_thread.start()

    def submit(self, request_id: str, filename: str) -> tuple[bool, str]:
        """Queue a request; return false and a short reason when it is rejected."""
        if not valid_request_id(request_id):
            return False, 'request ID must contain 1-64 letters, digits, dots, underscores, or hyphens'
        if not valid_audio_filename(filename):
            return False, 'file must be a WAV basename inside the configured inbox'
        with self._condition:
            if self._closed:
                return False, 'worker is shutting down'
            if request_id in self._jobs or request_id in self._events:
                return False, 'request ID is already in use'
            if len(self._pending) >= self.queue_size:
                return False, 'request queue is full'
            # Do not accept more work than the bounded event mailbox can report
            # if a ROS executor stalls. Running and pending jobs are counted once.
            terminal_events = sum(request_id not in self._jobs for request_id in self._events)
            if len(self._jobs) + terminal_events >= self.queue_size + 1:
                return False, 'status mailbox is full; retry after it is drained'

            job = _Job(request_id, filename, time.monotonic() + self.timeout_sec)
            self._jobs[request_id] = job
            self._pending.append(job)
            self._events[request_id] = SpeechEvent(request_id, 'queued')
            self._condition.notify_all()
            return True, ''

    def drain_events(self) -> list[SpeechEvent]:
        """Return the latest state for each request and free mailbox capacity."""
        with self._condition:
            events = list(self._events.values())
            self._events.clear()
            self._condition.notify_all()
            return events

    def close(self, join_timeout_sec: float = 0.2) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
            for job in self._pending:
                self._events[job.request_id] = SpeechEvent(
                    job.request_id, 'cancelled', 'worker is shutting down')
                self._jobs.pop(job.request_id, None)
            self._pending.clear()
            self._condition.notify_all()
        self._worker_thread.join(timeout=max(0.0, join_timeout_sec))
        self._deadline_thread.join(timeout=max(0.0, join_timeout_sec))

    def _emit_timeout_locked(self, job: _Job) -> None:
        if job.timed_out:
            return
        job.timed_out = True
        self._events[job.request_id] = SpeechEvent(
            job.request_id, 'timed_out', 'request deadline exceeded; late result will be discarded')

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
                    continue  # The deadline monitor already removed it.
                if job.deadline <= time.monotonic():
                    self._emit_timeout_locked(job)
                    self._jobs.pop(job.request_id, None)
                    self._condition.notify_all()
                    continue
                self._active = job
                self._events[job.request_id] = SpeechEvent(job.request_id, 'running')
                self._condition.notify_all()

            try:
                wav_path = resolve_wav_file(
                    self.audio_root,
                    job.filename,
                    max_file_bytes=self.max_file_bytes,
                    max_duration_sec=self.max_duration_sec,
                )
                recognized = self.transcribe(wav_path)
                if not isinstance(recognized, str):
                    raise TypeError('transcription backend must return text')
                recognized = recognized.strip()
                error: Exception | None = None
            except Exception as exc:  # Backend and file errors become request status.
                recognized = ''
                error = exc

            with self._condition:
                now = time.monotonic()
                if now >= job.deadline:
                    self._emit_timeout_locked(job)
                elif not job.timed_out:
                    if error is None and recognized:
                        event = SpeechEvent(job.request_id, 'completed', text=recognized)
                    elif error is None:
                        event = SpeechEvent(job.request_id, 'no_speech')
                    else:
                        if isinstance(error, (AudioInputError, BackendUnavailable)):
                            detail = str(error)[:240]
                        else:
                            detail = f'recognition failed ({type(error).__name__})'
                        event = SpeechEvent(job.request_id, 'failed', detail)
                    self._events[job.request_id] = event
                self._jobs.pop(job.request_id, None)
                if self._active is job:
                    self._active = None
                self._condition.notify_all()
