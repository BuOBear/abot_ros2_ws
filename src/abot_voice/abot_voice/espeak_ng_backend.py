"""Optional local eSpeak NG speech playback adapter."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import threading
import math

from .speech_output_worker import (
    SpeechOutputTimeout,
    SpeechOutputUnavailable,
    _VOICE_NAME,
    speech_text_error,
)


class EspeakNgBackend:
    """Speak text through a locally installed eSpeak NG executable.

    The executable is resolved only when an enabled node receives a request.
    Arguments are passed directly to subprocess, and announcement text travels
    on stdin; neither is interpreted by a shell.
    """

    def __init__(self, executable: str = 'espeak-ng', voice: str = 'zh') -> None:
        if not isinstance(executable, str) or not executable or '\x00' in executable:
            raise ValueError('executable must be a non-empty local executable name or path')
        if not isinstance(voice, str) or _VOICE_NAME.fullmatch(voice) is None:
            raise ValueError('voice must be a simple eSpeak voice name')
        self.executable = executable
        self.voice = voice
        self._lock = threading.Lock()
        self._closed = False
        self._process: subprocess.Popen[bytes] | None = None

    def speak(self, text: str, timeout_sec: float) -> None:
        text_error = speech_text_error(text)
        if text_error:
            raise ValueError(text_error)
        if (isinstance(timeout_sec, bool) or not isinstance(timeout_sec, (int, float))
                or not math.isfinite(timeout_sec) or timeout_sec <= 0):
            raise ValueError('timeout_sec must be finite and positive')
        executable = self._resolve_executable()

        with self._lock:
            if self._closed:
                raise SpeechOutputUnavailable('speech backend is shutting down')
            try:
                process = subprocess.Popen(
                    [executable, '--stdin', '-v', self.voice],
                    stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    shell=False,
                    close_fds=True,
                )
            except OSError as exc:
                raise SpeechOutputUnavailable(
                    f'could not start local eSpeak NG executable ({type(exc).__name__})') from exc
            self._process = process

        try:
            try:
                process.communicate(
                    input=text.encode('utf-8'), timeout=float(timeout_sec))
            except subprocess.TimeoutExpired as exc:
                self._kill_process(process)
                raise SpeechOutputTimeout('local eSpeak NG exceeded request timeout') from exc
            if process.returncode != 0:
                raise SpeechOutputUnavailable(
                    f'local eSpeak NG exited with status {process.returncode}')
        finally:
            with self._lock:
                if self._process is process:
                    self._process = None

    def stop(self) -> None:
        """Stop active playback and make this backend unusable after shutdown."""
        with self._lock:
            self._closed = True
            process = self._process
        if process is not None:
            self._kill_process(process)

    def _resolve_executable(self) -> str:
        if os.path.dirname(self.executable):
            candidate = Path(self.executable).expanduser()
            try:
                resolved = candidate.resolve(strict=True)
            except OSError as exc:
                raise SpeechOutputUnavailable(
                    'configured eSpeak NG executable is not available locally') from exc
            if not resolved.is_file() or not os.access(resolved, os.X_OK):
                raise SpeechOutputUnavailable(
                    'configured eSpeak NG executable is not an executable file')
            return str(resolved)

        executable = shutil.which(self.executable)
        if executable is None:
            raise SpeechOutputUnavailable(
                'eSpeak NG is not installed; install the local espeak-ng system package')
        return executable

    @staticmethod
    def _kill_process(process: subprocess.Popen[bytes]) -> None:
        if process.poll() is None:
            try:
                process.kill()
            except OSError:
                pass
            try:
                process.wait(timeout=1.0)
            except (OSError, subprocess.TimeoutExpired):
                pass
