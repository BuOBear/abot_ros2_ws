"""Optional, lazy-loaded offline faster-whisper adapter."""

from __future__ import annotations

from pathlib import Path

from .speech_worker import BackendUnavailable


class FasterWhisperBackend:
    """Load a pre-provisioned local model only on the first transcription."""

    def __init__(
        self,
        model_path: str = '',
        *,
        device: str = 'cpu',
        compute_type: str = 'int8',
        language: str = 'zh',
        beam_size: int = 5,
    ) -> None:
        self.model_path = model_path
        self.device = device
        self.compute_type = compute_type
        self.language = language
        self.beam_size = beam_size
        self._model = None

    def transcribe(self, wav_path: Path) -> str:
        model = self._load_model()
        segments, _info = model.transcribe(
            str(wav_path), language=self.language, beam_size=self.beam_size)
        return ''.join(segment.text for segment in segments).strip()

    def _load_model(self):
        if self._model is not None:
            return self._model
        if not self.model_path:
            raise BackendUnavailable(
                'model_path is unset; configure an existing local faster-whisper model directory')

        model_path = Path(self.model_path).expanduser()
        try:
            resolved_model_path = model_path.resolve(strict=True)
        except OSError as exc:
            raise BackendUnavailable(
                'model_path must name an existing local directory; model downloads are disabled') from exc
        if not resolved_model_path.is_dir():
            raise BackendUnavailable(
                'model_path must name an existing local directory; model downloads are disabled')

        try:
            from faster_whisper import WhisperModel
        except ImportError as exc:
            raise BackendUnavailable(
                'optional faster-whisper Python package is missing from the ROS 2 Python environment') from exc

        try:
            # Supplying a verified directory prevents faster-whisper from treating
            # a model label as a repository name and attempting a download.
            self._model = WhisperModel(
                str(resolved_model_path),
                device=self.device,
                compute_type=self.compute_type,
            )
        except Exception as exc:
            raise BackendUnavailable(
                f'failed to load the configured local faster-whisper model ({type(exc).__name__})') from exc
        return self._model
