"""
archer_ros/ai/stt/whisper_engine.py
=====================================
Local Speech-to-Text using OpenAI Whisper (fully offline).

No API key required. The model runs entirely on-device.

Usage:
    engine = WhisperEngine()
    text = engine.transcribe("path/to/audio.wav")   # from file
    text = engine.listen_and_transcribe()            # from microphone
"""

from __future__ import annotations

import io
import logging
import os
import tempfile
import wave
from pathlib import Path
from typing import Optional

logger = logging.getLogger("archer.stt")


def _load_config() -> dict:
    """Load whisper section from settings.yaml (gracefully)."""
    try:
        import yaml  # type: ignore
        cfg_path = Path(__file__).parent.parent.parent / "config" / "settings.yaml"
        with open(cfg_path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        return cfg.get("whisper", {})
    except Exception as e:
        logger.warning(f"Could not load settings.yaml: {e}. Using defaults.")
        return {}


class WhisperEngine:
    """
    Wraps OpenAI Whisper for local, offline speech transcription.

    The model is loaded once at construction time and kept in memory.
    """

    def __init__(
        self,
        model_name: Optional[str] = None,
        device: Optional[str] = None,
    ) -> None:
        cfg = _load_config()
        self.model_name = model_name or cfg.get("model", "base")
        self.device = device or cfg.get("device", "cpu")
        self.sample_rate: int = cfg.get("sample_rate", 16000)
        self.max_record_seconds: int = cfg.get("max_record_seconds", 10)
        self.silence_timeout: float = cfg.get("silence_timeout", 1.5)

        logger.info(f"Loading Whisper model '{self.model_name}' on '{self.device}'…")
        try:
            import whisper  # type: ignore
            self._model = whisper.load_model(self.model_name, device=self.device)
            logger.info("Whisper model loaded successfully.")
        except ImportError:
            raise ImportError(
                "openai-whisper is not installed. "
                "Run: pip install openai-whisper"
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def transcribe(self, audio_path: str | Path) -> str:
        """
        Transcribe audio from a file path.

        Args:
            audio_path: Path to a WAV, MP3, FLAC, or M4A file.

        Returns:
            Transcribed text (stripped), or empty string on failure.
        """
        audio_path = str(audio_path)
        logger.debug(f"Transcribing file: {audio_path}")
        try:
            import whisper  # type: ignore
            result = self._model.transcribe(
                audio_path,
                fp16=False,  # fp16 not supported on CPU
                language="en",
            )
            text: str = result.get("text", "").strip()
            logger.info(f"[STT] Transcribed: '{text}'")
            return text
        except Exception as e:
            logger.error(f"Transcription failed: {e}")
            return ""

    def listen_and_transcribe(self) -> str:
        """
        Capture audio from the default microphone and transcribe it.

        Listens until silence_timeout seconds of silence is detected,
        or max_record_seconds is reached.

        Returns:
            Transcribed text, or empty string if nothing was captured.
        """
        try:
            import sounddevice as sd  # type: ignore
            import numpy as np  # type: ignore
        except ImportError:
            raise ImportError(
                "sounddevice and numpy are required for microphone input. "
                "Run: pip install sounddevice numpy"
            )

        logger.info("Listening… (speak now)")

        # ---- record audio ----
        audio_chunks: list[np.ndarray] = []
        silence_frames = 0
        silence_frame_threshold = int(
            self.silence_timeout * self.sample_rate / 1024
        )

        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype="float32",
            blocksize=1024,
        ) as stream:
            total_frames = 0
            max_frames = int(self.max_record_seconds * self.sample_rate / 1024)

            while total_frames < max_frames:
                chunk, _ = stream.read(1024)
                audio_chunks.append(chunk.copy())
                total_frames += 1

                # Detect silence using RMS energy
                rms = float(np.sqrt(np.mean(chunk**2)))
                if rms < 0.01:  # silence threshold
                    silence_frames += 1
                    if silence_frames >= silence_frame_threshold and total_frames > 10:
                        logger.debug("Silence detected — stopping capture.")
                        break
                else:
                    silence_frames = 0

        if not audio_chunks:
            logger.warning("No audio captured.")
            return ""

        # ---- stitch chunks and write temp WAV ----
        import numpy as np  # type: ignore
        audio_np = np.concatenate(audio_chunks, axis=0).flatten()

        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            tmp_path = tmp.name

        try:
            with wave.open(tmp_path, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)  # 16-bit PCM
                wf.setframerate(self.sample_rate)
                pcm = (audio_np * 32767).astype("int16").tobytes()
                wf.writeframes(pcm)

            return self.transcribe(tmp_path)
        finally:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass


# ------------------------------------------------------------------
# Standalone test
# ------------------------------------------------------------------
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    engine = WhisperEngine(model_name="base")
    print("Speak something…")
    result = engine.listen_and_transcribe()
    print(f"\nYou said: '{result}'")
