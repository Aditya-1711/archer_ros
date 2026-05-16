"""
archer_ros/ai/tts/piper_engine.py
=====================================
Local Text-to-Speech using Piper neural TTS (fully offline).

Piper is a fast, high-quality TTS engine that runs without any cloud API.
It produces natural-sounding speech using ONNX neural models.

Download Piper: https://github.com/rhasspy/piper/releases
Download voices: https://huggingface.co/rhasspy/piper-voices/tree/main
Recommended voice: en_US-lessac-medium (good quality + reasonable speed)

Usage:
    engine = PiperEngine()
    engine.speak("Neural uplink established. Archer is monitoring, Boss.")
"""

from __future__ import annotations

import io
import logging
import os
import shutil
import subprocess
import tempfile
import wave
from pathlib import Path
from typing import Optional

logger = logging.getLogger("archer.tts")


def _load_config() -> dict:
    """Load piper section from settings.yaml (gracefully)."""
    try:
        import yaml  # type: ignore
        cfg_path = Path(__file__).parent.parent.parent / "config" / "settings.yaml"
        with open(cfg_path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        return cfg.get("piper", {})
    except Exception as e:
        logger.warning(f"Could not load settings.yaml: {e}. Using defaults.")
        return {}


class PiperEngine:
    """
    Wraps the Piper TTS binary for local neural text-to-speech synthesis.

    Piper must be installed and on PATH, or its path set in settings.yaml.
    Falls back to a simple print if Piper is not available (silent mode).
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        piper_binary: Optional[str] = None,
    ) -> None:
        cfg = _load_config()
        self.model_path = Path(
            model_path or cfg.get("model_path", "~/piper-models/en_US-lessac-medium.onnx")
        ).expanduser()
        self.speaker_id: int = cfg.get("speaker_id", 0)
        self.sample_rate: int = cfg.get("sample_rate", 22050)

        # Locate the piper binary
        self._piper_bin = piper_binary or shutil.which("piper") or "piper"
        self._available = self._check_available()

        if self._available:
            logger.info(f"PiperEngine ready — model='{self.model_path}'")
        else:
            logger.warning(
                "Piper binary not found or model missing. "
                "TTS will fall back to silent mode. "
                "Install: https://github.com/rhasspy/piper/releases"
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def speak(self, text: str) -> None:
        """
        Synthesise text and play it through the default audio output.

        Args:
            text: The plain text to speak. Markdown and special characters
                  are stripped automatically.
        """
        text = self._clean_text(text)
        if not text:
            return

        logger.info(f"[TTS] Speaking: '{text[:80]}'")

        if not self._available:
            # Silent fallback — print to terminal so we can still see output
            print(f"[Archer TTS — silent mode]: {text}")
            return

        try:
            self._synthesize_and_play(text)
        except Exception as e:
            logger.error(f"TTS synthesis failed: {e}")
            print(f"[Archer TTS fallback]: {text}")

    def synthesize_to_file(self, text: str, output_path: str | Path) -> bool:
        """
        Synthesise text to a WAV file without playing it.

        Returns True on success, False on failure.
        """
        text = self._clean_text(text)
        if not text or not self._available:
            return False

        output_path = Path(output_path)
        try:
            self._run_piper(text, str(output_path))
            logger.info(f"[TTS] Saved to: {output_path}")
            return True
        except Exception as e:
            logger.error(f"synthesize_to_file failed: {e}")
            return False

    @property
    def is_available(self) -> bool:
        """True if Piper binary and model are both present and functional."""
        return self._available

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _check_available(self) -> bool:
        """Verify that piper binary exists and the model file is present."""
        if not shutil.which(self._piper_bin) and not Path(self._piper_bin).is_file():
            return False
        if not self.model_path.exists():
            logger.debug(f"Piper model not found at: {self.model_path}")
            return False
        return True

    def _synthesize_and_play(self, text: str) -> None:
        """Run piper to a temp file, then play via sounddevice."""
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
            tmp_path = tmp.name

        try:
            self._run_piper(text, tmp_path)
            self._play_wav(tmp_path)
        finally:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass

    def _run_piper(self, text: str, output_path: str) -> None:
        """Call Piper via subprocess to generate a WAV file."""
        cmd = [
            self._piper_bin,
            "--model", str(self.model_path),
            "--output_file", output_path,
            "--speaker", str(self.speaker_id),
        ]
        proc = subprocess.run(
            cmd,
            input=text.encode("utf-8"),
            capture_output=True,
            timeout=30,
        )
        if proc.returncode != 0:
            err = proc.stderr.decode("utf-8", errors="replace")
            raise RuntimeError(f"Piper exited with code {proc.returncode}: {err}")

    def _play_wav(self, wav_path: str) -> None:
        """Play a WAV file through the default output device."""
        try:
            import sounddevice as sd  # type: ignore
            import soundfile as sf  # type: ignore
            data, fs = sf.read(wav_path)
            sd.play(data, fs)
            sd.wait()
        except ImportError:
            # Fallback: use os-level player
            if os.name == "nt":
                # Silent background playback (no windows!)
                os.system(f'powershell -c "$p = New-Object System.Media.SoundPlayer \'{wav_path}\'; $p.PlaySync()"')
            else:
                os.system(f"aplay {wav_path} 2>/dev/null || paplay {wav_path} 2>/dev/null")

    @staticmethod
    def _clean_text(text: str) -> str:
        """Remove markdown formatting and strip whitespace."""
        import re
        # Remove markdown bold/italic
        text = re.sub(r"[*_`#]", "", text)
        # Remove URLs
        text = re.sub(r"https?://\S+", "", text)
        # Collapse whitespace
        text = re.sub(r"\s+", " ", text).strip()
        return text


# ------------------------------------------------------------------
# Standalone test
# ------------------------------------------------------------------
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    engine = PiperEngine()
    if engine.is_available:
        engine.speak(
            "Neural uplink established. Archer online and monitoring the frequency, Boss."
        )
    else:
        print(
            "Piper not available. Install it from: "
            "https://github.com/rhasspy/piper/releases"
        )
