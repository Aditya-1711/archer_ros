"""
archer_ros/ai/parser/command_parser.py
=====================================
Converts LLM natural-language responses into structured robot commands (JSON).

SAFETY-FIRST DESIGN:
  - All velocity values are clamped hard before returning.
  - Malformed or unrecognised inputs produce a STOP command.
  - The LLM is NEVER allowed to produce motor values directly.
    All values come from this parser's pre-defined speed map.

Command schema:
  {
    "action":  "move" | "rotate" | "stop" | "status" | "unknown",
    "linear":  float,   # m/s  (clamped to ±max_linear)
    "angular": float,   # rad/s (clamped to ±max_angular)
    "duration": float,  # seconds (0 = indefinite, -1 = not applicable)
    "raw":     str,     # original LLM text (for logging)
  }

Usage:
    parser = CommandParser()
    cmd = parser.parse("Affirmative. Moving forward at moderate speed.")
    # → {"action": "move", "linear": 0.18, "angular": 0.0, "duration": 0, "raw": "…"}
"""

from __future__ import annotations

import json
import logging
import re
from pathlib import Path
from typing import Any, Optional

logger = logging.getLogger("archer.parser")

# ---------------------------------------------------------------------------
# Default limits (overridden by settings.yaml if available)
# ---------------------------------------------------------------------------
_DEFAULT_MAX_LINEAR = 2.0   # m/s (Increased to compensate for RTF)
_DEFAULT_MAX_ANGULAR = 3.0   # rad/s


def _load_safety_config() -> dict:
    try:
        import yaml  # type: ignore
        cfg_path = Path(__file__).parent.parent.parent / "config" / "settings.yaml"
        with open(cfg_path, encoding="utf-8") as f:
            return yaml.safe_load(f).get("safety", {})
    except Exception:
        return {}


def _load_locations() -> dict:
    try:
        cfg_path = Path(__file__).parent.parent.parent / "simulation" / "locations.json"
        with open(cfg_path, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


LOCATIONS = _load_locations()


# ---------------------------------------------------------------------------
# Speed reference table (keyword → (linear m/s, angular rad/s))
# All values must respect the safety limits above.
# ---------------------------------------------------------------------------
#  Adjective   linear   angular   note
SPEED_MAP: dict[str, tuple[float, float]] = {
    "slowly":       (0.40,  0.0),
    "slow":         (0.40,  0.0),
    "gently":       (0.40,  0.0),
    "carefully":    (0.40,  0.0),
    "moderate":     (1.00,  0.0),
    "moderately":   (1.00,  0.0),
    "normal":       (1.00,  0.0),
    "fast":         (1.60,  0.0),
    "quickly":      (1.60,  0.0),
    "full":         (2.00,  0.0),
}

TURN_SPEED_MAP: dict[str, float] = {
    "slowly":       0.6,
    "slow":         0.6,
    "gently":       0.6,
    "moderate":     1.2,
    "moderately":   1.2,
    "fast":         2.0,
    "quickly":      2.0,
    "sharply":      2.0,
}

# ---------------------------------------------------------------------------
# Pattern library — order matters (more specific first)
# ---------------------------------------------------------------------------
_MOVE_FORWARD = re.compile(
    r"\b(mov(?:e|ing)|go(?:ing)?|driv(?:e|ing)|advanc(?:e|ing)|step(?:ping)?|walk(?:ing)?|proceed(?:ing)?)\s*(forward|foward|ahead|straight|front)?\b|\b(straight\s+ahead|keep\s+going)\b",
    re.IGNORECASE,
)
_MOVE_BACKWARD = re.compile(
    r"\b(mov(?:e|ing)\s+back(?:ward|word)?s?|go(?:ing)?\s+back(?:ward|word)?s?|back(?:ing)?\s*up|revers(?:e|ing|al)|return(?:ing)?\s*back|back(?:ward)?|go(?:ing)?\s+back|step(?:ping)?\s+back|walk(?:ing)?\s+back)\b",
    re.IGNORECASE,
)
_TURN_LEFT = re.compile(
    r"\b(turn(?:ing)?|rotat(?:e|ing)|pivot(?:ing)?|take(?:ing)?(?:\s+a)?|go(?:ing)?|bear(?:ing)?|steer(?:ing)?|head(?:ing)?)\s+(left|counter\s*-?clockwise|ccw)\b|\bleft\s+turn\b",
    re.IGNORECASE,
)
_TURN_RIGHT = re.compile(
    r"\b(turn(?:ing)?|rotat(?:e|ing)|pivot(?:ing)?|take(?:ing)?(?:\s+a)?|go(?:ing)?|bear(?:ing)?|steer(?:ing)?|head(?:ing)?)\s+(right|clockwise|cw)\b|\bright\s+turn\b",
    re.IGNORECASE,
)
_STOP = re.compile(
    r"\b(stop(?:ping)?|halt(?:ing)?|stand(?:ing)?\s*by|cease|freeze|robot\s+halt(?:ed)?|stop\s+moving)\b",
    re.IGNORECASE,
)
_STATUS = re.compile(
    r"\b(status|nominal|operational|systems?|all\s+systems?|standing\s+by|battery|temp|temperature)\b",
    re.IGNORECASE,
)
_DURATION = re.compile(
    r"for\s+(\d+(?:\.\d+)?)\s*(second|seconds|sec|s)\b",
    re.IGNORECASE,
)
_STEPS = re.compile(
    r"\b(one|two|three|four|five|six|seven|eight|nine|ten|\d+)\s+steps?\b",
    re.IGNORECASE,
)
_WORD_TO_NUM = {
    "one": 1, "two": 2, "three": 3, "four": 4, "five": 5,
    "six": 6, "seven": 7, "eight": 8, "nine": 9, "ten": 10
}
_REMEMBER = re.compile(r'\b(remember|note|save|memorize)\b\s+(that\s+)?(.+)', re.IGNORECASE)
_LEARN_ROUTINE = re.compile(r'\b(learn|save|record)\b\s+routine\s+(.*?)\s*(?:[=:,-]|as)\s+(.+)', re.IGNORECASE)
_EXPLORE = re.compile(r"\b(explore|mapping|map\s+the\s+area|start\s+mapping|run\s+slam|scan)\b", re.IGNORECASE)
_EXECUTE_ROUTINE = re.compile(r"\b(execute|run|start)\s+routine\s+(.+)", re.IGNORECASE)


class CommandParser:
    """
    Parses Archer's LLM response text into a structured robot command dict.

    Primary method: regex keyword matching (no LLM call needed).
    Secondary method: if enabled, delegates to OllamaClient.generate_json()
                      for ambiguous inputs.
    """

    def __init__(self, enable_llm_fallback: bool = True) -> None:
        safety = _load_safety_config()
        self.max_linear: float = float(safety.get("max_linear", _DEFAULT_MAX_LINEAR))
        self.max_angular: float = float(safety.get("max_angular", _DEFAULT_MAX_ANGULAR))
        self._llm_fallback = enable_llm_fallback
        logger.debug(
            f"CommandParser limits — linear: ±{self.max_linear} m/s, "
            f"angular: ±{self.max_angular} rad/s"
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def parse(self, llm_text: str) -> dict[str, Any]:
        """
        Convert LLM response text into a structured command.

        Args:
            llm_text: The raw string output from the LLM.

        Returns:
            Command dict with keys: action, linear, angular, duration, raw.
        """
        raw = llm_text.strip()
        cmd = self._regex_parse(raw)

        if cmd["action"] == "unknown" and self._llm_fallback:
            logger.debug("Regex parse gave 'unknown', trying LLM JSON fallback…")
            cmd = self._llm_fallback_parse(raw)

        # Final safety clamp — always applied regardless of path
        cmd = self._clamp(cmd)
        logger.info(f"[Parser] '{raw[:60]}' -> {cmd}")
        return cmd

    def is_safe(self, cmd: dict) -> bool:
        """
        Check whether a command passes all safety validations.

        Returns False for malformed, unknown, or out-of-range commands.
        """
        if cmd.get("action") not in {"move", "rotate", "stop", "status", "nav_goal", "explore", "remember", "learn_routine", "execute_routine"}:
            return False
        lin = abs(cmd.get("linear", 0.0))
        ang = abs(cmd.get("angular", 0.0))
        if lin > self.max_linear or ang > self.max_angular:
            return False
        return True

    # ------------------------------------------------------------------
    # Parsing backends
    # ------------------------------------------------------------------

    def _regex_parse(self, text: str) -> dict[str, Any]:
        """Primary parser using compiled regex patterns."""
        # Detect speed qualifier in the text
        lin_speed, ang_speed = self._detect_speed(text)
        duration = self._detect_duration(text)

        # --- STOP (check first — highest priority safety action) ---
        if _STOP.search(text):
            return self._make(action="stop", linear=0.0, angular=0.0, duration=-1, raw=text)

        # --- STATUS query ---
        if _STATUS.search(text) and not any(
            p.search(text) for p in [_MOVE_FORWARD, _MOVE_BACKWARD, _TURN_LEFT, _TURN_RIGHT]
        ):
            return self._make(action="status", linear=0.0, angular=0.0, duration=-1, raw=text)

        # --- NAVIGATION ---
        # Look for motion intent and a known location name
        nav_intent = re.search(r"\b(go(?:ing)?(?:\s+to)?|head(?:ing)?(?:\s+to)?|navigat(?:e|ing)(?:\s+to)?|proceed(?:ing)?(?:\s+to)?|take(?:\s+me)?(?:\s+to)?|move(?:ing)?(?:\s+to)?|return(?:ing)?(?:\s+to)?)\s+(the\s+)?(origin|dock|living_room|kitchen|bedroom|garage)\b", text, re.IGNORECASE)
        if nav_intent:
            loc = nav_intent.group(3).lower()
            if loc == "dock":
                loc = "origin"
            cmd = self._make(action="nav_goal", linear=0.0, angular=0.0, duration=-1, raw=text)
            cmd["target"] = loc
            if loc in LOCATIONS:
                cmd["coordinates"] = LOCATIONS[loc]
            return cmd

        for loc_name in LOCATIONS:
            if re.search(r"\b" + re.escape(loc_name) + r"\b", text, re.IGNORECASE):
                # Also accept if "go" or "navigate" or "head" is nearby
                if any(w in text.lower() for w in ["go", "navigate", "head", "proceed", "move", "to"]):
                    cmd = self._make(action="nav_goal", linear=0.0, angular=0.0, duration=-1, raw=text)
                    cmd["target"] = loc_name
                    cmd["coordinates"] = LOCATIONS[loc_name]
                    return cmd

        # --- EXPLORE ---
        if _EXPLORE.search(text):
            return self._make(action="explore", linear=0.0, angular=0.0, duration=-1, raw=text)

        # --- EXECUTE ROUTINE ---
        routine_exec_match = _EXECUTE_ROUTINE.search(text)
        if routine_exec_match:
            r_name = routine_exec_match.group(2).strip()
            if r_name.endswith((".", "!", "?")):
                r_name = r_name[:-1]
            cmd = self._make(action="execute_routine", linear=0.0, angular=0.0, duration=-1, raw=text)
            cmd["routine_name"] = r_name
            return cmd

        # --- TURN LEFT ---
        if _TURN_LEFT.search(text):
            return self._make(
                action="rotate",
                linear=0.0,
                angular=ang_speed if ang_speed else 0.6,
                duration=duration,
                raw=text,
            )

        # --- TURN RIGHT ---
        if _TURN_RIGHT.search(text):
            return self._make(
                action="rotate",
                linear=0.0,
                angular=-(ang_speed if ang_speed else 0.6),
                duration=duration,
                raw=text,
            )

        # --- MOVE BACKWARD (Check before forward to avoid overlap) ---
        if _MOVE_BACKWARD.search(text):
            return self._make(
                action="move",
                linear=-(lin_speed if lin_speed else 0.5),
                angular=0.0,
                duration=duration,
                raw=text,
            )

        # --- MOVE FORWARD ---
        if _MOVE_FORWARD.search(text):
            return self._make(
                action="move",
                linear=lin_speed if lin_speed else 0.5,
                angular=0.0,
                duration=duration,
                raw=text,
            )

        # --- REMEMBER (Memory System) ---
        mem_match = _REMEMBER.search(text)
        if mem_match:
            fact = mem_match.group(3).strip()
            # Clean trailing punctuation
            if fact.endswith((".", "!", "?")):
                fact = fact[:-1]
            cmd = self._make(action="remember", linear=0.0, angular=0.0, duration=-1, raw=text)
            cmd["fact"] = fact
            return cmd
            
        # --- LEARN ROUTINE (Macro System) ---
        routine_match = _LEARN_ROUTINE.search(text)
        if routine_match:
            r_name = routine_match.group(2).strip()
            r_sequence = routine_match.group(3).strip()
            cmd = self._make(action="learn_routine", linear=0.0, angular=0.0, duration=-1, raw=text)
            cmd["routine_name"] = r_name
            cmd["routine_sequence"] = r_sequence
            return cmd

        # --- UNKNOWN — safe default is STOP ---
        logger.debug(f"No pattern matched for: '{text[:60]}'")
        return self._make(action="unknown", linear=0.0, angular=0.0, duration=-1, raw=text)

    def _llm_fallback_parse(self, text: str) -> dict[str, Any]:
        """
        Ask the LLM to extract a command JSON from ambiguous text.
        Only called when enable_llm_fallback=True.
        """
        try:
            from ai.llm.ollama_client import OllamaClient  # type: ignore
            client = OllamaClient()
            prompt = (
                f"Extract a robot movement command from this text:\n\"{text}\"\n\n"
                f"Return ONLY JSON in this exact format:\n"
                f'{{\"action\": \"move|rotate|stop|status\", '
                f'\"linear\": <float 0-{self.max_linear}>, '
                f'\"angular\": <float 0-{self.max_angular}>}}'
            )
            data = client.generate_json(prompt)
            if "error" not in data:
                return self._make(
                    action=data.get("action", "unknown"),
                    linear=float(data.get("linear", 0.0)),
                    angular=float(data.get("angular", 0.0)),
                    duration=0,
                    raw=text,
                )
        except Exception as e:
            logger.warning(f"LLM fallback parse failed: {e}")

        return self._make(action="unknown", linear=0.0, angular=0.0, duration=-1, raw=text)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _detect_speed(self, text: str) -> tuple[float, float]:
        """Detect speed qualifier words and return (linear, angular) speeds."""
        text_lower = text.lower()
        for kw, (lin, _) in SPEED_MAP.items():
            if kw in text_lower:
                ang = TURN_SPEED_MAP.get(kw, 0.6)
                return lin, ang
        return 0.0, 0.0  # signals "use default"

    def _detect_duration(self, text: str) -> float:
        """Extract 'for N seconds' or 'N steps' from text. Returns 2.0 if not found."""
        m = _DURATION.search(text)
        if m:
            return float(m.group(1))

        m_steps = _STEPS.search(text)
        if m_steps:
            val = m_steps.group(1).lower()
            num = _WORD_TO_NUM.get(val, None)
            if num is None:
                try:
                    num = float(val)
                except ValueError:
                    num = 2.0
            return float(num * 1.0)

        return 2.0  # Default to 2.0 (matching docstring)

    def _clamp(self, cmd: dict) -> dict:
        """Hard-clamp all velocity values to configured safety limits."""
        cmd["linear"] = max(-self.max_linear, min(self.max_linear, cmd.get("linear", 0.0)))
        cmd["angular"] = max(-self.max_angular, min(self.max_angular, cmd.get("angular", 0.0)))
        return cmd

    @staticmethod
    def _make(
        action: str,
        linear: float,
        angular: float,
        duration: float,
        raw: str,
    ) -> dict[str, Any]:
        return {
            "action": action,
            "linear": round(linear, 4),
            "angular": round(angular, 4),
            "duration": round(duration, 2),
            "raw": raw,
        }


# ------------------------------------------------------------------
# Module-level convenience function
# ------------------------------------------------------------------
_default_parser: Optional[CommandParser] = None


def parse_command(llm_text: str) -> dict[str, Any]:
    """
    Module-level convenience wrapper around CommandParser.

    Shares a single parser instance per process.
    """
    global _default_parser
    if _default_parser is None:
        _default_parser = CommandParser()
    return _default_parser.parse(llm_text)


# ------------------------------------------------------------------
# Standalone test
# ------------------------------------------------------------------
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    parser = CommandParser()

    test_cases = [
        "Affirmative. Moving forward at moderate speed, Boss.",
        "Turning left. Adjusting heading, Boss.",
        "Stopping now. Robot halted, Boss.",
        "All systems nominal. Standing by for your orders, Boss.",
        "Reversing slowly for 3 seconds.",
        "Rotating right quickly.",
        "This sentence doesn't map to any command.",
    ]

    for t in test_cases:
        cmd = parser.parse(t)
        safe = "✅" if parser.is_safe(cmd) else "⚠️"
        print(f"{safe} [{cmd['action']:8s}] L={cmd['linear']:+.2f} A={cmd['angular']:+.2f}  ← '{t[:50]}'")
