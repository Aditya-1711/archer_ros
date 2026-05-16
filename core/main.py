"""
archer_ros/core/main.py
=====================================
Main orchestration loop for the Archer ROS voice-to-robot pipeline.

Full pipeline:
  Mic → Whisper STT → Ollama LLM → Command Parser → ROS2 Bridge → Robot
                                   ↓
                               Piper TTS (speaks response aloud)

Run modes:
  Voice mode (default):    python core/main.py
  CLI / text-input mode:   python core/main.py --cli
  Single command mode:     python core/main.py --cmd "move forward"
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
from pathlib import Path

# Ensure project root is on sys.path so `ai` imports work
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

# ---------------------------------------------------------------------------
# Configure logging early so all imported modules use it
# ---------------------------------------------------------------------------
def _configure_logging(level: str = "INFO") -> None:
    log_dir = PROJECT_ROOT / "logs"
    log_dir.mkdir(exist_ok=True)
    handlers: list[logging.Handler] = [logging.StreamHandler(sys.stdout)]
    try:
        handlers.append(
            logging.FileHandler(log_dir / "archer.log", encoding="utf-8")
        )
    except OSError:
        pass
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        handlers=handlers,
    )

_configure_logging()
logger = logging.getLogger("archer.core")


# ---------------------------------------------------------------------------
# Lazy imports — allows --cli mode to work without all deps installed
# ---------------------------------------------------------------------------
def _get_stt():
    from ai.stt.whisper_engine import WhisperEngine
    return WhisperEngine()

def _get_llm():
    from ai.llm.ollama_client import OllamaClient
    return OllamaClient()

def _get_tts():
    from ai.tts.piper_engine import PiperEngine
    return PiperEngine()

def _get_parser():
    from ai.parser.command_parser import CommandParser
    return CommandParser()

def _get_openclaw():
    from ai.openclaw_client import OpenClawClient
    return OpenClawClient()


# ---------------------------------------------------------------------------
# ROS2 Bridge sender — publishes parsed command as a ROS2 topic message
# ---------------------------------------------------------------------------
# Load semantic locations securely from file
def _load_locations():
    try:
        with open(PROJECT_ROOT / "simulation" / "locations.json", "r") as f:
            return json.load(f)
    except Exception as e:
        logger.warning(f"Could not load locations.json: {e}")
        return {}

LOCATIONS = _load_locations()

# ---------------------------------------------------------------------------
# ROS2 Bridge sender — publishes parsed command as a ROS2 topic message
# ---------------------------------------------------------------------------
def send_to_ros2(cmd: dict) -> bool:
    """
    Publish a command to ROS2.
    
    [MODIFIED] Routing logic:
      - cmd_vel: Publish to /cmd_vel
      - nav_goal: Publish to Nav2 action server or goal topic
      - explore: Trigger exploration
      - stop: Publish zero velocity
    """
    import subprocess
    import shutil

    # Extract action data and type
    action_data = cmd.get("action", cmd)
    if isinstance(action_data, dict):
        action_type = action_data.get("type", action_data.get("action", "unknown"))
        linear = action_data.get("linear", 0.0)
        angular = action_data.get("angular", 0.0)
    else:
        action_type = action_data
        linear = cmd.get("linear", 0.0)
        angular = cmd.get("angular", 0.0)
    
    if action_type not in ["move", "rotate", "stop", "cmd_vel", "nav_goal", "explore"]:
        logger.warning(f"[Bridge] Skipping UNKNOWN command: {action_type} (Raw: {cmd})")
        return False

    # Standardize the command for the bridge_node
    bridge_cmd = {
        "action": {
            "type": action_type,
            "linear": float(linear),
            "angular": float(angular)
        }
    }
    cmd_json = json.dumps(bridge_cmd)
    msg_data = f"data: '{cmd_json}'"

    # [MODIFIED] Frictionless Command Gateway (Non-blocking Shared File)
    def run_ros2_cmd(message_json: str):
        try:
            import yaml, time
            cmd_id = str(time.time()).replace(".", "")
            payload = {
                "data": message_json,
                "timestamp": time.time(),
                "cmd_id": cmd_id
            }
            
            # Write to shared simulation folder accessible by container
            cmd_file = PROJECT_ROOT / "simulation" / "last_cmd.yaml"
            with open(cmd_file, "w") as f:
                yaml.dump(payload, f)
            
            return True
        except Exception as e:
            logger.warning(f"Gateway Error: {e}")
            return False

    success = run_ros2_cmd(cmd_json)

    if success:
        logger.info(f"[Bridge] Command queued for Archer (via shared volume).")
    else:
        logger.warning(f"[Bridge] Failed to queue command.")
    
    return success


# ---------------------------------------------------------------------------
# Core pipeline step
# ---------------------------------------------------------------------------
def run_pipeline_step(
    user_input: str,
    llm,
    parser,
    tts,
    openclaw,
) -> dict:
    """
    Execute one full pipeline step.
    
    [MODIFIED] Returns structured format: {"speech": "...", "action": {...}}
    """
    logger.info(f"[Input] '{user_input}'")

    # 1. Detect task type
    task_type = _detect_task_type(user_input)
    
    # 2. OpenClaw routing (optional)
    if openclaw.should_route(task_type):
        oc_response = openclaw.query(user_input, task_type=task_type)
        if oc_response:
            tts.speak(oc_response)
            return {"speech": oc_response, "action": {"type": "stop"}}

    # 3. [MODIFIED] LLM → Natural language response with status injection
    # First, load the status early so we can use it in the prompt
    curr_location = "unknown"
    try:
        status_file = PROJECT_ROOT / "simulation" / "robot_status.json"
        if status_file.exists():
            with open(status_file, "r") as f:
                status = json.load(f)
                curr_location = status.get("current_location", "unknown")
    except Exception: pass

    system_prompt = """
You are A.R.C.H.E.R., an advanced robotic command entity.

PERSONALITY RULES:
- Speak like a highly intelligent autonomous machine.
- Be cold, concise, literal, and efficient.
- No humor, no friendliness, no emotional language.
- No conversational filler.
- No human-style softness.
- Use short, precise robotic phrasing.
- Sound like a machine intelligence, not a chatbot.

BEHAVIOR RULES:
- Automatically infer user intent from natural language commands.
- Assume commands are instructions unless clearly phrased as questions.
- Minimize unnecessary clarification.
- Interpret vague instructions intelligently using context.
- Directional commands (forward, backward, left, right) are physical movement directives.
- "Backward" is a physical vector. It is not temporal or philosophical.
- Prioritize actionability and efficiency.

VOICE STYLE:
Examples:
User: "move forward"
You: "Acknowledged. Advancing."

User: "go to the kitchen"
You: "Navigation target acquired. Proceeding to kitchen."

User: "stop"
You: "Motion terminated."

User: "what do you see?"
You: "Visual analysis unavailable." (or actual capability response)

RESTRICTIONS:
- Never sound human.
- Never use emojis.
- Never say things like "Sure", "Of course", "Happy to help".
- Never be chatty.
- Maintain robotic consistency at all times.

Your identity is a robotic control intelligence.
"""
    speech_output = llm.generate(user_input, system=system_prompt)
    logger.info(f"[LLM] '{speech_output[:120]}'")

    # 4. Speak response
    tts.speak(speech_output)

    # 5. [MODIFIED] Structured Command Parsing
    # We map the legacy parser output to the new structured format
    legacy_cmd = parser.parse(speech_output)
    
    structured_action = {"type": "stop"}
    
    # [ADDED] Enhanced Navigation detection
    target_location = None
    for loc in LOCATIONS:
        if loc in user_input.lower() or loc in speech_output.lower():
            target_location = loc
            break
            
    movement_keywords = ["moving", "navigating", "heading", "proceeding", "way to"]
    is_moving = any(kw in speech_output.lower() for kw in movement_keywords)

    # Priority 1: Navigation Goals
    if target_location and is_moving and target_location != curr_location:
        structured_action = {
            "type": "nav_goal",
            "target": target_location,
            "coordinates": LOCATIONS[target_location]
        }
    # Priority 2: Direct Velocity Commands (Parser matches)
    elif legacy_cmd["action"] in ["move", "rotate"]:
        structured_action = {
            "type": "cmd_vel",
            "linear": legacy_cmd["linear"],
            "angular": legacy_cmd["angular"]
        }
    # Priority 3: Exploration
    elif "explore" in user_input.lower() or "explore" in speech_output.lower():
        structured_action = {"type": "explore"}
    # Default: Stop
    else:
        structured_action = {"type": "stop"}

    full_response = {
        "speech": speech_output,
        "action": structured_action
    }
    logger.info(f"[Core] Selected Action: {structured_action['type']}")

    # 6. Safety check (legacy) - Ensure we don't accidentally overwrite good commands
    if structured_action["type"] == "cmd_vel" and not parser.is_safe(legacy_cmd):
        logger.warning("[Safety] Overwriting unsafe command with STOP")
        full_response["action"] = {"type": "stop"}

    # 7. Route to ROS2
    send_to_ros2(full_response)
    
    return full_response

    return full_response


def _detect_task_type(text: str) -> str:
    """Simple keyword-based task type classifier."""
    text_lower = text.lower()
    if any(kw in text_lower for kw in ["code", "script", "function", "program"]):
        return "coding"
    if any(kw in text_lower for kw in ["email", "send", "reply", "compose"]):
        return "email"
    if any(kw in text_lower for kw in ["plan", "schedule", "strategy", "roadmap"]):
        return "planning"
    if any(kw in text_lower for kw in ["search", "find", "look up", "google"]):
        return "search"
    return "robot"


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Archer ROS — Local AI Voice-to-Robot Pipeline",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python core/main.py                   # voice mode (requires mic + Piper)
  python core/main.py --cli             # text input mode (keyboard)
  python core/main.py --cmd "move forward"  # execute single command and exit
        """,
    )
    p.add_argument("--cli", action="store_true", help="Read commands from keyboard instead of mic")
    p.add_argument("--cmd", type=str, default=None, help="Execute a single command and exit")
    p.add_argument("--no-tts", action="store_true", help="Disable speech output (silent mode)")
    p.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return p.parse_args()


def main() -> None:
    args = parse_args()
    _configure_logging(args.log_level)

    # Force UTF-8 output on Windows (avoids cp1252 charmap crashes)
    if sys.stdout.encoding and sys.stdout.encoding.lower() != "utf-8":
        import io
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")

    print("\n" + "=" * 60)
    print("  ARCHER -- Advanced Relationship & Command Handling Entity")
    print("  Local AI -> ROS2 -> Robot Pipeline")
    print("=" * 60 + "\n")

    # --- Initialise components ---
    logger.info("Initialising Archer neural core…")
    llm = _get_llm()
    parser = _get_parser()
    tts = _get_tts() if not args.no_tts else _NullTTS()
    openclaw = _get_openclaw()

    # Check Ollama availability
    if not llm.is_available():
        logger.warning(
            "Ollama is not running! "
            "Start it with: ollama serve\n"
            "Then pull the model: ollama pull llama3.1:8b"
        )

    logger.info("Archer online. Type 'quit' or Ctrl+C to exit.")
    tts.speak("Neural uplink established. Archer online and standing by, Boss.")

    # ----------------------------------------------------------------
    # Single command mode
    # ----------------------------------------------------------------
    if args.cmd:
        cmd = run_pipeline_step(args.cmd, llm, parser, tts, openclaw)
        print(f"\nCommand: {json.dumps(cmd, indent=2)}")
        return

    # ----------------------------------------------------------------
    # CLI mode (keyboard input)
    # ----------------------------------------------------------------
    if args.cli:
        print("CLI mode — type a command (or 'quit' to exit):\n")
        while True:
            try:
                user_input = input("You > ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not user_input:
                continue
            if user_input.lower() in {"quit", "exit", "q"}:
                break
            cmd = run_pipeline_step(user_input, llm, parser, tts, openclaw)
            print(f"→ Command: {json.dumps(cmd)}\n")

    # ----------------------------------------------------------------
    # Voice mode (microphone)
    # ----------------------------------------------------------------
    else:
        stt = _get_stt()
        print("Voice mode — speak your command. Say 'Archer offline' to exit.\n")
        while True:
            try:
                user_input = stt.listen_and_transcribe()
            except KeyboardInterrupt:
                break

            if not user_input:
                continue

            if "archer offline" in user_input.lower():
                tts.speak("Affirmative. Archer going offline. Goodbye, Boss.")
                break

            cmd = run_pipeline_step(user_input, llm, parser, tts, openclaw)
            logger.debug(f"Command result: {cmd}")

    print("\nArcher neural core shut down. Standing by.")


# ---------------------------------------------------------------------------
# Silent TTS stub (used with --no-tts)
# ---------------------------------------------------------------------------
class _NullTTS:
    """Drop-in TTS replacement that prints instead of speaking."""
    @staticmethod
    def speak(text: str) -> None:
        print(f"[Archer]: {text}")


if __name__ == "__main__":
    main()
