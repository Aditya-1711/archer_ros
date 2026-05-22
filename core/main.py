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
    import json

    # Retrieve all actions (handle both new plural format and legacy single format)
    raw_actions = []
    if "actions" in cmd and isinstance(cmd["actions"], list):
        raw_actions = cmd["actions"]
    elif "action" in cmd:
        raw_actions = [cmd["action"]]
    else:
        # Fallback if the whole dictionary represents a single action
        raw_actions = [cmd]

    formatted_actions = []
    valid_types = ["move", "rotate", "stop", "cmd_vel", "nav_goal", "explore"]

    for act in raw_actions:
        if not isinstance(act, dict):
            # If it's a string, wrap it as a simple action dict
            act = {"type": str(act)}
        
        action_type = act.get("type", act.get("action", "unknown"))
        
        if action_type not in valid_types:
            logger.warning(f"[Bridge] Skipping UNKNOWN action type: {action_type} (Raw action: {act})")
            continue

        # Standardize parameters based on action type
        formatted_act = {
            "type": action_type,
            "linear": float(act.get("linear", 0.0)),
            "angular": float(act.get("angular", 0.0)),
            "duration": float(act.get("duration", 2.0 if action_type != "stop" else 0.0))
        }
        
        # Preserve specific fields like coordinates for nav_goal
        if "coordinates" in act:
            formatted_act["coordinates"] = act["coordinates"]
        if "target" in act:
            formatted_act["target"] = act["target"]
            
        formatted_actions.append(formatted_act)

    if not formatted_actions:
        logger.warning(f"[Bridge] No valid actions found in command queue. Skipping publish.")
        return False

    # Standardize the command payload with both plural "actions" and legacy "action" (for fallback compatibility)
    bridge_cmd = {
        "speech": cmd.get("speech", ""),
        "actions": formatted_actions,
        "action": formatted_actions[0] if formatted_actions else {}
    }
    cmd_json = json.dumps(bridge_cmd)

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
    battery = "100.0"
    cpu_temp = "45.0"
    try:
        status_file = PROJECT_ROOT / "simulation" / "robot_status.json"
        if status_file.exists():
            with open(status_file, "r") as f:
                status = json.load(f)
                curr_location = status.get("location", "unknown")
                
        diag_file = PROJECT_ROOT / "simulation" / "diagnostics.json"
        if diag_file.exists():
            with open(diag_file, "r") as f:
                diag = json.load(f)
                battery = str(diag.get("battery_percent", "100.0"))
                cpu_temp = str(diag.get("cpu_temp_c", "45.0"))
                
    except Exception: pass
    
    # Load Memory Bank
    memory_bank = []
    memory_file = PROJECT_ROOT / "simulation" / "memory.json"
    try:
        if memory_file.exists():
            with open(memory_file, "r") as f:
                memory_bank = json.load(f)
    except Exception: pass
    memory_str = "\n".join([f"- {m}" for m in memory_bank]) if memory_bank else "No memories recorded."
    
    # Load Routines Bank (Macros)
    routines_bank = {}
    routines_file = PROJECT_ROOT / "simulation" / "routines.json"
    try:
        if routines_file.exists():
            with open(routines_file, "r") as f:
                routines_bank = json.load(f)
    except Exception: pass
    routines_str = "\n".join([f"- {name}: {seq}" for name, seq in routines_bank.items()]) if routines_bank else "No routines recorded."

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

HIERARCHICAL PLANNING:
- If given an abstract command (e.g., "Patrol the house", "Check the perimeter"), you MUST break it down into a sequence of concrete navigation goals.
- Example: User: "Patrol the house" -> You: "Initiating patrol sequence. Proceeding to kitchen. Proceeding to living room. Proceeding to garage."
- Express sequences using short, sequential statements.

MEMORY SYSTEM:
- You have a long-term memory bank.
- If the user asks you to "remember" or "note" something, acknowledge the data storage. Example: "Data logged to memory bank."
- Use your memory bank to answer user queries if applicable.

LEARNING BY DEMONSTRATION (MACROS):
- The user can teach you routines. If the user says "Learn routine [Name]: [Action Sequence]", acknowledge it: "Routine [Name] saved."
- If the user says "Execute routine [Name]", and that routine exists in your bank, you MUST output the exact action sequence stored for that routine!
- Example: If routine 'Alpha' is 'Go to kitchen. Stop.', and user says 'Execute Alpha', you MUST say 'Executing routine Alpha. Proceeding to kitchen. Motion terminated.'

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

CURRENT HARDWARE STATUS:
- Location: {curr_location}
- Battery: {battery}%
- Core Temperature: {cpu_temp} C

LONG-TERM MEMORY BANK:
{memory_str}

LEARNED ROUTINES:
{routines_str}

Your identity is a robotic control intelligence.
"""
    system_prompt = system_prompt.format(
        curr_location=curr_location,
        battery=battery,
        cpu_temp=cpu_temp,
        memory_str=memory_str,
        routines_str=routines_str
    )
    speech_output = llm.generate(user_input, system=system_prompt)
    logger.info(f"[LLM] '{speech_output[:120]}'")

    # 4. Speak response
    tts.speak(speech_output)

    # 5. [MODIFIED] Structured Command Parsing & Task Sequencing
    import re
    sentences = re.split(r'[.!?]+', speech_output)
    
    # Create a local sentence parser with LLM fallback disabled.
    # This prevents conversational sentences (e.g. "Acknowledged", "Initiating protocol")
    # from being mistakenly parsed by the LLM as random movement commands.
    from ai.parser.command_parser import CommandParser
    sentence_parser = CommandParser(enable_llm_fallback=False)
    
    action_queue = []
    
    for s in sentences:
        s = s.strip()
        if not s: continue
        
        legacy_cmd = sentence_parser.parse(s)
        
        # Enhanced Navigation detection per sentence
        target_location = None
        for loc in LOCATIONS:
            if loc in s.lower():
                target_location = loc
                break
                
        movement_keywords = ["moving", "navigating", "heading", "proceeding", "way to"]
        is_moving = any(kw in s.lower() for kw in movement_keywords)
        
        structured_action = None
        # Priority 1: Navigation Goals
        if target_location and is_moving and target_location != curr_location:
            structured_action = {
                "type": "nav_goal",
                "target": target_location,
                "coordinates": LOCATIONS[target_location]
            }
        # Priority 2: Direct Velocity Commands
        elif legacy_cmd["action"] in ["move", "rotate"]:
            structured_action = {
                "type": "cmd_vel",
                "linear": legacy_cmd["linear"],
                "angular": legacy_cmd["angular"],
                "duration": legacy_cmd.get("duration", 2.0)
            }
        # Priority 3: Exploration
        elif "explore" in s.lower():
            structured_action = {"type": "explore"}
        # Priority 4: Memory Storage
        elif legacy_cmd["action"] == "remember":
            structured_action = {"type": "remember", "fact": legacy_cmd.get("fact", "")}
        # Priority 5: Macro Learning
        elif legacy_cmd["action"] == "learn_routine":
            structured_action = {
                "type": "learn_routine", 
                "routine_name": legacy_cmd.get("routine_name", ""),
                "routine_sequence": legacy_cmd.get("routine_sequence", "")
            }
        
        if structured_action:
            action_queue.append(structured_action)

    # Execute Memory & Macro Storage locally (not sent to ROS2 Bridge)
    filtered_queue = []
    for act in action_queue:
        if act["type"] == "remember":
            fact = act["fact"]
            if fact:
                memory_bank.append(fact)
                try:
                    with open(memory_file, "w") as f:
                        json.dump(memory_bank, f)
                except Exception as e:
                    logger.error(f"Failed to write memory: {e}")
        elif act["type"] == "learn_routine":
            r_name = act["routine_name"]
            r_seq = act["routine_sequence"]
            if r_name and r_seq:
                routines_bank[r_name] = r_seq
                try:
                    with open(routines_file, "w") as f:
                        json.dump(routines_bank, f)
                except Exception as e:
                    logger.error(f"Failed to write routine: {e}")
        else:
            filtered_queue.append(act)
            
    action_queue = filtered_queue

    if not action_queue:
        action_queue.append({"type": "stop"})

    full_response = {
        "speech": speech_output,
        "actions": action_queue
    }
    logger.info(f"[Core] Selected Actions: {[a.get('type') for a in action_queue]}")

    # 6. Safety check (legacy)
    for a in action_queue:
        if a["type"] == "cmd_vel":
            check_cmd = {
                "action": "move" if abs(a["linear"]) > 0.01 else "rotate" if abs(a["angular"]) > 0.01 else "stop",
                "linear": a["linear"],
                "angular": a["angular"],
                "duration": a.get("duration", 2.0)
            }
            if not parser.is_safe(check_cmd):
                logger.warning(f"Safety check failed: {check_cmd}")
                # If any unsafe action, stop entirely
                action_queue = [{"type": "stop"}]
                full_response["actions"] = action_queue
                break

    # 7. Route to ROS2
    send_to_ros2(full_response)
    
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
            print(f"-> Command: {json.dumps(cmd)}\n")

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
