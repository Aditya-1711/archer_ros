"""
archer_ros/ai/llm/ollama_client.py
=====================================
Ultra-Stable LLM client for Archer.
"""

import json
import logging
import re
import requests
from typing import Optional
from pathlib import Path

logger = logging.getLogger("archer.llm")

def _load_ollama_config() -> dict:
    try:
        import yaml  # type: ignore
        cfg_path = Path(__file__).parent.parent.parent / "config" / "settings.yaml"
        with open(cfg_path, encoding="utf-8") as f:
            return yaml.safe_load(f).get("ollama", {})
    except Exception:
        return {}

class OllamaClient:
    def __init__(self, base_url: Optional[str] = None, model: Optional[str] = None):
        cfg = _load_ollama_config()
        self.base_url = (base_url or cfg.get("base_url", "http://localhost:11434")).rstrip("/")
        self.model = model or cfg.get("model", "llama3.2:1b")
        self.timeout = int(cfg.get("timeout", 120))

    def is_available(self) -> bool:
        try:
            return requests.get(f"{self.base_url}/api/tags", timeout=5).status_code == 200
        except:
            return False

    def generate_json(self, prompt: str) -> dict:
        """Helper to ensure the LLM returns valid JSON."""
        resp_text = self.generate(prompt)
        try:
            # Try to find JSON block in the response
            match = re.search(r'\{.*\}', resp_text, re.DOTALL)
            if match:
                return json.loads(match.group(0))
            return json.loads(resp_text)
        except Exception as e:
            logger.warning(f"JSON Parse failed: {e}. Raw: {resp_text}")
            return {"error": "parse_failure", "raw": resp_text}

    def generate(self, prompt: str, system: Optional[str] = None) -> str:
        try:
            # Using /api/chat instead of /api/generate for better stability
            system_content = system or """
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
            payload = {
                "model": self.model,
                "messages": [
                    {"role": "system", "content": system_content},
                    {"role": "user", "content": prompt}
                ],
                "stream": False,
                "options": {"temperature": 0.0}
            }

            resp = requests.post(f"{self.base_url}/api/chat", json=payload, timeout=self.timeout)
            
            if resp.status_code != 200:
                logger.warning(f"Ollama Chat error {resp.status_code}. Fallback to raw.")
                return prompt

            return resp.json().get("message", {}).get("content", prompt).strip()

        except Exception as e:
            logger.error(f"Ollama Error: {e}")
            return prompt
