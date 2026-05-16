"""
archer_ros/ai/openclaw_client.py
=====================================
Optional OpenClaw integration for complex task routing.

DISABLED BY DEFAULT — set `openclaw.enabled: true` in config/settings.yaml
to activate. Robot control tasks are NEVER routed to OpenClaw regardless
of this setting.

OpenClaw handles: coding, planning, email, search, and complex reasoning.
Robot motion commands always go through the local ROS2 bridge only.

Usage (when enabled):
    client = OpenClawClient()
    if client.is_enabled and client.is_available():
        response = client.query("Write a path planning algorithm for the robot")
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional

import requests

logger = logging.getLogger("archer.openclaw")

# Task types that must NEVER be routed to OpenClaw
ROBOT_SAFETY_ACTIONS = {"move", "rotate", "stop", "navigate", "cmd_vel"}


def _load_config() -> dict:
    """Load openclaw section from settings.yaml."""
    try:
        import yaml  # type: ignore
        cfg_path = Path(__file__).parent.parent / "config" / "settings.yaml"
        with open(cfg_path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        return cfg.get("openclaw", {})
    except Exception as e:
        logger.warning(f"Could not load settings.yaml: {e}. OpenClaw disabled.")
        return {}


class OpenClawClient:
    """
    Thin HTTP client for the OpenClaw AI gateway.

    When openclaw.enabled is false (default), all methods are no-ops
    and the system works identically without OpenClaw installed.
    """

    def __init__(self) -> None:
        cfg = _load_config()
        self.is_enabled: bool = bool(cfg.get("enabled", False))
        self.url: str = cfg.get("url", "http://localhost:33635").rstrip("/")
        self.timeout: int = int(cfg.get("request_timeout", 15))
        self.task_routes: list[str] = cfg.get("task_routes", [])

        if self.is_enabled:
            logger.info(f"OpenClawClient enabled — endpoint: '{self.url}'")
        else:
            logger.debug(
                "OpenClaw is DISABLED (openclaw.enabled: false in settings.yaml). "
                "All tasks handled by local Ollama."
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def is_available(self) -> bool:
        """
        Probe whether the OpenClaw gateway is reachable.

        Always returns False when openclaw.enabled is false.
        """
        if not self.is_enabled:
            return False
        try:
            resp = requests.get(f"{self.url}/health", timeout=3)
            return resp.status_code == 200
        except requests.exceptions.RequestException:
            return False

    def should_route(self, task_type: str) -> bool:
        """
        Decide whether a task type should be sent to OpenClaw.

        Robot safety actions are NEVER routed regardless of config.

        Args:
            task_type: A category string, e.g. "coding", "planning", "move".

        Returns:
            True if OpenClaw is enabled, available, and the task is configured
            for routing. False in all other cases.
        """
        if not self.is_enabled:
            return False
        if task_type.lower() in ROBOT_SAFETY_ACTIONS:
            logger.debug(f"Task type '{task_type}' is a robot action — will NOT route to OpenClaw.")
            return False
        return task_type.lower() in [r.lower() for r in self.task_routes]

    def query(self, task: str, task_type: str = "general") -> Optional[str]:
        """
        Send a task to OpenClaw and return its response.

        Args:
            task:      The natural-language task description.
            task_type: Category hint (e.g. "coding", "planning").

        Returns:
            Response text, or None if unavailable or disabled.
        """
        if not self.is_enabled:
            logger.debug("OpenClaw query skipped — disabled.")
            return None

        if task_type in ROBOT_SAFETY_ACTIONS:
            logger.warning(
                f"Blocked attempt to send robot action '{task_type}' to OpenClaw. "
                "Robot control is handled exclusively by the local ROS2 bridge."
            )
            return None

        if not self.is_available():
            logger.warning("OpenClaw gateway is not reachable. Falling back to local LLM.")
            return None

        logger.info(f"[OpenClaw] Routing task_type='{task_type}' to OpenClaw: '{task[:60]}'")

        try:
            resp = requests.post(
                f"{self.url}/query",
                json={"task": task, "type": task_type},
                timeout=self.timeout,
            )
            resp.raise_for_status()
            result: str = resp.json().get("response", "").strip()
            logger.debug(f"[OpenClaw] Response: '{result[:120]}'")
            return result
        except requests.exceptions.Timeout:
            logger.error("OpenClaw query timed out.")
            return None
        except requests.exceptions.RequestException as e:
            logger.error(f"OpenClaw request failed: {e}")
            return None


# ------------------------------------------------------------------
# Standalone test
# ------------------------------------------------------------------
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    client = OpenClawClient()

    print(f"OpenClaw enabled: {client.is_enabled}")
    print(f"OpenClaw available: {client.is_available()}")
    print(f"Should route 'coding': {client.should_route('coding')}")
    print(f"Should route 'move'  : {client.should_route('move')} (always False — safety)")

    if client.is_enabled and client.is_available():
        result = client.query("Plan a simple maze navigation strategy", task_type="planning")
        print(f"OpenClaw says: {result}")
    else:
        print(
            "\nTo enable OpenClaw:\n"
            "  1. Ensure OpenClaw gateway is running on port 33635\n"
            "  2. Set `openclaw.enabled: true` in config/settings.yaml"
        )
