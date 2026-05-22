"""
tests/test_command_parser.py
============================
Unit tests for archer_ros/ai/parser/command_parser.py

Tests verify:
  - Correct action detection for all command types
  - Speed qualifier detection
  - Duration extraction
  - Velocity clamping (safety critical)
  - Malformed/unknown input handling
  - is_safe() validation

Run: pytest tests/test_command_parser.py -v
"""

from __future__ import annotations

import sys
from pathlib import Path

# Add project root to path so `ai` imports work
sys.path.insert(0, str(Path(__file__).parent.parent))

import pytest
from ai.parser.command_parser import CommandParser, parse_command


@pytest.fixture
def parser() -> CommandParser:
    return CommandParser(enable_llm_fallback=False)


# ---------------------------------------------------------------------------
# MOVE FORWARD
# ---------------------------------------------------------------------------
class TestMoveForward:
    def test_basic_forward(self, parser):
        cmd = parser.parse("Moving forward at moderate speed, Boss.")
        assert cmd["action"] == "move"
        assert cmd["linear"] > 0, "Forward linear should be positive"
        assert cmd["angular"] == 0.0

    def test_drive_forward(self, parser):
        cmd = parser.parse("Driving forward slowly.")
        assert cmd["action"] == "move"
        assert cmd["linear"] == pytest.approx(0.40, abs=0.01)

    def test_go_ahead(self, parser):
        cmd = parser.parse("Going ahead quickly.")
        assert cmd["action"] == "move"
        assert cmd["linear"] == pytest.approx(1.60, abs=0.01)

    def test_advance(self, parser):
        cmd = parser.parse("Advancing straight.")
        assert cmd["action"] == "move"
        assert cmd["linear"] > 0


# ---------------------------------------------------------------------------
# MOVE BACKWARD
# ---------------------------------------------------------------------------
class TestMoveBackward:
    def test_basic_backward(self, parser):
        cmd = parser.parse("Moving backward slowly.")
        assert cmd["action"] == "move"
        assert cmd["linear"] < 0, "Backward linear should be negative"

    def test_reversing(self, parser):
        cmd = parser.parse("Reversing slowly for 3 seconds.")
        assert cmd["action"] == "move"
        assert cmd["linear"] < 0
        assert cmd["duration"] == pytest.approx(3.0, abs=0.01)

    def test_back_up(self, parser):
        cmd = parser.parse("Backing up gently.")
        assert cmd["action"] == "move"
        assert cmd["linear"] < 0


# ---------------------------------------------------------------------------
# TURN LEFT
# ---------------------------------------------------------------------------
class TestTurnLeft:
    def test_turn_left(self, parser):
        cmd = parser.parse("Turning left. Adjusting heading, Boss.")
        assert cmd["action"] == "rotate"
        assert cmd["angular"] > 0, "Left turn angular should be positive"
        assert cmd["linear"] == 0.0

    def test_rotate_ccw(self, parser):
        cmd = parser.parse("Rotating counter-clockwise slowly.")
        assert cmd["action"] == "rotate"
        assert cmd["angular"] > 0

    def test_pivot_left(self, parser):
        cmd = parser.parse("Pivoting left.")
        assert cmd["action"] == "rotate"
        assert cmd["angular"] > 0


# ---------------------------------------------------------------------------
# TURN RIGHT
# ---------------------------------------------------------------------------
class TestTurnRight:
    def test_turn_right(self, parser):
        cmd = parser.parse("Turning right quickly.")
        assert cmd["action"] == "rotate"
        assert cmd["angular"] < 0, "Right turn angular should be negative"
        assert cmd["linear"] == 0.0

    def test_rotate_cw(self, parser):
        cmd = parser.parse("Rotating clockwise.")
        assert cmd["action"] == "rotate"
        assert cmd["angular"] < 0

    def test_pivot_right(self, parser):
        cmd = parser.parse("Pivoting right sharply.")
        assert cmd["action"] == "rotate"
        assert cmd["angular"] < 0


# ---------------------------------------------------------------------------
# STOP
# ---------------------------------------------------------------------------
class TestStop:
    def test_stop(self, parser):
        cmd = parser.parse("Stopping now. Robot halted, Boss.")
        assert cmd["action"] == "stop"
        assert cmd["linear"] == 0.0
        assert cmd["angular"] == 0.0

    def test_halt(self, parser):
        cmd = parser.parse("Halting immediately.")
        assert cmd["action"] == "stop"

    def test_freeze(self, parser):
        cmd = parser.parse("Freeze.")
        assert cmd["action"] == "stop"

    def test_stand_by(self, parser):
        cmd = parser.parse("Standing by.")
        assert cmd["action"] == "stop"

    def test_stop_overrides_forward(self, parser):
        """STOP keyword should take priority over any movement keyword."""
        cmd = parser.parse("Stop moving forward.")
        assert cmd["action"] == "stop"


# ---------------------------------------------------------------------------
# STATUS
# ---------------------------------------------------------------------------
class TestStatus:
    def test_status_nominal(self, parser):
        cmd = parser.parse("All systems nominal. Standing by for your orders, Boss.")
        assert cmd["action"] in {"status", "stop"}  # "standing by" may match stop

    def test_systems_operational(self, parser):
        cmd = parser.parse("Systems operational.")
        assert cmd["action"] == "status"


# ---------------------------------------------------------------------------
# UNKNOWN / MALFORMED
# ---------------------------------------------------------------------------
class TestUnknown:
    def test_unknown_text(self, parser):
        cmd = parser.parse("The weather is nice today.")
        assert cmd["action"] == "unknown"
        assert cmd["linear"] == 0.0
        assert cmd["angular"] == 0.0

    def test_empty_string(self, parser):
        cmd = parser.parse("")
        assert cmd["action"] == "unknown"

    def test_random_json_does_not_move_robot(self, parser):
        cmd = parser.parse('{"foo": "bar"}')
        assert cmd["action"] == "unknown"
        assert cmd["linear"] == 0.0


# ---------------------------------------------------------------------------
# VELOCITY CLAMPING (safety critical)
# ---------------------------------------------------------------------------
class TestVelocityClamping:
    """Verify that no command can exceed configured safety limits."""

    def test_fast_forward_clamped(self, parser):
        """Even 'fast' forward should not exceed max_linear."""
        cmd = parser.parse("Moving forward as fast as possible.")
        assert abs(cmd["linear"]) <= parser.max_linear

    def test_fast_turn_clamped(self, parser):
        cmd = parser.parse("Rotating right as fast as possible.")
        assert abs(cmd["angular"]) <= parser.max_angular

    def test_clamp_method_positive(self, parser):
        """Direct clamp test: value above max."""
        raw = {"action": "move", "linear": 999.9, "angular": 999.9, "duration": 0, "raw": ""}
        clamped = parser._clamp(raw)
        assert clamped["linear"] <= parser.max_linear
        assert clamped["angular"] <= parser.max_angular

    def test_clamp_method_negative(self, parser):
        """Direct clamp test: value below negative max."""
        raw = {"action": "move", "linear": -999.9, "angular": -999.9, "duration": 0, "raw": ""}
        clamped = parser._clamp(raw)
        assert clamped["linear"] >= -parser.max_linear
        assert clamped["angular"] >= -parser.max_angular


# ---------------------------------------------------------------------------
# DURATION PARSING
# ---------------------------------------------------------------------------
class TestDuration:
    def test_duration_seconds(self, parser):
        cmd = parser.parse("Moving forward for 5 seconds.")
        assert cmd["action"] == "move"
        assert cmd["duration"] == pytest.approx(5.0, abs=0.01)

    def test_duration_float_seconds(self, parser):
        cmd = parser.parse("Turn left for 2.5 sec.")
        assert cmd["action"] == "rotate"
        assert cmd["duration"] == pytest.approx(2.5, abs=0.01)

    def test_no_duration(self, parser):
        cmd = parser.parse("Move forward.")
        assert cmd["duration"] == 2.0  # Default to 2.0


# ---------------------------------------------------------------------------
# SAFETY CHECK (is_safe)
# ---------------------------------------------------------------------------
class TestIsSafe:
    def test_valid_move_is_safe(self, parser):
        cmd = {"action": "move", "linear": 0.18, "angular": 0.0, "duration": 0, "raw": ""}
        assert parser.is_safe(cmd) is True

    def test_valid_rotate_is_safe(self, parser):
        cmd = {"action": "rotate", "linear": 0.0, "angular": 0.6, "duration": 0, "raw": ""}
        assert parser.is_safe(cmd) is True

    def test_stop_is_safe(self, parser):
        cmd = {"action": "stop", "linear": 0.0, "angular": 0.0, "duration": -1, "raw": ""}
        assert parser.is_safe(cmd) is True

    def test_unknown_is_not_safe(self, parser):
        cmd = {"action": "unknown", "linear": 0.0, "angular": 0.0, "duration": -1, "raw": ""}
        assert parser.is_safe(cmd) is False

    def test_over_limit_linear_is_not_safe(self, parser):
        cmd = {"action": "move", "linear": 10.0, "angular": 0.0, "duration": 0, "raw": ""}
        assert parser.is_safe(cmd) is False

    def test_over_limit_angular_is_not_safe(self, parser):
        cmd = {"action": "rotate", "linear": 0.0, "angular": 50.0, "duration": 0, "raw": ""}
        assert parser.is_safe(cmd) is False


# ---------------------------------------------------------------------------
# Module-level convenience function
# ---------------------------------------------------------------------------
class TestModuleLevelFunction:
    def test_parse_command_function(self):
        cmd = parse_command("Moving forward slowly.")
        assert cmd["action"] == "move"
        assert cmd["linear"] > 0
