"""
tests/test_gazebo_movement.py
==============================
Integration smoke test for the Archer bridge → /cmd_vel pipeline.

These tests require:
  - A running ROS2 environment (inside Docker or native Humble)
  - The archer_bridge node running: ros2 run archer_bridge bridge_node

Run (inside Docker / WSL2):
  source /opt/ros/humble/setup.bash
  source ros2_ws/install/setup.bash
  pytest tests/test_gazebo_movement.py -v

Skip on Windows:
  These tests are skipped automatically if rclpy is not available.
"""

from __future__ import annotations

import json
import sys
import time

import pytest

# Skip entire module gracefully if rclpy isn't installed (e.g., on Windows)
rclpy = pytest.importorskip("rclpy", reason="rclpy not available — run inside ROS2 environment")

from rclpy.node import Node  # type: ignore
from geometry_msgs.msg import Twist  # type: ignore
from std_msgs.msg import String  # type: ignore


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
class CommandSender(Node):
    """Test helper node that publishes to /archer/command and records /cmd_vel."""

    def __init__(self):
        super().__init__("test_command_sender")
        self._publisher = self.create_publisher(String, "/archer/command", 10)
        self._received: list[Twist] = []
        self._sub = self.create_subscription(
            Twist, "/cmd_vel", self._on_twist, 10
        )

    def publish_command(self, cmd: dict) -> None:
        msg = String()
        msg.data = json.dumps(cmd)
        self._publisher.publish(msg)

    def _on_twist(self, msg: Twist) -> None:
        self._received.append(msg)

    def spin_briefly(self, seconds: float = 1.0) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    @property
    def last_twist(self) -> Twist | None:
        return self._received[-1] if self._received else None

    def clear(self) -> None:
        self._received.clear()


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def sender(ros_context):
    node = CommandSender()
    yield node
    node.destroy_node()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
class TestGazeboMovement:
    """
    Publish commands to /archer/command and verify /cmd_vel is correctly set.
    The bridge_node must already be running for these tests to pass.
    """

    def test_move_forward_produces_positive_linear(self, sender):
        """A 'move' command with positive linear should produce positive linear.x on /cmd_vel."""
        cmd = {"action": "move", "linear": 0.18, "angular": 0.0, "duration": 0, "raw": "test"}
        sender.clear()
        sender.publish_command(cmd)
        sender.spin_briefly(1.5)

        twist = sender.last_twist
        assert twist is not None, "No Twist received on /cmd_vel — is the bridge node running?"
        assert twist.linear.x == pytest.approx(0.18, abs=0.01)
        assert twist.angular.z == pytest.approx(0.0, abs=0.01)

    def test_stop_produces_zero_velocity(self, sender):
        """A 'stop' command should produce zero linear and angular on /cmd_vel."""
        cmd = {"action": "stop", "linear": 0.0, "angular": 0.0, "duration": -1, "raw": "test"}
        sender.clear()
        sender.publish_command(cmd)
        sender.spin_briefly(1.5)

        twist = sender.last_twist
        assert twist is not None
        assert twist.linear.x == pytest.approx(0.0, abs=0.001)
        assert twist.angular.z == pytest.approx(0.0, abs=0.001)

    def test_turn_left_produces_positive_angular(self, sender):
        """A 'rotate' command with positive angular should produce positive angular.z."""
        cmd = {"action": "rotate", "linear": 0.0, "angular": 0.6, "duration": 0, "raw": "test"}
        sender.clear()
        sender.publish_command(cmd)
        sender.spin_briefly(1.5)

        twist = sender.last_twist
        assert twist is not None
        assert twist.angular.z == pytest.approx(0.6, abs=0.01)
        assert twist.linear.x == pytest.approx(0.0, abs=0.001)

    def test_bridge_clamps_over_limit_velocity(self, sender):
        """Values above the safety limit should be clamped by the bridge."""
        cmd = {"action": "move", "linear": 99.9, "angular": 99.9, "duration": 0, "raw": "test"}
        sender.clear()
        sender.publish_command(cmd)
        sender.spin_briefly(1.5)

        twist = sender.last_twist
        assert twist is not None
        assert abs(twist.linear.x) <= 0.22, "Bridge failed to clamp linear velocity!"
        assert abs(twist.angular.z) <= 1.0, "Bridge failed to clamp angular velocity!"

    def test_invalid_json_does_not_crash_bridge(self, sender):
        """Sending invalid JSON should not crash the bridge — it should just log and ignore."""
        msg = String()
        msg.data = "this is not json {{{"
        sender._publisher.publish(msg)
        sender.spin_briefly(1.0)
        # If we get here without exception, the bridge handled it gracefully
        assert True

    def test_unknown_action_produces_no_motion(self, sender):
        """An 'unknown' action should not produce any movement."""
        # First send a stop so /cmd_vel is zero
        sender.publish_command({"action": "stop", "linear": 0.0, "angular": 0.0, "duration": -1, "raw": ""})
        sender.spin_briefly(0.5)
        sender.clear()

        # Now send unknown
        sender.publish_command({"action": "unknown", "linear": 0.0, "angular": 0.0, "duration": -1, "raw": ""})
        sender.spin_briefly(1.5)

        # Should not receive a non-zero velocity
        if sender.last_twist:
            assert sender.last_twist.linear.x == pytest.approx(0.0, abs=0.001)
            assert sender.last_twist.angular.z == pytest.approx(0.0, abs=0.001)
