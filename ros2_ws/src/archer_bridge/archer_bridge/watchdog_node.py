"""
ros2_ws/src/archer_bridge/archer_bridge/watchdog_node.py
======================================================
ROS2 System Supervisor Watchdog Node.
Monitors node health and heartbeats, executing emergency stop actions if nodes fail.
"""

import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

class ArcherWatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__("archer_watchdog")
        
        # Discover directories
        self._sim_path = self._discover_sim_path()
        
        # Heartbeat tracking dictionary: {node_name: last_received_time}
        now = self.get_clock().now()
        self._heartbeats = {
            "ai_core": now,
            "bridge": now,
            "vision": now,
            "safety": now,
            "power": now
        }
        
        # Subscriptions
        self.create_subscription(Header, "/archer/heartbeat/ai_core", lambda msg: self._recv_heartbeat("ai_core"), 10)
        self.create_subscription(Header, "/archer/heartbeat/bridge", lambda msg: self._recv_heartbeat("bridge"), 10)
        self.create_subscription(Header, "/archer/heartbeat/vision", lambda msg: self._recv_heartbeat("vision"), 10)
        self.create_subscription(Header, "/archer/heartbeat/safety", lambda msg: self._recv_heartbeat("safety"), 10)
        self.create_subscription(Header, "/archer/heartbeat/power", lambda msg: self._recv_heartbeat("power"), 10)
        
        # Publisher to immediately command stops
        self._vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # Watchdog verification timer (10Hz)
        self.create_timer(0.1, self._watchdog_loop)
        
        # State
        self._start_time = self.get_clock().now()
        self._status = "nominal"
        self.get_logger().info("Watchdog supervisor running.")

    def _discover_sim_path(self) -> str:
        paths = [
            "/archer_ros/simulation",
            "/mnt/d/vm_friday/archer_ros/simulation",
            os.path.join(os.getcwd(), "simulation"),
            os.path.join(os.path.dirname(os.getcwd()), "simulation"),
        ]
        for p in paths:
            if os.path.exists(p):
                return p
        return os.path.join(os.getcwd(), "simulation")

    def _recv_heartbeat(self, node_name: str) -> None:
        self._heartbeats[node_name] = self.get_clock().now()

    def _watchdog_loop(self) -> None:
        now = self.get_clock().now()
        timeouts = {}
        status = "nominal"
        
        # Compute timeout durations
        for node, last_time in self._heartbeats.items():
            dt = (now - last_time).nanoseconds / 1e9
            timeouts[node] = dt
            
        # Determine status state (enforce a 15-second startup grace period)
        startup_duration = (now - self._start_time).nanoseconds / 1e9
        is_startup = (startup_duration < 15.0)
        
        if timeouts["bridge"] > 5.0 and not is_startup:
            status = "bridge_failure"
        elif timeouts["ai_core"] > 60.0 and not is_startup:
            status = "ai_core_offline"
        else:
            status = "nominal"

        # Edge-triggered transition actions
        if status != self._status:
            self.get_logger().info(f"WATCHDOG: State transition from {self._status} to {status}")
            self._status = status
            
            if self._status in ["safety_failure", "bridge_failure", "ai_core_offline"]:
                self.get_logger().warn(f"WATCHDOG: Critical node timeout ({status}). Command halting motion.")
                self._trigger_stop()
            elif self._status == "degraded":
                self.get_logger().warn("WATCHDOG: System status degraded due to minor node timeout.")
            elif self._status == "nominal":
                self.get_logger().info("WATCHDOG: All nodes nominal. Recovering safety states.")
                self._trigger_recovery()
            
        self._write_watchdog_status(timeouts)

    def _trigger_stop(self) -> None:
        stop_msg = Twist()
        self._vel_pub.publish(stop_msg)

        # Trigger estop service on safety supervisor if available
        try:
            cli = self.create_client(Trigger, "/archer/safety/estop")
            if cli.service_is_ready():
                req = Trigger.Request()
                cli.call_async(req)
        except Exception: pass

    def _trigger_recovery(self) -> None:
        # Trigger reset service on safety supervisor to clear EMERGENCY_STOP
        try:
            cli = self.create_client(Trigger, "/archer/safety/reset")
            if cli.service_is_ready():
                req = Trigger.Request()
                cli.call_async(req)
        except Exception: pass

    def _write_watchdog_status(self, timeouts: dict) -> None:
        status_file = os.path.join(self._sim_path, "robot_status.json")
        status_data = {}
        if os.path.exists(status_file):
            try:
                with open(status_file, "r") as f:
                    status_data = json.load(f)
            except: pass
            
        status_data["watchdog_status"] = self._status
        status_data["watchdog_timeouts"] = timeouts
        
        try:
            with open(status_file, "w") as f:
                json.dump(status_data, f)
        except: pass

def main(args=None):
    rclpy.init(args=args)
    node = ArcherWatchdogNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
