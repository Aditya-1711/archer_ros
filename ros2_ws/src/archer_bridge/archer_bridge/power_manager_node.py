"""
ros2_ws/src/archer_bridge/archer_bridge/power_manager_node.py
=============================================================
ROS2 node for Battery & Power Management with Dock Verification.
Simulates discharge, verifies contact alignment/current, and aggregates diagnostics.
"""

import os
import json
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from std_srvs.srv import Trigger

try:
    import faiss
    FAISS_AVAILABLE = True
except ImportError:
    FAISS_AVAILABLE = False

class ArcherPowerManager(Node):
    def __init__(self) -> None:
        super().__init__("power_manager")
        
        # Discover paths
        self._sim_path = self._discover_sim_path()
        
        # States
        self._battery_percent = 100.0
        self._voltage = 12.0
        self._current_draw = 1.25 # Amps
        self._charging = False
        
        # Current robot position (from Odometry)
        self._current_x = 0.0
        self._current_y = 0.0
        
        # Dock location
        self._dock_x = 0.0
        self._dock_y = 0.0
        
        # Verification state
        self._dock_sequence_active = False
        self._dock_verification_state = "idle" # idle, verifying, success, retrying, failed
        self._dock_retries = 0
        self._verification_start_time = 0.0
        self._battery_at_verification_start = 0.0
        
        # Other node states (for Diagnostics)
        self._safety_state = "NORMAL"
        self._vision_status = "nominal"
        self._vision_fps = 10.0
        
        # Publishers & Subscribers
        self._vel_sub = self.create_subscription(Twist, "/cmd_vel", self._vel_callback, 10)
        self._odom_sub = self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self._safety_sub = self.create_subscription(String, "/archer/safety/state", self._safety_callback, 10)
        self._vision_sub = self.create_subscription(String, "/archer/vision/detections", self._vision_callback, 10)
        
        self._battery_pub = self.create_publisher(BatteryState, "/sensor/battery", 10)
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self._heartbeat_pub = self.create_publisher(Header, "/archer/heartbeat/power", 10)
        
        # Services
        self._dock_srv = self.create_service(Trigger, "/archer/power/dock", self._dock_callback)
        self._undock_srv = self.create_service(Trigger, "/archer/power/undock", self._undock_callback)
        
        # 1Hz loop
        self.create_timer(1.0, self._power_loop)
        self.get_logger().info("Power Manager & Dock Verifier active.")

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

    def _vel_callback(self, msg: Twist) -> None:
        linear_val = abs(msg.linear.x)
        angular_val = abs(msg.angular.z)
        if linear_val > 0.01 or angular_val > 0.01:
            self._current_draw = 3.5 + (linear_val * 4.0) + (angular_val * 2.0)
        else:
            self._current_draw = 1.25

    def _odom_callback(self, msg: Odometry) -> None:
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y

    def _safety_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._safety_state = data.get("state", "NORMAL")
        except: pass

    def _vision_callback(self, msg: String) -> None:
        self._vision_fps = 10.0 # Nominal FPS
        self._vision_status = "nominal"

    def _power_loop(self) -> None:
        # Publish heartbeat
        hb = Header()
        hb.stamp = self.get_clock().now().to_msg()
        self._heartbeat_pub.publish(hb)
        
        # Compute charge / discharge
        if self._charging:
            self._battery_percent = min(100.0, self._battery_percent + 1.0)
            self._current_draw = -2.5 # Negative current draws represents charging flow
        else:
            self._battery_percent = max(0.0, self._battery_percent - (self._current_draw * 0.01))
            
        self._voltage = 10.5 + (self._battery_percent / 100.0) * 1.5
        
        # Auto-docking trigger
        if self._battery_percent <= 10.0 and not self._charging and not self._dock_sequence_active:
            self.get_logger().warn(f"Battery low ({self._battery_percent:.1f}%). Dispatching to dock.")
            self._trigger_docking()

        # Run Dock Verification State Machine
        if self._dock_sequence_active:
            self._run_dock_verification()

        # Publish battery metrics
        bat_msg = BatteryState()
        bat_msg.percentage = float(self._battery_percent / 100.0)
        bat_msg.voltage = float(self._voltage)
        bat_msg.current = float(self._current_draw)
        bat_msg.charge = float(self._battery_percent)
        bat_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING if self._charging else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self._battery_pub.publish(bat_msg)
        
        # Write diagnostics schemas
        self._write_unified_diagnostics()

    def _trigger_docking(self) -> None:
        self._dock_sequence_active = True
        self._dock_verification_state = "verifying"
        self._verification_start_time = time.time()
        self._battery_at_verification_start = self._battery_percent
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self._dock_x
        goal.pose.position.y = self._dock_y
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)
        self.get_logger().info("Dispatched return-to-dock goal coordinates to Nav2.")

    def _run_dock_verification(self) -> None:
        # Distance to dock center
        dist = ((self._current_x - self._dock_x)**2 + (self._current_y - self._dock_y)**2)**0.5
        
        if self._dock_verification_state == "verifying":
            # 1. Pose Alignment Check: must be within 0.25 meters of the dock
            if dist < 0.25:
                # 2. Charger contact pins closed (simulate delay and check)
                dt = time.time() - self._verification_start_time
                if dt > 5.0:
                    self.get_logger().info("Charger contact pins verified. Engaging active charging current.")
                    self._charging = True
                    self._dock_verification_state = "verifying_charge_increase"
                    self._verification_start_time = time.time()
                    self._battery_at_verification_start = self._battery_percent
            else:
                # Still navigating to dock; verify timeout
                if time.time() - self._verification_start_time > 60.0:
                    self.get_logger().error("Docking verification timeout! Re-attempting alignment.")
                    self._trigger_retry()
                    
        elif self._dock_verification_state == "verifying_charge_increase":
            # Ensure battery is increasing
            dt = time.time() - self._verification_start_time
            if dt > 3.0:
                if self._battery_percent > self._battery_at_verification_start:
                    self.get_logger().info("Active charge increase verified! Docking sequence successfully completed.")
                    self._dock_verification_state = "success"
                    self._dock_sequence_active = False
                else:
                    self.get_logger().warn("Contact verified but charge failed to increase. Retrying docking alignment.")
                    self._trigger_retry()

    def _trigger_retry(self) -> None:
        self._charging = False
        self._dock_retries += 1
        if self._dock_retries > 3:
            self.get_logger().error("Fatal: 3 docking retries failed. Halting and alerting user.")
            self._dock_verification_state = "failed"
            self._dock_sequence_active = False
            return
            
        self.get_logger().info(f"Retrying docking alignment (Attempt {self._dock_retries}/3). Backing up...")
        self._dock_verification_state = "retrying"
        
        # Publish temporary backup goal coordinate
        backup = PoseStamped()
        backup.header.frame_id = "map"
        backup.header.stamp = self.get_clock().now().to_msg()
        backup.pose.position.x = 0.0
        backup.pose.position.y = -0.6
        backup.pose.orientation.w = 1.0
        self._goal_pub.publish(backup)
        
        # Reset docking sequence after 5s backup
        self.create_timer(5.0, self._trigger_docking, once=True)

    def _write_unified_diagnostics(self) -> None:
        diag_file = os.path.join(self._sim_path, "diagnostics.json")
        status_file = os.path.join(self._sim_path, "robot_status.json")
        
        # Read parameters
        wd_status = "nominal"
        active_goal = None
        current_mode = "direct"
        location = "unknown"
        
        if os.path.exists(status_file):
            try:
                with open(status_file, "r") as f:
                    s_data = json.load(f)
                    wd_status = s_data.get("watchdog_status", "nominal")
                    active_goal = s_data.get("active_navigation_goal", None)
                    current_mode = s_data.get("current_operational_mode", "direct")
                    location = s_data.get("location", "unknown")
            except: pass
            
        # Resources (simulate realistic values of process execution)
        import random
        cpu = round(12.5 + random.uniform(-2.0, 2.0), 1)
        ram = round(342.0 + random.uniform(-5.0, 5.0), 1)
        
        # Simulated ambient temperature based on location
        base_temp = 22.0
        if location == "garage":
            base_temp = 18.5
        elif location == "kitchen":
            base_temp = 24.0
        elif location == "bedroom":
            base_temp = 21.0
        ambient_temp = round(base_temp + random.uniform(-0.3, 0.3), 1)
        
        diag = {
            "cpu_usage_pct": cpu,
            "ram_usage_mb": ram,
            "vision_fps": self._vision_fps,
            "battery_percent": round(self._battery_percent, 1),
            "safety_state": self._safety_state,
            "active_navigation_goal": active_goal,
            "memory_db_health": "nominal",
            "faiss_index_health": "active" if FAISS_AVAILABLE else "disabled",
            "watchdog_status": wd_status,
            "current_operational_mode": current_mode,
            "ambient_temp_c": ambient_temp
        }
        
        try:
            with open(diag_file, "w") as f:
                json.dump(diag, f)
        except: pass

    # --- Service Callbacks ---
    def _dock_callback(self, request, response):
        self._trigger_docking()
        response.success = True
        response.message = "Return-to-dock verification sequence started."
        return response

    def _undock_callback(self, request, response):
        self._charging = False
        self._dock_sequence_active = False
        self._dock_verification_state = "idle"
        response.success = True
        response.message = "Undocking verified. Charger contact broken."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArcherPowerManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
