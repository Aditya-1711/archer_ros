"""
ros2_ws/src/archer_bridge/archer_bridge/safety_supervisor_node.py
===================================================================
ROS2 node for local Safety & Emergency Stop Architecture.
Intercepts cmd_vel, monitors lidar /scan, and governs humanoid locomotion velocity.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
import json
from std_srvs.srv import Trigger

class ArcherSafetySupervisor(Node):
    def __init__(self) -> None:
        super().__init__("safety_supervisor")
        
        # State configurations
        self._state = "NORMAL" # NORMAL, WARNING, SAFE_STOP, EMERGENCY_STOP, RECOVERY
        self._min_dist = float('inf')
        self._last_scan_time = None
        
        # ROS parameters or constants
        self._warning_threshold = 0.8  # meters
        self._stop_threshold = 0.3     # meters
        self._sensor_timeout = 5.0     # seconds (increased for slow simulation)
        
        # Publishers & Subscribers
        self._cmd_vel_sub = self.create_subscription(
            Twist, "/archer/cmd_vel_raw", self._raw_vel_callback, 10
        )
        self._scan_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_callback, 10
        )
        
        self._vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._state_pub = self.create_publisher(String, "/archer/safety/state", 10)
        self._heartbeat_pub = self.create_publisher(Header, "/archer/heartbeat/safety", 10)
        
        # Services
        self._estop_srv = self.create_service(Trigger, "/archer/safety/estop", self._estop_callback)
        self._reset_srv = self.create_service(Trigger, "/archer/safety/reset", self._reset_callback)
        self._recover_srv = self.create_service(Trigger, "/archer/safety/recover", self._recover_callback)
        
        # Check sensor health periodic timer (20Hz)
        self.create_timer(0.05, self._safety_check_timer)
        self.get_logger().info("Safety Supervisor active and monitoring /scan.")

    def _scan_callback(self, msg: LaserScan) -> None:
        self._last_scan_time = self.get_clock().now()
        
        # Filter ranges for valid numbers
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        if valid_ranges:
            self._min_dist = min(valid_ranges)
        else:
            self._min_dist = float('inf')
            
        # Update safety state based on distance (unless E-STOP is active)
        if self._state != "EMERGENCY_STOP":
            if self._min_dist < self._stop_threshold:
                self._state = "SAFE_STOP"
            elif self._min_dist < self._warning_threshold:
                self._state = "WARNING"
            elif self._state in ["WARNING", "SAFE_STOP"]:
                self._state = "NORMAL"

    def _odom_callback(self, msg: Odometry) -> None:
        # Odometry heartbeat check if needed
        pass

    def _raw_vel_callback(self, msg: Twist) -> None:
        """Governs velocity commands before forwarding to Gazebo."""
        governed_msg = Twist()
        
        if self._state == "NORMAL":
            governed_msg.linear.x = msg.linear.x
            governed_msg.angular.z = msg.angular.z
            
        elif self._state == "WARNING":
            # Scale velocity by 50%
            governed_msg.linear.x = msg.linear.x * 0.5
            governed_msg.angular.z = msg.angular.z * 0.5
            
        elif self._state == "RECOVERY":
            # Back up slowly
            governed_msg.linear.x = -0.1
            governed_msg.angular.z = 0.0
            
        else: # SAFE_STOP or EMERGENCY_STOP
            governed_msg.linear.x = 0.0
            governed_msg.angular.z = 0.0
            
        self._vel_pub.publish(governed_msg)

    def _safety_check_timer(self) -> None:
        # Check sensor timeout if we have received at least one scan
        dt = 0.0
        if self._last_scan_time is not None:
            now = self.get_clock().now()
            dt = (now - self._last_scan_time).nanoseconds / 1e9
            
            if dt > self._sensor_timeout:
                if self._state != "EMERGENCY_STOP":
                    self.get_logger().error(f"Lidar sensor timeout! Time since last scan: {dt:.2f}s. Triggering EMERGENCY_STOP.")
                    self._state = "EMERGENCY_STOP"

        # Publish safety heartbeat
        hb_msg = Header()
        hb_msg.stamp = self.get_clock().now().to_msg()
        self._heartbeat_pub.publish(hb_msg)

        # Publish safety state topic
        state_msg = String()
        state_msg.data = json.dumps({
            "state": self._state,
            "min_obstacle_distance": self._min_dist if self._min_dist != float('inf') else 999.0,
            "lidar_timeout_seconds": dt
        })
        self._state_pub.publish(state_msg)

    # --- Service Callbacks ---
    def _estop_callback(self, request, response):
        self._state = "EMERGENCY_STOP"
        response.success = True
        response.message = "EMERGENCY_STOP triggered via service call. Motors disabled."
        self.get_logger().warn("EMERGENCY_STOP triggered by supervisor service.")
        
        # Publish immediate stop velocity
        self._vel_pub.publish(Twist())
        return response

    def _reset_callback(self, request, response):
        if self._state == "EMERGENCY_STOP":
            # Reset scan time to avoid immediate re-trigger
            self._last_scan_time = self.get_clock().now()
            self._state = "NORMAL"
            response.success = True
            response.message = "System state reset. Returning to NORMAL operation mode."
            self.get_logger().info("Safety reset successful. Returning to NORMAL.")
        else:
            response.success = False
            response.message = f"Cannot reset from state: {self._state}"
        return response

    def _recover_callback(self, request, response):
        if self._state in ["SAFE_STOP", "WARNING"]:
            self._state = "RECOVERY"
            response.success = True
            response.message = "Initiating safety recovery backing sequence."
            self.get_logger().info("Safety recovery backup initiated.")
            
            # Start a timer to finish recovery after 2 seconds
            self.create_timer(2.0, self._end_recovery_callback, once=True)
        else:
            response.success = False
            response.message = f"Cannot execute recovery from state: {self._state}"
        return response

    def _end_recovery_callback(self) -> None:
        if self._state == "RECOVERY":
            self.get_logger().info("Recovery sequence completed. Transitioning state.")
            self._state = "NORMAL"

def main(args=None):
    rclpy.init(args=args)
    node = ArcherSafetySupervisor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
