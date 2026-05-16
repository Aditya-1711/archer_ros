import json
import logging
import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry

logger = logging.getLogger("archer.bridge")

class ArcherBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("archer_bridge")
        
        # Smart Path Discovery
        self._sim_path = self._discover_sim_path()
        self.get_logger().info(f"ArcherBridge initialized. Monitoring: {self._sim_path}")

        # Publishers & Subscribers
        self._vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self._odom_sub = self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        
        # State
        self._last_processed_id = None
        self._locations = self._load_locations()
        
        # Polling Timer (10Hz)
        self.create_timer(0.1, self._file_poll_callback)

    def _discover_sim_path(self) -> str:
        # Check explicit paths
        paths = [
            "/archer_ros/simulation",
            "/mnt/d/vm_friday/archer_ros/simulation",
            os.path.join(os.getcwd(), "simulation"),
            os.path.join(os.path.dirname(os.getcwd()), "simulation"),
        ]
        for p in paths:
            if os.path.exists(p):
                return p
        return os.path.join(os.getcwd(), "simulation") # Fallback

    def _load_locations(self):
        loc_path = os.path.join(self._sim_path, "locations.json")
        if os.path.exists(loc_path):
            with open(loc_path, "r") as f:
                return json.load(f)
        return {}

    def _file_poll_callback(self):
        cmd_file = os.path.join(self._sim_path, "last_cmd.yaml")
        if not os.path.exists(cmd_file):
            if not hasattr(self, '_warned_missing'):
                self.get_logger().info(f"Bridge active. Waiting for: {cmd_file}")
                self._warned_missing = True
            return
            
        try:
            with open(cmd_file, "r") as f:
                payload = yaml.safe_load(f)
            
            cmd_id = payload.get("cmd_id")
            if cmd_id != self._last_processed_id:
                self._last_processed_id = cmd_id
                self.get_logger().info(f"Executing AI Command: {cmd_id}")
                
                data = json.loads(payload.get("data", "{}"))
                action = data.get("action", {})
                self.get_logger().info(f"Decoded Action: {action}")
                
                twist = Twist()
                if action.get("type") in ["cmd_vel", "move", "rotate"]:
                    twist.linear.x = float(action.get("linear", 0.0))
                    twist.angular.z = float(action.get("angular", 0.0))
                    self._vel_pub.publish(twist)
                    self.get_logger().info(f">>> PUBLISHING VELOCITY: L={twist.linear.x}, A={twist.angular.z}")
                elif action.get("type") == "stop":
                    self._vel_pub.publish(twist) # Zero
                elif action.get("type") == "nav_goal":
                    goal = PoseStamped()
                    goal.header.frame_id = "map"
                    goal.header.stamp = self.get_clock().now().to_msg()
                    coords = action.get("coordinates", [0.0, 0.0, 0.0])
                    goal.pose.position.x = float(coords[0])
                    goal.pose.position.y = float(coords[1])
                    goal.pose.orientation.w = 1.0
                    self._goal_pub.publish(goal)
                    self.get_logger().info(f"Published Nav2 Goal: {coords}")
                elif action.get("type") == "explore":
                    # Simple exploration: publish a random goal or trigger a service
                    # For now, we'll just log it as a placeholder for SLAM Toolbox exploration
                    self.get_logger().info("Exploration triggered (Stub)")
        except Exception as e:
            self.get_logger().error(f"Bridge error: {e}")

    def _odom_callback(self, msg: Odometry):
        # Update shared status file
        status_file = os.path.join(self._sim_path, "robot_status.json")
        status = {
            "x": round(msg.pose.pose.position.x, 2),
            "y": round(msg.pose.pose.position.y, 2),
            "location": "living_room" # Placeholder
        }
        try:
            with open(status_file, "w") as f:
                json.dump(status, f)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ArcherBridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
