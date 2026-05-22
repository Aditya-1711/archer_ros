import json
import logging
import os
import yaml
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Float64MultiArray
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
        self._cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_callback, 10)
        
        # State
        self._last_processed_id = None
        self._locations = self._load_locations()
        self._action_queue = []
        self._active_action = None
        self._stop_time = 0.0
        self._current_x = 0.0
        self._current_y = 0.0
        self._latest_cmd_vel = Twist()
        
        # Simulated Diagnostics
        self._battery = 100.0
        self._cpu_temp = 45.0
        
        # Gait & Walking Animation Configuration
        self._gait_pub = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
        self._gait_time = 0.0
        self._gait_amplitude = 0.35 # Radian swing for hips
        self._knee_amplitude = 0.40 # Radian bend for knees
        self._arm_amplitude = 0.30 # Radian swing for shoulders
        self._current_joint_positions = [0.0] * 11 # Track current states to interpolate smoothly
        
        # Timers
        self.create_timer(0.05, self._gait_timer_callback) # 20Hz Gait Animator
        self.create_timer(0.1, self._file_poll_callback)
        self.create_timer(10.0, self._diagnostics_callback)

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
        # 1. Continuous Velocity Publishing & Queue Management
        now = self.get_clock().now().nanoseconds / 1e9
        
        # Check if active nav_goal is reached
        if self._active_action is not None and self._active_action.get("type") == "nav_goal":
            tx = self._active_action.get("target_x")
            ty = self._active_action.get("target_y")
            if tx is not None and ty is not None:
                dist = math.hypot(self._current_x - tx, self._current_y - ty)
                if dist < 0.5:
                    self.get_logger().info(f"Goal reached! Distance to target: {dist:.2f}m. Finishing action.")
                    self._active_action = None
                    self._vel_pub.publish(Twist()) # Stop moving
        
        # If we have an active action and its duration elapsed
        if self._active_action is not None and self._stop_time > 0 and now >= self._stop_time:
            self.get_logger().info("Action duration elapsed. Finishing action.")
            self._active_action = None
            self._vel_pub.publish(Twist()) # Stop moving
            
        # Process next action in queue if idle
        if self._active_action is None and len(self._action_queue) > 0:
            self._active_action = self._action_queue.pop(0)
            act_type = self._active_action.get("type")
            
            if act_type in ["cmd_vel", "move", "rotate"]:
                twist = Twist()
                twist.linear.x = float(self._active_action.get("linear", 0.0))
                twist.angular.z = float(self._active_action.get("angular", 0.0))
                duration = float(self._active_action.get("duration", 2.0))
                
                self._active_action["twist"] = twist
                if duration > 0:
                    self._stop_time = now + duration
                else:
                    self._stop_time = 0.0 # Infinite
                
                self.get_logger().info(f">>> QUEUE: VELOCITY SET: L={twist.linear.x}, A={twist.angular.z}, Dur={duration}s")
            
            elif act_type == "stop":
                self._active_action = None
                self._vel_pub.publish(Twist()) # Zero
                self.get_logger().info(">>> QUEUE: STOP")
                
            elif act_type == "nav_goal":
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = self.get_clock().now().to_msg()
                coords = self._active_action.get("coordinates", [0.0, 0.0, 0.0])
                goal.pose.position.x = float(coords[0])
                goal.pose.position.y = float(coords[1])
                goal.pose.orientation.w = 1.0
                self._goal_pub.publish(goal)
                self.get_logger().info(f">>> QUEUE: Published Nav2 Goal: {coords}")
                # Save target coordinates for distance monitoring
                self._active_action["target_x"] = float(coords[0])
                self._active_action["target_y"] = float(coords[1])
                # Set a safety timeout of 60.0 seconds
                self._stop_time = now + 60.0
                
            elif act_type == "explore":
                self.get_logger().info(">>> QUEUE: Exploration triggered (Stub)")
                self._active_action = None

        # Continuous publishing for active twist
        if self._active_action is not None and "twist" in self._active_action:
            self._vel_pub.publish(self._active_action["twist"])

        # 2. File Polling
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
                self.get_logger().info(f"Executing AI Command ID: {cmd_id}")
                
                data = json.loads(payload.get("data", "{}"))
                actions = data.get("actions", [])
                
                # If legacy 'action' key is used, wrap it
                if not actions and "action" in data:
                    actions = [data["action"]]
                    
                self.get_logger().info(f"Decoded {len(actions)} Actions into Queue.")
                
                # Overwrite queue
                self._action_queue = actions
                self._active_action = None
                
        except Exception as e:
            self.get_logger().error(f"Bridge error: {e}")

    def _cmd_vel_callback(self, msg: Twist):
        self._latest_cmd_vel = msg

    def _odom_callback(self, msg: Odometry):
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        x = self._current_x
        y = self._current_y
        
        # Calculate yaw from quaternion orientation
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Determine current room based on locations.json bounding boxes
        current_room = "unknown"
        for loc_name, coords in self._locations.items():
            if len(coords) >= 2:
                # Basic distance check since coordinates are centers (assuming 2m radius per room)
                dist = ((x - coords[0])**2 + (y - coords[1])**2)**0.5
                if dist < 3.0:
                    current_room = loc_name
                    break
        
        status_file = os.path.join(self._sim_path, "robot_status.json")
        status = {
            "x": round(x, 2),
            "y": round(y, 2),
            "yaw": round(yaw, 3),
            "location": current_room,
            "battery": round(self._battery, 1)
        }
        try:
            with open(status_file, "w") as f:
                json.dump(status, f)
        except:
            pass

    def _diagnostics_callback(self):
        # Simulate Battery Drain
        if self._active_action is not None and "twist" in self._active_action:
            self._battery -= 0.05
            self._cpu_temp += 0.5
        else:
            self._battery -= 0.01
            self._cpu_temp = max(45.0, self._cpu_temp - 1.0)
            
        diag_file = os.path.join(self._sim_path, "diagnostics.json")
        diag = {
            "battery_percent": round(self._battery, 1),
            "cpu_temp_c": round(self._cpu_temp, 1),
            "status": "nominal" if self._battery > 15 else "low_battery"
        }
        try:
            with open(diag_file, "w") as f:
                json.dump(diag, f)
        except:
            pass

    def _gait_timer_callback(self):
        # Detect if the robot is moving
        moving = False
        linear_vel = 0.0
        angular_vel = 0.0
        
        # Check latest commanded velocity (both direct commands and Nav2 cmd_vel)
        linear_vel = abs(self._latest_cmd_vel.linear.x)
        angular_vel = abs(self._latest_cmd_vel.angular.z)
        if linear_vel > 0.01 or angular_vel > 0.01:
            moving = True
                
        # Target joint angles array matching order in controllers.yaml:
        # 0: jL5S1_roty (spine 1)
        # 1: jT9T8_roty (spine 2)
        # 2: jT1C7_roty (neck/head)
        # 3: jLeftHip_roty
        # 4: jLeftKnee_roty
        # 5: jRightHip_roty
        # 6: jRightKnee_roty
        # 7: jLeftShoulder_roty
        # 8: jLeftElbow_roty
        # 9: jRightShoulder_roty
        # 10: jRightElbow_roty
        targets = [0.0] * 11
        
        if moving:
            # Dynamic velocity-responsive frequency (steps speed)
            speed_factor = max(linear_vel, angular_vel * 0.3)
            freq = 4.0 * max(0.2, min(1.0, speed_factor))
            self._gait_time += 0.05 * freq * 2.0 * math.pi
            
            # 1. Hips (Opposing sinusoidal swings)
            targets[3] = self._gait_amplitude * math.sin(self._gait_time)  # Left Hip
            targets[5] = -self._gait_amplitude * math.sin(self._gait_time) # Right Hip
            
            # 2. Knees (Bend backward during the rear swing phase)
            # Left Knee bends when Left Hip swings back (sin(t) < 0)
            targets[4] = self._knee_amplitude * (math.cos(self._gait_time) + 1.0) if math.sin(self._gait_time) < 0 else 0.0
            # Right Knee bends when Right Hip swings back (sin(t+pi) < 0)
            targets[6] = self._knee_amplitude * (math.cos(self._gait_time + math.pi) + 1.0) if math.sin(self._gait_time + math.pi) < 0 else 0.0
            
            # 3. Shoulders (Swing in sync with opposite leg)
            targets[7] = -self._arm_amplitude * math.sin(self._gait_time) # Left Shoulder
            targets[9] = self._arm_amplitude * math.sin(self._gait_time)  # Right Shoulder
            
            # 4. Elbows (Compliant dynamic arm sway)
            targets[8] = -0.3 + 0.15 * math.cos(self._gait_time) # Left Elbow
            targets[10] = -0.3 + 0.15 * math.cos(self._gait_time + math.pi) # Right Elbow
            
            # 5. Torso/Spine (Counter-balance lean)
            targets[0] = 0.04 * math.sin(self._gait_time) # Spine 1 (L5S1)
            targets[1] = 0.02 * math.cos(self._gait_time) # Spine 2 (T9T8)
            
        else:
            # Stand upright with a relaxed, domestic humanoid posture (slight elbow bends)
            rest_pose = [0.0] * 11
            rest_pose[8] = -0.2  # Left elbow relaxed bend
            rest_pose[10] = -0.2 # Right elbow relaxed bend
            targets = rest_pose
            
        # Exponential moving average filter for smooth, organic transition (prevents snaps)
        interpolation_speed = 0.12 # Smooth glide rate
        for i in range(11):
            self._current_joint_positions[i] = (1.0 - interpolation_speed) * self._current_joint_positions[i] + interpolation_speed * targets[i]
            
        # Publish to joint controllers
        msg = Float64MultiArray()
        msg.data = self._current_joint_positions
        self._gait_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArcherBridgeNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
