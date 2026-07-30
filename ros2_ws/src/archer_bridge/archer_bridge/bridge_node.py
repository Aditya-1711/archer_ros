import json
import logging
import os
import yaml
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Float64MultiArray, Header, Bool
import time
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger

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
        self._explorer_enable_pub = self.create_publisher(Bool, "/explorer/enable", 10)
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._odom_sub = self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self._cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_callback, 10)
        
        # Heartbeat publishers
        self._heartbeat_pub = self.create_publisher(Header, "/archer/heartbeat/bridge", 10)
        self._ai_heartbeat_pub = self.create_publisher(Header, "/archer/heartbeat/ai_core", 10)
        
        # tf2 listener and broadcaster
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        from tf2_ros import TransformBroadcaster
        self._tf_broadcaster = TransformBroadcaster(self)
        
        # Services
        self._demo_start_client = self.create_client(Trigger, '/demonstration/start')
        self._demo_stop_client = self.create_client(Trigger, '/demonstration/stop')
        self._demo_replay_client = self.create_client(Trigger, '/demonstration/replay')

        
        # State
        self._last_processed_id = None
        try:
            cmd_file = os.path.join(self._sim_path, "last_cmd.yaml")
            if os.path.exists(cmd_file):
                with open(cmd_file, "r") as f:
                    payload = yaml.safe_load(f)
                self._last_processed_id = payload.get("cmd_id")
        except Exception:
            pass
            
        self._last_seen_ai_ts = None
        self._last_seen_ai_time = 0.0
        self._locations = self._load_locations()
        self._action_queue = []
        self._active_action = None
        self._stop_time = 0.0
        self._current_x = 0.0
        self._current_y = 0.0
        self._latest_cmd_vel = Twist()
        self._nav_goal_active = False
        self._nav_goal_success = False
        
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
        # Publish bridge heartbeat
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        self._heartbeat_pub.publish(hdr)
        
        # Check and publish AI Core heartbeat from host side (clock-independent)
        ai_hb_file = os.path.join(self._sim_path, "ai_heartbeat.json")
        if os.path.exists(ai_hb_file):
            try:
                with open(ai_hb_file, "r") as f:
                    payload = json.load(f)
                ts = payload.get("timestamp", 0.0)
                current_time = time.time()
                if self._last_seen_ai_ts is None or ts != self._last_seen_ai_ts:
                    self._last_seen_ai_ts = ts
                    self._last_seen_ai_time = current_time
                
                if current_time - self._last_seen_ai_time < 5.0: # If fresh within 5s
                    ai_hdr = Header()
                    ai_hdr.stamp = self.get_clock().now().to_msg()
                    self._ai_heartbeat_pub.publish(ai_hdr)
            except Exception: pass

        # 1. Continuous Velocity Publishing & Queue Management
        now = self.get_clock().now().nanoseconds / 1e9
        
        # Check if active nav_goal is reached via Nav2 feedback
        if self._active_action is not None and self._active_action.get("type") == "nav_goal":
            if not self._nav_goal_active and self._nav_goal_success:
                self.get_logger().info("Nav2 Goal reached successfully! Finishing action.")
                self._active_action = None
                self._nav_goal_success = False
            elif not self._nav_goal_active and not self._nav_goal_success and "target_x" in self._active_action:
                # Goal failed or aborted
                self.get_logger().warn("Nav2 Goal failed or was aborted. Finishing action.")
                self._active_action = None
                
        # If we have an active non-nav action and its duration elapsed
        if self._active_action is not None and self._active_action.get("type") != "nav_goal" and self._stop_time > 0 and now >= self._stop_time:
            self.get_logger().info("Action duration elapsed. Finishing action.")
            self._active_action = None
            self._vel_pub.publish(Twist()) # Stop moving
            
        # Process next action in queue if idle
        if self._active_action is None and len(self._action_queue) > 0:
            self._active_action = self._action_queue.pop(0)
            act_type = self._active_action.get("type") or self._active_action.get("action")
            
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
                
                # Disable explorer
                msg = Bool()
                msg.data = False
                self._explorer_enable_pub.publish(msg)
                
                self.get_logger().info(">>> QUEUE: STOP")
                
            elif act_type == "nav_goal":
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose.header.frame_id = "map"
                goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
                coords = self._active_action.get("coordinates", [0.0, 0.0, 0.0])
                goal_msg.pose.pose.position.x = float(coords[0])
                goal_msg.pose.pose.position.y = float(coords[1])
                goal_msg.pose.pose.orientation.w = 1.0
                
                # Publish to /goal_pose just for RViz visualisation
                pose_stamped = PoseStamped()
                pose_stamped.header = goal_msg.pose.header
                pose_stamped.pose = goal_msg.pose.pose
                self._goal_pub.publish(pose_stamped)
                
                # Send the actual Action Goal to Nav2
                if self._nav_client.wait_for_server(timeout_sec=1.0):
                    send_goal_future = self._nav_client.send_goal_async(goal_msg)
                    send_goal_future.add_done_callback(self._nav_goal_response_callback)
                    self._nav_goal_active = True
                    self._nav_goal_success = False
                    self.get_logger().info(f">>> QUEUE: Sent Nav2 Action Goal: {coords}")
                else:
                    self.get_logger().warn(f">>> QUEUE: Nav2 Action Server not available! Dropping goal: {coords}")
                    self._active_action = None

                # Save target coordinates
                if self._active_action is not None:
                    self._active_action["target_x"] = float(coords[0])
                    self._active_action["target_y"] = float(coords[1])
                    self._stop_time = now + 120.0 # Maximum 2 min wait
                
            elif act_type == "explore":
                self.get_logger().info(">>> QUEUE: Exploration triggered - enabling smart explorer node")
                
                # Enable the standalone smart_explorer node
                msg = Bool()
                msg.data = True
                self._explorer_enable_pub.publish(msg)
                
                self._active_action = None
                
            elif act_type == "patrol":
                self.get_logger().info(">>> QUEUE: Patrol triggered - queueing all rooms for sequential navigation")
                self._active_action = None
                
                patrol_actions = []
                for room_name, coords in self._locations.items():
                    patrol_actions.append({
                        "type": "nav_goal",
                        "coordinates": coords
                    })
                
                # Insert at the front of the queue so patrol starts immediately
                self._action_queue = patrol_actions + self._action_queue
                
            elif act_type == "dance":
                self.get_logger().info(">>> QUEUE: Dance triggered - let's bust a move!")
                self._active_action = None
                
                dance_actions = [
                    {"type": "cmd_vel", "linear": 0.0, "angular": 2.0, "duration": 1.0},
                    {"type": "cmd_vel", "linear": 0.0, "angular": -2.0, "duration": 1.0},
                    {"type": "cmd_vel", "linear": 0.5, "angular": 0.0, "duration": 0.5},
                    {"type": "cmd_vel", "linear": -0.5, "angular": 0.0, "duration": 0.5},
                    {"type": "cmd_vel", "linear": 0.0, "angular": 4.0, "duration": 1.5},
                    {"type": "stop"}
                ]
                self._action_queue = dance_actions + self._action_queue

            elif act_type == "return_to_dock":
                self.get_logger().info(">>> QUEUE: Return to dock triggered")
                self._active_action = None
                if "dock" in self._locations:
                    self._action_queue.insert(0, {
                        "type": "nav_goal",
                        "coordinates": self._locations["dock"]
                    })
                
            elif act_type == "spin":
                self.get_logger().info(">>> QUEUE: Spin command triggered")
                self._active_action = None
                self._action_queue.insert(0, {
                    "type": "rotate",
                    "angular": 1.0,
                    "duration": 6.28
                })
                
            elif act_type == "dance":
                self.get_logger().info(">>> QUEUE: Dance command triggered")
                self._active_action = None
                dance_moves = [
                    {"type": "rotate", "angular": 1.5, "duration": 1.0},
                    {"type": "rotate", "angular": -1.5, "duration": 2.0},
                    {"type": "rotate", "angular": 1.5, "duration": 1.0},
                    {"type": "move", "linear": 0.5, "duration": 1.0},
                    {"type": "move", "linear": -0.5, "duration": 1.0}
                ]
            elif act_type in ["introduce", "about_me", "tell_me_about_yourself"]:
                self.get_logger().info(">>> QUEUE: Introduce action triggered! Archer presenting details.")
                self._active_action = None
                
                intro_text = (
                    "Hello! I am Archer, an advanced autonomous humanoid robot platform designed for environment exploration, "
                    "semantic mapping, object perception, and intelligent voice/AI-driven task execution. "
                    "I am built on ROS 2 Jazzy with Gazebo Harmonic simulation, equipped with stereo vision (cuVSLAM), "
                    "360-degree LiDAR sensors, YOLOv8 vision perception, and Nav2 navigation."
                )
                self.get_logger().info(f"[ARCHER INTRO]: {intro_text}")
                
                # Write introduction to robot status file so UI/AI host can display/speak it
                try:
                    status_path = os.path.join(self._sim_path, "robot_status.json")
                    status_data = {}
                    if os.path.exists(status_path):
                        with open(status_path, "r") as f:
                            status_data = json.load(f)
                    status_data["last_speech"] = intro_text
                    status_data["introduction"] = {
                        "name": "Archer",
                        "type": "Autonomous Humanoid Robot Platform",
                        "architecture": "ROS 2 Jazzy + Gazebo Harmonic",
                        "perception": ["360° LiDAR", "YOLOv8 Object Detection", "Stereo cuVSLAM"],
                        "navigation": "Nav2 + Semantic Mapper + Autonomous Explorer",
                        "status": "Operational"
                    }
                    with open(status_path, "w") as f:
                        json.dump(status_data, f, indent=2)
                except Exception as e:
                    self.get_logger().error(f"Failed to write intro status: {e}")

                intro_moves = [
                    {"type": "rotate", "angular": 1.5, "duration": 1.5},
                    {"type": "rotate", "angular": -1.5, "duration": 1.5},
                    {"type": "stop"}
                ]
                self._action_queue = intro_moves + self._action_queue

            elif act_type == "clear_queue":
                self.get_logger().info(">>> QUEUE: Clear queue triggered")
                self._action_queue = []
                self._active_action = None
                self._vel_pub.publish(Twist()) # Stop moving
                
                # Disable explorer if active
                msg = Bool()
                msg.data = False
                self._explorer_enable_pub.publish(msg)

            elif act_type == "start_recording":
                self.get_logger().info(">>> QUEUE: Start LBD recording")
                if self._demo_start_client.wait_for_service(timeout_sec=1.0):
                    self._demo_start_client.call_async(Trigger.Request())
                self._active_action = None
                
            elif act_type == "stop_recording":
                self.get_logger().info(">>> QUEUE: Stop LBD recording")
                if self._demo_stop_client.wait_for_service(timeout_sec=1.0):
                    self._demo_stop_client.call_async(Trigger.Request())
                self._active_action = None
                
            elif act_type == "replay_demonstration":
                self.get_logger().info(">>> QUEUE: Replay LBD recording")
                if self._demo_replay_client.wait_for_service(timeout_sec=1.0):
                    self._demo_replay_client.call_async(Trigger.Request())
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

    def _nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Nav2 Goal rejected.')
            self._nav_goal_active = False
            self._nav_goal_success = False
            return

        self.get_logger().info('Nav2 Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._nav_goal_result_callback)

    def _nav_goal_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == 4: # SUCCEEDED
            self.get_logger().info('Nav2 Goal succeeded!')
            self._nav_goal_success = True
        else:
            self.get_logger().warn(f'Nav2 Goal failed with status: {status}')
            self._nav_goal_success = False
        self._nav_goal_active = False

    def _cmd_vel_callback(self, msg: Twist):
        self._latest_cmd_vel = msg

    def _odom_callback(self, msg: Odometry):
        # Fallback to odom values if TF is not ready
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Calculate yaw from quaternion orientation
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Attempt to get map-frame pose from TF
        try:
            t = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            # Calculate yaw from TF orientation
            q_tf = t.transform.rotation
            siny_tf = 2.0 * (q_tf.w * q_tf.z + q_tf.x * q_tf.y)
            cosy_tf = 1.0 - 2.0 * (q_tf.y * q_tf.y + q_tf.z * q_tf.z)
            yaw = math.atan2(siny_tf, cosy_tf)
            
            if not hasattr(self, '_tf_success_logged'):
                self.get_logger().info("TF lookup map -> base_link succeeded! Using map coordinates.")
                self._tf_success_logged = True
        except Exception as ex:
            if not hasattr(self, '_tf_fail_logged'):
                self.get_logger().warn(f"TF lookup map -> base_link failed: {ex}. Falling back to raw odom.")
                self._tf_fail_logged = True
            pass
            
        self._current_x = x
        self._current_y = y

        # Broadcast perfect odom -> base_link TF using Gazebo's wheel odometry
        # This replaces cuVSLAM's drifting TF and Gazebo's built-in TF which crashes Harmonic
        if hasattr(self, '_tf_broadcaster'):
            from geometry_msgs.msg import TransformStamped
            tf_msg = TransformStamped()
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.header.frame_id = "odom"
            tf_msg.child_frame_id = "base_link"
            tf_msg.transform.translation.x = msg.pose.pose.position.x
            tf_msg.transform.translation.y = msg.pose.pose.position.y
            tf_msg.transform.translation.z = msg.pose.pose.position.z
            tf_msg.transform.rotation = msg.pose.pose.orientation
            self._tf_broadcaster.sendTransform(tf_msg)
        
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
