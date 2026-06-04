import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, SetBool
import json
import time
import os

class DemonstrationNode(Node):
    def __init__(self):
        super().__init__('demonstration_node')
        
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Services
        self.srv_start = self.create_service(Trigger, '/demonstration/start', self.start_callback)
        self.srv_stop = self.create_service(Trigger, '/demonstration/stop', self.stop_callback)
        self.srv_replay = self.create_service(Trigger, '/demonstration/replay', self.replay_callback)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.is_recording = False
        self.trajectory = []
        self.start_time = 0.0
        self.save_path = os.path.join(os.getcwd(), 'simulation', 'demonstration.json')
        
        self.get_logger().info("Demonstration Node started. Ready to record/replay trajectories.")

    def cmd_vel_callback(self, msg: Twist):
        if self.is_recording:
            current_time = time.time()
            dt = current_time - self.start_time
            
            # Only save non-zero or significant changes to avoid huge files
            if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
                self.trajectory.append({
                    "time": dt,
                    "linear_x": msg.linear.x,
                    "angular_z": msg.angular.z
                })

    def start_callback(self, request, response):
        self.is_recording = True
        self.trajectory = []
        self.start_time = time.time()
        response.success = True
        response.message = "Started recording demonstration."
        self.get_logger().info(response.message)
        return response

    def stop_callback(self, request, response):
        self.is_recording = False
        response.success = True
        response.message = f"Stopped recording. Saved {len(self.trajectory)} points."
        self.get_logger().info(response.message)
        
        try:
            with open(self.save_path, 'w') as f:
                json.dump(self.trajectory, f)
        except Exception as e:
            self.get_logger().error(f"Failed to save demonstration: {e}")
            
        return response

    def replay_callback(self, request, response):
        if not os.path.exists(self.save_path):
            response.success = False
            response.message = "No demonstration saved."
            return response
            
        try:
            with open(self.save_path, 'r') as f:
                traj = json.load(f)
                
            self.get_logger().info(f"Replaying {len(traj)} points...")
            start_t = time.time()
            
            for pt in traj:
                target_t = start_t + pt['time']
                sleep_time = target_t - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                msg = Twist()
                msg.linear.x = float(pt['linear_x'])
                msg.angular.z = float(pt['angular_z'])
                self.cmd_vel_pub.publish(msg)
                
            response.success = True
            response.message = "Replay complete."
        except Exception as e:
            response.success = False
            response.message = f"Failed to replay: {e}"
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DemonstrationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
