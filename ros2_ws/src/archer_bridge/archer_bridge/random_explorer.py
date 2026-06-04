#!/usr/bin/env python3
"""
Smart Explorer Node
Continuously sends Nav2 goals to explore the environment.
Only selects points in known FREE SPACE on the map, drastically reducing stuck times.
Subscribes to `/explorer/enable` to allow pausing/resuming exploration.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool
import random
import time
import math

class SmartExplorer(Node):
    def __init__(self):
        super().__init__('random_explorer')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self._map_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.map_callback, 10)
        self._enable_sub = self.create_subscription(Bool, '/explorer/enable', self.enable_callback, 10)
        
        self._timer = self.create_timer(2.0, self.timer_callback)
        
        self._goal_active = False
        self._explore_enabled = False
        self._map_data = None
        self._map_info = None
        
        self._goal_handle = None

        self._last_target_x = 0.0
        self._last_target_y = 0.0
        self._robot_x = 0.0
        self._robot_y = 0.0

        self._odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

        # Bounding box for the house
        self.x_min, self.x_max = -8.0, 8.0
        self.y_min, self.y_max = -10.0, 10.0

    def enable_callback(self, msg: Bool):
        self._explore_enabled = msg.data
        if self._explore_enabled:
            self.get_logger().info("Exploration ENABLED.")
        else:
            self.get_logger().info("Exploration DISABLED. Halting.")
            if self._goal_active and self._goal_handle:
                self.get_logger().info("Canceling current exploration goal.")
                self._goal_handle.cancel_goal_async()
                self._goal_active = False

    def map_callback(self, msg: OccupancyGrid):
        self._map_data = msg.data
        self._map_info = msg.info

    def is_free_space(self, x, y):
        if self._map_info is None or self._map_data is None:
            return False
            
        res = self._map_info.resolution
        origin_x = self._map_info.origin.position.x
        origin_y = self._map_info.origin.position.y
        width = self._map_info.width
        height = self._map_info.height
        
        grid_x = int((x - origin_x) / res)
        grid_y = int((y - origin_y) / res)
        
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return False
            
        index = grid_y * width + grid_x
        if index < 0 or index >= len(self._map_data):
            return False
            
        cell_value = self._map_data[index]
        return cell_value == 0 # 0 is definitively free space

    def timer_callback(self):
        if not self._explore_enabled:
            return
            
        if self._goal_active:
            return
            
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for Nav2 action server...')
            return
            
        if self._map_info is None:
            self.get_logger().info('Waiting for /map data to pick valid goals...')
            return
            
        self.send_smart_goal()

    def send_smart_goal(self):
        if self._map_info is None or self._map_data is None:
            return
            
        # Find all free cells (value == 0)
        free_indices = [i for i, val in enumerate(self._map_data) if val == 0]
        
        if not free_indices:
            self.get_logger().warn('No free space found in the map yet. Retrying later.')
            return
            
        res = self._map_info.resolution
        origin_x = self._map_info.origin.position.x
        origin_y = self._map_info.origin.position.y
        width = self._map_info.width
        height = self._map_info.height
        
        # Try to find a point at least 2.0 meters away from current location, and within 8.0 meters (so it stays in the costmap)
        best_x, best_y = 0.0, 0.0
        found = False
        # Initialize history if not present
        if not hasattr(self, '_history'):
            self._history = []
            
        candidates = random.sample(free_indices, min(1000, len(free_indices)))
        for idx in candidates:
            grid_y = idx // width
            grid_x = idx % width
            x = origin_x + (grid_x * res) + (res / 2.0)
            y = origin_y + (grid_y * res) + (res / 2.0)
            
            dist_to_robot = math.hypot(x - self._robot_x, y - self._robot_y)
            
            # Prefer points far away, but penalize points near previous goals
            if dist_to_robot > 4.0:
                too_close_to_history = False
                for hx, hy in self._history:
                    if math.hypot(x - hx, y - hy) < 3.0:
                        too_close_to_history = True
                        break
                        
                if too_close_to_history:
                    continue
                    
                # Basic check: avoid points right next to walls (check neighbors)
                safe = True
                for dx in [-4, 0, 4]:
                    for dy in [-4, 0, 4]:
                        n_x = grid_x + dx
                        n_y = grid_y + dy
                        if 0 <= n_x < width and 0 <= n_y < height:
                            n_idx = n_y * width + n_x
                            if self._map_data[n_idx] != 0:
                                safe = False
                                break
                    if not safe: break
                
                if safe:
                    best_x = x
                    best_y = y
                    found = True
                    break
                
        if not found:
            self.get_logger().warn('Could not find a safe distant unvisited point. Picking completely random free space.')
            # Clear history if we are trapped
            self._history = []
            idx = random.choice(candidates)
            grid_y = idx // width
            grid_x = idx % width
            best_x = origin_x + (grid_x * res) + (res / 2.0)
            best_y = origin_y + (grid_y * res) + (res / 2.0)
            
        self._history.append((best_x, best_y))
        
        # Keep history short (last 10 goals)
        if len(self._history) > 10:
            self._history.pop(0)
            
        self._last_target_x = best_x
        self._last_target_y = best_y
        
        x = best_x
        y = best_y
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'Sending Smart Goal: ({x:.2f}, {y:.2f}) - Validated as free space!')
        
        self._goal_active = True
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2. Trying another...')
            self._goal_active = False
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted. Navigating...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().status
        # Status code 4 is SUCCEEDED, 6 is ABORTED, 5 is CANCELED
        if result == 4:
            self.get_logger().info('Smart goal reached successfully!')
        elif result == 5:
            self.get_logger().info('Smart goal canceled.')
        else:
            self.get_logger().warn(f'Nav2 Goal failed/aborted (status: {result}). Retrying...')
            
        time.sleep(1.0)
        self._goal_active = False

def main(args=None):
    rclpy.init(args=args)
    node = SmartExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
