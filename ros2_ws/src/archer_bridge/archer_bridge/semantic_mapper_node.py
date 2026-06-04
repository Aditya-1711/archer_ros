#!/usr/bin/env python3
"""
ros2_ws/src/archer_bridge/archer_bridge/semantic_mapper_node.py
=====================================================
Fuses YOLO 2D detections with Depth camera data to project objects into 3D.
Publishes RViz markers and triggers TTS on the dashboard.
"""

import json
import math
import requests
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from tf2_ros import Buffer, TransformException
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

try:
    from cv_bridge import CvBridge
except ImportError:
    pass

class SemanticMapperNode(Node):
    def __init__(self) -> None:
        super().__init__("semantic_mapper")
        
        # Subscriptions
        self._det_sub = self.create_subscription(String, "/yolo/detections", self._det_callback, 10)
        self._depth_sub = self.create_subscription(Image, "/archer/camera/depth/image_raw", self._depth_callback, 10)
        self._info_sub = self.create_subscription(CameraInfo, "/archer/camera/camera_info", self._info_callback, 10)
        
        # Publishers
        self._marker_pub = self.create_publisher(MarkerArray, "/archer/semantic_markers", 10)
        
        # TF2
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        # State
        self._cv_bridge = CvBridge()
        self._latest_depth = None
        self._camera_info = None
        self._seen_objects = {}  # {class_name: [(x, y, z), ...]}
        self._marker_id_counter = 0
        
        self._dashboard_url = "http://localhost:8080/api/speak"
        self._tts_cooldowns = {} # Prevent spamming TTS for the same object class
        
        self.get_logger().info("Semantic Mapper initialized. Waiting for detections and depth...")

    def _info_callback(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _depth_callback(self, msg: Image) -> None:
        try:
            # Depth from Gazebo is usually 32FC1 (float meters)
            self._latest_depth = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warn(f"Failed to convert depth image: {e}")

    def _det_callback(self, msg: String) -> None:
        if self._latest_depth is None or self._camera_info is None:
            return
        
        try:
            detections = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Invalid detection JSON: {e}")
            return
            
        marker_array = MarkerArray()
        new_objects_found = []

        for det in detections:
            bbox = det.get("bbox", [])
            if len(bbox) != 4:
                continue
                
            cls_name = det.get("class_name", "object")
            
            # Find center pixel of bounding box
            x1, y1, x2, y2 = bbox
            cx = int((x1 + x2) / 2.0)
            cy = int((y1 + y2) / 2.0)
            
            # Bound check against image dimensions
            h, w = self._latest_depth.shape
            if not (0 <= cx < w and 0 <= cy < h):
                continue
                
            # Sample depth at center
            z_cam = float(self._latest_depth[cy, cx])
            
            # Skip invalid depth (0.0 or infinity)
            if math.isnan(z_cam) or math.isinf(z_cam) or z_cam <= 0.05 or z_cam > 15.0:
                continue
                
            # Deproject pixel to 3D point in camera frame
            # K matrix: [fx,  0, cx]
            #           [ 0, fy, cy]
            #           [ 0,  0,  1]
            fx = self._camera_info.k[0]
            cx0 = self._camera_info.k[2]
            fy = self._camera_info.k[4]
            cy0 = self._camera_info.k[5]
            
            x_cam = (cx - cx0) * z_cam / fx
            y_cam = (cy - cy0) * z_cam / fy
            
            # Transform from camera optical frame to map frame
            try:
                # We want point relative to map
                trans = self._tf_buffer.lookup_transform(
                    "map", 
                    self._camera_info.header.frame_id, 
                    rclpy.time.Time()
                )
                
                # Manual point transformation
                import tf2_geometry_msgs
                from geometry_msgs.msg import PointStamped
                
                pt_cam = PointStamped()
                pt_cam.header.frame_id = self._camera_info.header.frame_id
                pt_cam.header.stamp = self.get_clock().now().to_msg()
                pt_cam.point.x = x_cam
                pt_cam.point.y = y_cam
                pt_cam.point.z = z_cam
                
                pt_map = tf2_geometry_msgs.do_transform_point(pt_cam, trans)
                map_x = pt_map.point.x
                map_y = pt_map.point.y
                map_z = pt_map.point.z
                
                # Check if this is a known object at this location (distance threshold)
                is_new = True
                if cls_name in self._seen_objects:
                    for known_x, known_y, known_z in self._seen_objects[cls_name]:
                        dist = math.sqrt((map_x - known_x)**2 + (map_y - known_y)**2 + (map_z - known_z)**2)
                        if dist < 1.0:  # 1 meter radius for same object
                            is_new = False
                            break
                            
                if is_new:
                    if cls_name not in self._seen_objects:
                        self._seen_objects[cls_name] = []
                    self._seen_objects[cls_name].append((map_x, map_y, map_z))
                    new_objects_found.append(cls_name)
                    
            except TransformException as ex:
                self.get_logger().debug(f"Could not transform object to map: {ex}")
                continue

        # Trigger TTS for new objects
        now = self.get_clock().now().nanoseconds / 1e9
        for obj in set(new_objects_found):
            # Cooldown of 15 seconds per object class so it doesn't spam
            last_spoken = self._tts_cooldowns.get(obj, 0.0)
            if now - last_spoken > 15.0:
                self._tts_cooldowns[obj] = now
                self.get_logger().info(f"New object mapped: {obj}! Triggering TTS...")
                threading.Thread(target=self._trigger_tts, args=(obj,), daemon=True).start()

        # Publish all seen objects as markers
        self._publish_markers()

    def _trigger_tts(self, object_name: str):
        try:
            # Remove underscores/hyphens for better pronunciation
            clean_name = object_name.replace("_", " ").replace("-", " ")
            requests.post(self._dashboard_url, json={"text": f"I just spotted a {clean_name}"}, timeout=2.0)
        except Exception as e:
            self.get_logger().warn(f"TTS Request failed: {e}")

    def _publish_markers(self):
        marker_array = MarkerArray()
        
        # Color mapping by class
        colors = {
            "person": (0.0, 1.0, 0.0),      # Green
            "chair": (1.0, 0.5, 0.0),       # Orange
            "couch": (1.0, 0.0, 1.0),       # Magenta
            "tv": (0.0, 1.0, 1.0),          # Cyan
            "bottle": (1.0, 1.0, 0.0),      # Yellow
            "default": (1.0, 0.0, 0.0)      # Red
        }
        
        marker_id = 0
        for cls_name, locations in self._seen_objects.items():
            col = colors.get(cls_name.lower(), colors["default"])
            
            for (x, y, z) in locations:
                # 1. Sphere Marker
                sphere = Marker()
                sphere.header.frame_id = "map"
                sphere.header.stamp = self.get_clock().now().to_msg()
                sphere.ns = "semantic_objects"
                sphere.id = marker_id
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position.x = x
                sphere.pose.position.y = y
                sphere.pose.position.z = z
                sphere.scale.x = 0.3
                sphere.scale.y = 0.3
                sphere.scale.z = 0.3
                sphere.color.r = col[0]
                sphere.color.g = col[1]
                sphere.color.b = col[2]
                sphere.color.a = 0.8
                
                # 2. Text Label
                text = Marker()
                text.header.frame_id = "map"
                text.header.stamp = self.get_clock().now().to_msg()
                text.ns = "semantic_labels"
                text.id = marker_id + 10000
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x = x
                text.pose.position.y = y
                text.pose.position.z = z + 0.4
                text.scale.z = 0.2
                text.text = cls_name
                text.color.r = 1.0
                text.color.g = 1.0
                text.color.b = 1.0
                text.color.a = 1.0
                
                marker_array.markers.append(sphere)
                marker_array.markers.append(text)
                marker_id += 1
                
        if marker_array.markers:
            self._marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapperNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
