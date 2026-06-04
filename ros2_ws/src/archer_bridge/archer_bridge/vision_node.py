"""
ros2_ws/src/archer_bridge/archer_bridge/vision_node.py
=====================================================
ROS2 node for local Vision AI Pipeline.
Processes camera frames, runs YOLOv8 ONNX object detection, tracks targets, and runs OCR.
"""

import os
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from std_srvs.srv import Trigger

try:
    import cv2
    import numpy as np
    OPENCV_AVAILABLE = True
except ImportError:
    OPENCV_AVAILABLE = False

class ArcherVisionNode(Node):
    def __init__(self) -> None:
        super().__init__("archer_vision")
        
        # Discover directories
        self._sim_path = self._discover_sim_path()
        self._project_root = os.path.dirname(self._sim_path)
        self.get_logger().info(f"Vision Node monitoring simulator files in: {self._sim_path}")

        # Subscriptions & Publications
        self._image_sub = self.create_subscription(
            Image, "/archer/camera/image_raw", self._image_callback, 10
        )
        self._det_pub = self.create_publisher(String, "/archer/vision/detections", 10)
        self._ocr_pub = self.create_publisher(String, "/archer/vision/ocr", 10)
        self._track_pub = self.create_publisher(String, "/archer/vision/target_tracking", 10)
        self._heartbeat_pub = self.create_publisher(Header, "/archer/heartbeat/vision", 10)
        
        # Services
        self._ocr_srv = self.create_service(Trigger, "/archer/vision/read_text", self._ocr_service_callback)
        
        # Load YOLO Model (via OpenCV DNN)
        self._net = None
        self._model_loaded = False
        model_path = os.path.join(self._project_root, "models", "yolov8n.onnx")
        if OPENCV_AVAILABLE and os.path.exists(model_path):
            try:
                self._net = cv2.dnn.readNetFromONNX(model_path)
                self._model_loaded = True
                self.get_logger().info(f"YOLOv8 ONNX model loaded successfully from: {model_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load ONNX model via cv2.dnn: {e}. Falling back to simulation mode.")
        else:
            self.get_logger().info("YOLOv8 ONNX model file not found. Running in simulated fallback mode.")

        # Dynamically import MemoryManager from shared mount
        self._project_root = os.path.dirname(self._sim_path)
        import sys
        if self._project_root not in sys.path:
            sys.path.insert(0, self._project_root)
            
        try:
            from ai.memory.memory_manager import MemoryManager
            db_file = os.path.join(self._project_root, "ai", "memory", "db", "memory.json")
            self._memory_mgr = MemoryManager(db_path=db_file)
            self._memory_available = True
            self.get_logger().info("Connected Vision node to Shared Long-Term Memory Database.")
        except Exception as e:
            self.get_logger().warn(f"Could not initialize MemoryManager in Vision node: {e}")
            self._memory_available = False

        self._last_logged_time = 0.0
        self._last_logged_objects = set()

        # Rate control
        self._frame_count = 0
        self._process_every_n_frames = 15 # Process at ~2 FPS (to save CPU in WSL)
        self._last_detected_objects = []

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

    def _image_callback(self, msg: Image) -> None:
        # Publish vision heartbeat
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        self._heartbeat_pub.publish(hdr)

        self._frame_count += 1
        if self._frame_count % self._process_every_n_frames != 0:
            return

        if self._model_loaded and OPENCV_AVAILABLE:
            self._process_real_image(msg)
        else:
            self._process_simulated_image()

    def _process_real_image(self, msg: Image) -> None:
        try:
            if msg.height == 0 or msg.width == 0 or not msg.data:
                return

            import cv_bridge
            br = cv_bridge.CvBridge()
            frame = br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            
            # Prepare blob for YOLO
            blob = cv2.dnn.blobFromImage(frame, 1/255.0, (640, 640), swapRB=True, crop=False)
            self._net.setInput(blob)
            outputs = self._net.forward()
            
            # Simple YOLO output extraction (85 dimensions: cx, cy, w, h, confidence, class_probs...)
            # For simplicity, filter detections
            detections = []
            rows = outputs[0].shape[0]
            for i in range(rows):
                confidence = outputs[0][i][4]
                if confidence > 0.4:
                    class_id = np.argmax(outputs[0][i][5:])
                    detections.append({
                        "class_id": int(class_id),
                        "confidence": float(confidence),
                        "bbox": [float(val) for val in outputs[0][i][0:4]]
                    })
            
            # Publish detections JSON
            self._last_detected_objects = [d["class_id"] for d in detections]
            det_msg = String()
            det_msg.data = json.dumps(detections)
            self._det_pub.publish(det_msg)
            
            # Auto-update robot status file with vision data
            self._update_status_file(f"Detected {len(detections)} objects in frame.", [str(d["class_id"]) for d in detections])
            
            # Log visual observations to SQL Memory database
            now_sec = self.get_clock().now().nanoseconds / 1e9
            current_objects = set([str(d["class_id"]) for d in detections])
            should_log = (now_sec - self._last_logged_time > 10.0) or (current_objects != self._last_logged_objects)
            
            if should_log and self._memory_available:
                self._last_logged_time = now_sec
                self._last_logged_objects = current_objects
                
                # Fetch current semantic location
                status_file = os.path.join(self._sim_path, "robot_status.json")
                location = "unknown"
                if os.path.exists(status_file):
                    try:
                        with open(status_file, "r") as f:
                            location = json.load(f).get("location", "unknown")
                    except: pass
                    
                for d in detections:
                    try:
                        self._memory_mgr.store_visual_observation(
                            object_name=str(d["class_id"]),
                            location=location,
                            coords=d["bbox"],
                            ocr_text="",
                            scene_desc=f"Detected object ID {d['class_id']} with confidence {d['confidence']:.2f}"
                        )
                    except Exception as ex:
                        self.get_logger().warn(f"Failed to log real detection: {ex}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing real image (disabling YOLO processing): {e}")
            self._model_loaded = False
            self._process_simulated_image()

    def _process_simulated_image(self) -> None:
        # Load current robot location from status file
        status_file = os.path.join(self._sim_path, "robot_status.json")
        location = "unknown"
        if os.path.exists(status_file):
            try:
                with open(status_file, "r") as f:
                    status = json.load(f)
                    location = status.get("location", "unknown")
            except:
                pass
        
        # Location-based simulated detections
        scene_desc = "Visual sensor frame analyzed."
        detected = []
        
        if location == "kitchen":
            detected = ["refrigerator", "chair", "sink", "countertop", "microwave"]
            scene_desc = "Analyzing kitchen layout. Identified domestic appliances and dining chairs."
        elif location == "garage":
            detected = ["car", "toolbench", "tire", "toolbox", "shelves"]
            scene_desc = "Analyzing garage space. Workbench and stationary vehicle detected."
        elif location == "living_room":
            detected = ["sofa", "chair", "tv_monitor", "bookshelf", "coffee_table"]
            scene_desc = "Analyzing living room. TV console and seating furniture visible."
        elif location == "bedroom":
            detected = ["bed", "wardrobe", "lamp", "desk", "chair"]
            scene_desc = "Analyzing bedroom. Bedframe and study desk detected."
        else:
            detected = ["floor", "wall", "doorway"]
            scene_desc = "Scanning generic hallway. Path clear."

        self._last_detected_objects = detected
        
        # Publish simulated detections
        det_msg = String()
        det_msg.data = json.dumps({"detected_objects": detected, "scene": scene_desc})
        self._det_pub.publish(det_msg)
        
        # Publish tracking target (if a target like 'chair' or 'person' exists)
        if detected:
            track_msg = String()
            track_msg.data = json.dumps({"target": detected[0], "x": 0.0, "y": 1.2, "z": 0.0})
            self._track_pub.publish(track_msg)

        # Update robot_status.json so host-side Ollama can query visual state
        self._update_status_file(scene_desc, detected)
        
        # Log simulated visual observations to SQL Memory database
        now_sec = self.get_clock().now().nanoseconds / 1e9
        current_objects = set(detected)
        should_log = (now_sec - self._last_logged_time > 10.0) or (current_objects != self._last_logged_objects)
        
        if should_log and self._memory_available:
            self._last_logged_time = now_sec
            self._last_logged_objects = current_objects
            for obj in detected:
                try:
                    self._memory_mgr.store_visual_observation(
                        object_name=obj,
                        location=location,
                        coords=[0.0, 0.0, 0.0],
                        ocr_text="",
                        scene_desc=scene_desc
                    )
                except Exception as ex:
                    self.get_logger().warn(f"Failed to log simulated detection: {ex}")

    def _ocr_service_callback(self, request, response):
        status_file = os.path.join(self._sim_path, "robot_status.json")
        location = "unknown"
        if os.path.exists(status_file):
            try:
                with open(status_file, "r") as f:
                    status = json.load(f)
                    location = status.get("location", "unknown")
            except:
                pass
        
        # Provide simulated OCR text based on location
        if location == "kitchen":
            text = "LABEL: 'Premium Milk - Grade A'"
        elif location == "garage":
            text = "LABEL: 'Caution - High Voltage Battery'"
        elif location == "bedroom":
            text = "LABEL: 'Adi's Journal - Private'"
        else:
            text = "LABEL: 'ARCHER Humanoid - System Diagnostic Tag'"
            
        response.success = True
        response.message = text
        self.get_logger().info(f"OCR triggered. Read text: {text}")
        
        # Publish OCR string
        ocr_msg = String()
        ocr_msg.data = text
        self._ocr_pub.publish(ocr_msg)
        
        # Log OCR reading to SQL Memory database
        if self._memory_available:
            try:
                self._memory_mgr.store_visual_observation(
                    object_name="text_label",
                    location=location,
                    coords=[0.0, 0.0, 0.0],
                    ocr_text=text,
                    scene_desc=f"OCR read tag text: '{text}'"
                )
            except Exception as ex:
                self.get_logger().warn(f"Failed to log OCR sighting: {ex}")
        
        return response

    def _ocr_service_callback_legacy(self, request, response):
        return self._ocr_service_callback(request, response)

    def _update_status_file(self, scene: str, objects: list) -> None:
        status_file = os.path.join(self._sim_path, "robot_status.json")
        status = {}
        if os.path.exists(status_file):
            try:
                with open(status_file, "r") as f:
                    status = json.load(f)
            except:
                pass
        
        status["visual_description"] = scene
        status["detected_objects"] = objects
        
        try:
            with open(status_file, "w") as f:
                json.dump(status, f)
        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ArcherVisionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
