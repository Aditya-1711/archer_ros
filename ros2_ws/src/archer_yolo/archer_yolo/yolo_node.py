#!/usr/bin/env python3
"""
ros2_ws/src/archer_yolo/archer_yolo/yolo_node.py
=================================================
ROS2 node that runs YOLOv8 object detection on incoming camera images
and publishes detected objects as JSON strings.

Subscriptions
  /image_raw  (sensor_msgs/msg/Image)  — remapped to /archer/camera/image_raw

Publications
  /yolo/detections  (std_msgs/msg/String)  — JSON list of detections
  /yolo/image       (sensor_msgs/msg/Image) — annotated image (debug)
"""

import json
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Lazy-import ultralytics so the node starts even if the model is not found yet
try:
    from ultralytics import YOLO as _YOLO
    _ULTRALYTICS_AVAILABLE = True
except ImportError:
    _ULTRALYTICS_AVAILABLE = False


class YoloNode(Node):
    def __init__(self) -> None:
        super().__init__('yolo_node')

        # Parameters
        self.declare_parameter('model_path', os.path.expanduser('~/archer_ros/models/yolov8n.pt'))
        self.declare_parameter('conf_thresh', 0.4)
        self.declare_parameter('device', 'cpu')

        model_path  = self.get_parameter('model_path').get_parameter_value().string_value
        conf_thresh = self.get_parameter('conf_thresh').get_parameter_value().double_value
        device      = self.get_parameter('device').get_parameter_value().string_value

        self._conf_thresh = conf_thresh

        # Load model
        self._model = None
        if not _ULTRALYTICS_AVAILABLE:
            self.get_logger().error('ultralytics not installed — YOLO node running in stub mode.')
        elif not os.path.isfile(model_path):
            self.get_logger().error(f'Model not found at {model_path} — running in stub mode.')
        else:
            self._model = _YOLO(model_path)
            self._model.to(device)
            self.get_logger().info(f'YOLOv8 model loaded from {model_path} on device={device}')

        # ROS interfaces
        self._img_sub = self.create_subscription(Image, '/image_raw', self._image_callback, 5)
        self._det_pub = self.create_publisher(String, '/detections', 10)
        self._ann_pub = self.create_publisher(Image, '/yolo/image', 5)

        self._last_printed_names = set()
        self.get_logger().info('YOLO node ready — listening on /image_raw')

    # ------------------------------------------------------------------
    def _image_callback(self, msg: Image) -> None:
        """Convert ROS Image → numpy, run inference, publish results."""
        # Convert ROS Image to OpenCV BGR
        try:
            frame = self._ros_image_to_cv2(msg)
        except Exception as exc:
            self.get_logger().warn(f'Image conversion failed: {exc}')
            return

        detections = []
        current_names = set()

        if self._model is not None:
            try:
                results = self._model(frame, conf=self._conf_thresh, verbose=False)
                for result in results:
                    for box in result.boxes:
                        cls_id   = int(box.cls[0].item())
                        cls_name = result.names.get(cls_id, str(cls_id))
                        conf     = float(box.conf[0].item())
                        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                        detections.append({
                            'class_id':   cls_id,
                            'class_name': cls_name,
                            'confidence': round(conf, 3),
                            'bbox':       [round(x1), round(y1), round(x2), round(y2)],
                        })
                        current_names.add(cls_name)
                if current_names and current_names != getattr(self, '_last_printed_names', set()):
                    self.get_logger().info(f"I came across these objects: {', '.join(current_names)}")
                    self._last_printed_names = current_names
                elif not current_names:
                    self._last_printed_names = set()

                # Publish annotated image
                annotated = results[0].plot()
                self._ann_pub.publish(self._cv2_to_ros_image(annotated, msg.header))

            except Exception as exc:
                self.get_logger().warn(f'Inference failed: {exc}')

        # Publish detections JSON
        det_msg = String()
        det_msg.data = json.dumps(detections)
        self._det_pub.publish(det_msg)

    # ------------------------------------------------------------------
    @staticmethod
    def _ros_image_to_cv2(msg: Image) -> np.ndarray:
        """Convert sensor_msgs/Image to BGR numpy array."""
        dtype = np.uint8
        channels = 3 if msg.encoding in ('rgb8', 'bgr8') else 1
        frame = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, channels)
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame

    @staticmethod
    def _cv2_to_ros_image(frame: np.ndarray, header) -> Image:
        """Convert BGR numpy array to sensor_msgs/Image."""
        msg = Image()
        msg.header    = header
        msg.height    = frame.shape[0]
        msg.width     = frame.shape[1]
        msg.encoding  = 'bgr8'
        msg.step      = frame.shape[1] * 3
        msg.data      = frame.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
