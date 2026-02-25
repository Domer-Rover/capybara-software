#!/usr/bin/env python3
"""YOLOv8 object detector ROS2 node.

Subscribes to the ZED left camera image, runs YOLOv8 inference, and publishes:
  - /yolo/image_debug   (sensor_msgs/Image  – annotated image with bounding boxes)

Default model: yolov8n-oiv7.onnx  (Open Images v7 – detects "Bottle" and "Hammer")
Target classes default to ['Bottle', 'Hammer'] matching OIV7 class names.

Usage examples (ros2 run or launch arg):
  # Water bottles + hammers (OIV7 model, default)
  ros2 run capybara_bringup yolov8_detector

  # Custom classes / model
  ros2 run capybara_bringup yolov8_detector \
    --ros-args -p model_path:=/path/to/model.onnx \
               -p target_classes:="['Bottle','Hammer']"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    raise SystemExit(
        '[yolov8_detector] ultralytics package not found. '
        'Install it with:  pip install ultralytics'
    )

# ── colour palette (BGR) – one colour per tracked class name ──────────────────
_CLASS_COLORS = {
    'Bottle':  (0, 200, 255),   # amber-orange  → water bottles (OIV7)
    'Hammer':  (80,  80, 255),  # red           → hammers       (OIV7)
}
_DEFAULT_COLOR = (0, 255, 0)   # green for any other matched class


class Yolov8Detector(Node):
    def __init__(self):
        super().__init__('yolov8_detector')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('model_path', '/home/jetsonson/capybara-software/models/yolov8n-oiv7.onnx')
        self.declare_parameter('target_classes', ['Bottle', 'Hammer'])
        self.declare_parameter('confidence_threshold', 0.40)
        self.declare_parameter('image_topic', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('camera_frame', 'zed_left_camera_frame_optical')

        model_path = self.get_parameter('model_path').value
        self.target_classes: list = self.get_parameter('target_classes').value
        self.conf_thresh: float = self.get_parameter('confidence_threshold').value
        image_topic: str = self.get_parameter('image_topic').value

        # ── Load model ────────────────────────────────────────────────────────
        self.get_logger().info(f'Loading YOLOv8 model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info(
            f'Model loaded. Target classes: {self.target_classes}  '
            f'(conf >= {self.conf_thresh})'
        )

        self._warn_missing_classes()

        # ── ROS2 I/O ──────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Image, image_topic, self._image_cb, qos)
        self.debug_pub = self.create_publisher(Image, '/yolo/image_debug', 5)

        self.get_logger().info(
            f'YOLOv8 detector ready – subscribed to {image_topic}'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _warn_missing_classes(self):
        """Warn if a requested class is not in the loaded model's names."""
        model_names = set(self.model.names.values())
        for cls in self.target_classes:
            if cls not in model_names:
                self.get_logger().warn(
                    f'Class "{cls}" not found in model. '
                    f'Available classes (sample): '
                    f'{list(model_names)[:20]}. '
                    f'For "Bottle"/"Hammer" use yolov8n-oiv7.onnx (OIV7 model).'
                )

    def _image_cb(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(cv_image, conf=self.conf_thresh, verbose=False)[0]
        annotated = self._draw_detections(cv_image, results)

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = msg.header
        self.debug_pub.publish(out_msg)

    def _draw_detections(self, image: np.ndarray, results) -> np.ndarray:
        """Draw bounding boxes and labels for target classes only."""
        overlay = image.copy()

        for box in results.boxes:
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]

            if cls_name not in self.target_classes:
                continue

            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            color = _CLASS_COLORS.get(cls_name, _DEFAULT_COLOR)

            # Semi-transparent filled rectangle
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, cv2.FILLED)

        # Blend overlay
        cv2.addWeighted(overlay, 0.25, image, 0.75, 0, image)

        # Draw opaque borders and labels on top
        for box in results.boxes:
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]

            if cls_name not in self.target_classes:
                continue

            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            color = _CLASS_COLORS.get(cls_name, _DEFAULT_COLOR)

            # Border
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

            # Label
            display_name = 'water bottle' if cls_name == 'Bottle' else cls_name.lower()
            label = f'{display_name}  {conf:.0%}'
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.55
            thickness = 2
            (tw, th), bl = cv2.getTextSize(label, font, font_scale, thickness)
            pad = 3

            # Label background
            lx1 = x1
            ly1 = y1 - th - pad * 2
            lx2 = x1 + tw + pad * 2
            ly2 = y1
            # Keep label inside frame
            if ly1 < 0:
                ly1, ly2 = y2, y2 + th + pad * 2

            cv2.rectangle(image, (lx1, ly1), (lx2, ly2), color, cv2.FILLED)
            cv2.putText(
                image, label,
                (lx1 + pad, ly2 - pad),
                font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA,
            )

        return image


def main(args=None):
    rclpy.init(args=args)
    node = Yolov8Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
