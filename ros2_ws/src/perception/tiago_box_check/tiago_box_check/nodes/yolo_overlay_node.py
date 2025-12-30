#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from interfaces.msg import BoxDetection2D


class YoloOverlayNode(Node):
    def __init__(self):
        super().__init__("yolo_overlay_node")

        self.declare_parameter("image_topic", "/gemini2/color/image_raw")
        self.declare_parameter("bbox_topic", "/perception/box_detection_2d")
        self.declare_parameter("overlay_topic", "/perception/box_detection_overlay")

        self.image_topic = self.get_parameter("image_topic").value
        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.overlay_topic = self.get_parameter("overlay_topic").value

        self.bridge = CvBridge()
        self.last_image_msg = None

        self.create_subscription(Image, self.image_topic, self.image_cb, 10)
        self.create_subscription(BoxDetection2D, self.bbox_topic, self.bbox_cb, 10)
        self.pub = self.create_publisher(Image, self.overlay_topic, 10)

        self.get_logger().info("YOLO Overlay Node ready.")
        self.get_logger().info(f"  image_topic  : {self.image_topic}")
        self.get_logger().info(f"  bbox_topic   : {self.bbox_topic}")
        self.get_logger().info(f"  overlay_topic: {self.overlay_topic}")

    def image_cb(self, msg: Image):
        self.last_image_msg = msg

    def bbox_cb(self, det: BoxDetection2D):
        if self.last_image_msg is None:
            return

        cv_img = self.bridge.imgmsg_to_cv2(self.last_image_msg, desired_encoding="bgr8")

        x1, y1, x2, y2 = int(det.x1), int(det.y1), int(det.x2), int(det.y2)
        if x2 < x1:
            x1, x2 = x2, x1
        if y2 < y1:
            y1, y2 = y2, y1

        h, w = cv_img.shape[:2]
        x1 = max(0, min(w - 1, x1))
        x2 = max(0, min(w - 1, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(0, min(h - 1, y2))

        label = f"{det.label} {float(det.confidence):.2f}"
        cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(cv_img, label, (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        out_msg.header = self.last_image_msg.header
        self.pub.publish(out_msg)


def main():
    rclpy.init()
    node = YoloOverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
