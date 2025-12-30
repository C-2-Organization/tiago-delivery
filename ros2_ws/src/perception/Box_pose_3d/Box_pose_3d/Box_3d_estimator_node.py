#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from interfaces.msg import BoxDetection2D, BoxPose3D


class Box3DEstimatorStub(Node):
    """
    로봇/시뮬 없이도 테스트 가능한 Stub.
    - 입력: BoxDetection2D (2D bbox)
    - 출력: BoxPose3D (base_link 기준 가짜 3D)
    실제 환경 붙일 때:
      - 이 stub을 'PointCloud2/LaserScan + TF + calib' 기반 estimator로 교체
      - 토픽명/프레임명은 파라미터로만 바꾸면 됨
    """
    def __init__(self):
        super().__init__("box_3d_estimator_stub")

        # 토픽명 파라미터화
        self.declare_parameter("bbox_topic", "/perception/box_detection_2d")
        self.declare_parameter("output_topic", "/perception/box_pose_3d")
        self.declare_parameter("base_frame", "base_link")

        # 테스트용 가정 파라미터
        self.declare_parameter("assumed_range_m", 1.0)   # 라이다/뎁스 없을 때 임시 거리
        self.declare_parameter("assumed_z_m", 0.75)      # 테이블 높이 가정

        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.base_frame = self.get_parameter("base_frame").value

        self.assumed_range = float(self.get_parameter("assumed_range_m").value)
        self.assumed_z = float(self.get_parameter("assumed_z_m").value)

        self.sub = self.create_subscription(BoxDetection2D, self.bbox_topic, self.cb, 10)
        self.pub = self.create_publisher(BoxPose3D, self.output_topic, 10)

        self.get_logger().info("Box3D Estimator STUB ready.")
        self.get_logger().info(f"  bbox_topic   : {self.bbox_topic}")
        self.get_logger().info(f"  output_topic : {self.output_topic}")
        self.get_logger().info(f"  base_frame   : {self.base_frame}")

    def cb(self, bbox: BoxDetection2D):
        # bbox 중심을 이용해 "가짜 3D" 생성 (테스트용)
        cx = (bbox.x1 + bbox.x2) * 0.5
        cy = (bbox.y1 + bbox.y2) * 0.5

        out = BoxPose3D()
        out.header.stamp = bbox.header.stamp
        out.header.frame_id = self.base_frame
        out.label = bbox.label if bbox.label else "box"
        out.confidence = float(bbox.confidence)

        # 여기 값들은 테스트용. 실제 환경에서는 라이다/캘리브레이션으로 계산한 값으로 대체.
        out.x = float(self.assumed_range)   # 전방 거리 가정
        out.y = 0.0                         # 중앙이라고 가정
        out.z = float(self.assumed_z)       # 테이블 높이 가정
        out.range = float(self.assumed_range)

        out.x1, out.y1, out.x2, out.y2 = bbox.x1, bbox.y1, bbox.x2, bbox.y2

        self.pub.publish(out)
        self.get_logger().info(
            f"STUB publish BoxPose3D: x={out.x:.2f}, y={out.y:.2f}, z={out.z:.2f} (from bbox center {cx:.1f},{cy:.1f})"
        )


def main():
    rclpy.init()
    node = Box3DEstimatorStub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
