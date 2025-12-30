#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer

from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2

from interfaces.msg import BoxDetection2D, BoxPose3D


def load_extrinsic_4x4(path: str) -> np.ndarray:
    """
    extrinsic txt format (16 numbers) or npy
    - txt: 4 lines x 4 cols or single line 16 floats
    - npy: numpy (4,4)
    """
    if path.endswith(".npy"):
        M = np.load(path)
        return M.astype(np.float32)

    # txt
    with open(path, "r") as f:
        vals = []
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            vals.extend([float(x) for x in line.replace(",", " ").split()])
    if len(vals) != 16:
        raise ValueError(f"Extrinsic txt must have 16 floats, got {len(vals)}")
    M = np.array(vals, dtype=np.float32).reshape(4, 4)
    return M


def project_points(K, pts_cam):
    # pts_cam: (N,3) in camera frame (X,Y,Z)
    X = pts_cam[:, 0]
    Y = pts_cam[:, 1]
    Z = pts_cam[:, 2]
    u = K[0, 0] * (X / Z) + K[0, 2]
    v = K[1, 1] * (Y / Z) + K[1, 2]
    return u, v


def transform_points(T, pts):
    # T: (4,4), pts: (N,3)
    ones = np.ones((pts.shape[0], 1), dtype=np.float32)
    p4 = np.hstack([pts.astype(np.float32), ones])
    out = (T @ p4.T).T
    return out[:, :3]


class Box3DEstimatorReal(Node):
    """
    Real estimator (topic names are parameters)
    Inputs:
      - bbox (BoxDetection2D)
      - camera_info (CameraInfo) -> intrinsics K
      - lidar points (PointCloud2) in lidar frame
    Params:
      - T_lidar_to_cam_path : extrinsic (4x4) file
      - T_cam_to_base_path  : extrinsic (4x4) file
    Output:
      - BoxPose3D in base_link
    """
    def __init__(self):
        super().__init__("box_3d_estimator_real")

        # topics (all param)
        self.declare_parameter("bbox_topic", "/perception/box_detection_2d")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("points_topic", "/lidar/points")
        self.declare_parameter("output_topic", "/perception/box_pose_3d")

        # frames (for output header only)
        self.declare_parameter("base_frame", "base_link")

        # extrinsic files (required in real mode)
        self.declare_parameter("T_lidar_to_cam_path", "")
        self.declare_parameter("T_cam_to_base_path", "")

        # filters
        self.declare_parameter("min_points_in_bbox", 20)
        self.declare_parameter("max_depth_m", 5.0)

        self.bbox_topic = self.get_parameter("bbox_topic").value
        self.caminfo_topic = self.get_parameter("camera_info_topic").value
        self.points_topic = self.get_parameter("points_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.base_frame = self.get_parameter("base_frame").value

        self.min_points = int(self.get_parameter("min_points_in_bbox").value)
        self.max_depth = float(self.get_parameter("max_depth_m").value)

        T_lidar_to_cam_path = self.get_parameter("T_lidar_to_cam_path").value
        T_cam_to_base_path = self.get_parameter("T_cam_to_base_path").value

        if not T_lidar_to_cam_path or not T_cam_to_base_path:
            self.get_logger().error("Extrinsic paths are required: T_lidar_to_cam_path, T_cam_to_base_path")
            self.get_logger().error("Run with --ros-args -p T_lidar_to_cam_path:=... -p T_cam_to_base_path:=...")
            raise RuntimeError("Missing extrinsic paths")

        self.T_lidar_to_cam = load_extrinsic_4x4(T_lidar_to_cam_path)
        self.T_cam_to_base = load_extrinsic_4x4(T_cam_to_base_path)

        self.K = None  # intrinsics 3x3

        # pubs
        self.pub = self.create_publisher(BoxPose3D, self.output_topic, 10)

        # sync subs
        self.sub_bbox = Subscriber(self, BoxDetection2D, self.bbox_topic)
        self.sub_caminfo = Subscriber(self, CameraInfo, self.caminfo_topic)
        self.sub_points = Subscriber(self, PointCloud2, self.points_topic)

        self.sync = ApproximateTimeSynchronizer([self.sub_bbox, self.sub_caminfo, self.sub_points],
                                                queue_size=10, slop=0.20)
        self.sync.registerCallback(self.cb)

        self.get_logger().info("Box3DEstimator REAL ready.")
        self.get_logger().info(f"  bbox_topic   : {self.bbox_topic}")
        self.get_logger().info(f"  caminfo_topic: {self.caminfo_topic}")
        self.get_logger().info(f"  points_topic : {self.points_topic}")
        self.get_logger().info(f"  output_topic : {self.output_topic}")
        self.get_logger().info(f"  base_frame   : {self.base_frame}")

    def cb(self, bbox: BoxDetection2D, caminfo: CameraInfo, cloud: PointCloud2):
        # intrinsics K
        K = np.array(caminfo.k, dtype=np.float32).reshape(3, 3)
        self.K = K

        x1, y1, x2, y2 = bbox.x1, bbox.y1, bbox.x2, bbox.y2
        if x2 <= x1 or y2 <= y1:
            return

        # read points from lidar
        pts = []
        for p in point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            pts.append([float(p[0]), float(p[1]), float(p[2])])
        if len(pts) == 0:
            return

        pts_lidar = np.array(pts, dtype=np.float32)

        # lidar -> camera
        pts_cam = transform_points(self.T_lidar_to_cam, pts_lidar)

        # filter in front of camera & depth
        Z = pts_cam[:, 2]
        mask = (Z > 0.05) & (Z < self.max_depth)
        pts_cam = pts_cam[mask]
        if pts_cam.shape[0] == 0:
            return

        # project to image
        u, v = project_points(self.K, pts_cam)

        # bbox filter
        in_mask = (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)
        pts_in = pts_cam[in_mask]

        if pts_in.shape[0] < self.min_points:
            self.get_logger().info(f"Not enough pts in bbox: {pts_in.shape[0]}")
            return

        # robust representative (median)
        Xc, Yc, Zc = np.median(pts_in, axis=0)

        # camera -> base
        p_base = transform_points(self.T_cam_to_base, np.array([[Xc, Yc, Zc]], dtype=np.float32))[0]

        out = BoxPose3D()
        out.header.stamp = bbox.header.stamp
        out.header.frame_id = self.base_frame
        out.label = bbox.label if bbox.label else "box"
        out.confidence = float(bbox.confidence)

        out.x = float(p_base[0])
        out.y = float(p_base[1])
        out.z = float(p_base[2])
        out.range = float(np.linalg.norm(p_base))

        out.x1, out.y1, out.x2, out.y2 = x1, y1, x2, y2

        self.pub.publish(out)
        self.get_logger().info(
            f"Published 3D base: ({out.x:.2f},{out.y:.2f},{out.z:.2f}) pts={pts_in.shape[0]}"
        )


def main():
    rclpy.init()
    node = Box3DEstimatorReal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
