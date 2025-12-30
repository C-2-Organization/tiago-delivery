#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from interfaces.msg import BoxPose3D
from interfaces.action import PickPlace
from geometry_msgs.msg import PoseStamped


class BoxPickPlaceManager(Node):
    def __init__(self):
        super().__init__("box_pick_place_manager")

        self.state = "WAIT_BOX"
        self.goal_in_flight = False

        self.sub = self.create_subscription(
            BoxPose3D,
            "/perception/box_pose_3d",
            self.on_box_pose,
            10
        )

        self.client = ActionClient(self, PickPlace, "/manipulation/pick_place")

        # 임시 place pose (나중에 table 좌표로 교체)
        self.place_pose = PoseStamped()
        self.place_pose.header.frame_id = "base_link"
        self.place_pose.pose.position.x = 0.50
        self.place_pose.pose.position.y = 0.00
        self.place_pose.pose.position.z = 0.80
        self.place_pose.pose.orientation.w = 1.0

        self.get_logger().info("Orchestrator ready. Waiting /perception/box_pose_3d ...")

    def on_box_pose(self, msg: BoxPose3D):
        if self.goal_in_flight:
            return

        self.get_logger().info(
            f"Box received: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f} (base_link)"
        )

        goal = PickPlace.Goal()
        goal.pick_pose = msg
        goal.place_pose = self.place_pose
        goal.use_gripper = True
        goal.max_retries = 2

        self.send_goal(goal)

    def send_goal(self, goal):
        self.goal_in_flight = True

        if not self.client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("PickPlace action server not available.")
            self.goal_in_flight = False
            return

        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_feedback(self, fb_msg):
        fb = fb_msg.feedback
        self.get_logger().info(f"[FB] stage={fb.stage}, progress={fb.progress:.2f}")

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            self.goal_in_flight = False
            return

        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: success={result.success}, msg={result.message}")
        self.goal_in_flight = False


def main():
    rclpy.init()
    node = BoxPickPlaceManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
