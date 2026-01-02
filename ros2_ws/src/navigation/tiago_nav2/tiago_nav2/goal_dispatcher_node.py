#!/usr/bin/env python3
import os
import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quat(yaw: float):
    # 2D yaw -> quaternion(z,w)
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class GoalDispatcherNode(Node):
    def __init__(self):
        super().__init__("goal_dispatcher_node")

        # launch에서 넘기는 파라미터명 그대로
        self.declare_parameter("destination_topic", "/delivery/destination_id")
        self.declare_parameter("goals_yaml", "")
        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter("ignore_empty", True)
        self.declare_parameter("dedup_same_goal", True)

        self.destination_topic = self.get_parameter("destination_topic").value
        self.goals_yaml = self.get_parameter("goals_yaml").value
        self.action_name = self.get_parameter("action_name").value
        self.ignore_empty = bool(self.get_parameter("ignore_empty").value)
        self.dedup_same_goal = bool(self.get_parameter("dedup_same_goal").value)

        self.goals = {}
        self._load_goals()

        self.client = ActionClient(self, NavigateToPose, self.action_name)

        self.last_dest = None

        self.sub = self.create_subscription(String, self.destination_topic, self._on_dest, 10)

        self.get_logger().info(f"Subscribed: {self.destination_topic}")
        self.get_logger().info(f"Goals yaml: {self.goals_yaml}")
        self.get_logger().info(f"Action: {self.action_name}")
        self.get_logger().info(f"Rooms: {list(self.goals.keys())}")

    def _load_goals(self):
        if not self.goals_yaml or not os.path.exists(self.goals_yaml):
            self.get_logger().warn(f"goals_yaml not found: {self.goals_yaml}")
            self.goals = {}
            return

        with open(self.goals_yaml, "r") as f:
            data = yaml.safe_load(f) or {}

        rooms = data.get("rooms", {})
        self.goals = rooms if isinstance(rooms, dict) else {}

    def _on_dest(self, msg: String):
        dest = (msg.data or "").strip()

        if self.ignore_empty and dest == "":
            return

        if self.dedup_same_goal and self.last_dest == dest:
            return

        if dest not in self.goals:
            self.get_logger().warn(f"Unknown destination_id='{dest}' (not in goals.yaml)")
            self.last_dest = dest
            return

        pose = self._make_pose(dest)
        self._send_goal(dest, pose)
        self.last_dest = dest

    def _make_pose(self, dest: str) -> PoseStamped:
        g = self.goals[dest]
        frame_id = g.get("frame_id", "map")
        x = float(g.get("x", 0.0))
        y = float(g.get("y", 0.0))
        yaw = float(g.get("yaw", 0.0))

        qz, qw = yaw_to_quat(yaw)

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _send_goal(self, dest: str, pose: PoseStamped):
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("NavigateToPose action server not available.")
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"Send goal room {dest} -> x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}")

        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected.")
            return

        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status
        self.get_logger().info(f"Goal finished. status={status}")


def main(args=None):
    rclpy.init(args=args)
    node = GoalDispatcherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
