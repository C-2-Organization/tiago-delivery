#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from interfaces.action import PickPlace


class PickPlaceActionServer(Node):
    def __init__(self):
        super().__init__("pick_place_action_server")
        self.server = ActionServer(
            self,
            PickPlace,
            "/manipulation/pick_place",
            self.execute_cb
        )
        self.get_logger().info("PickPlace Action Server ready.")

    async def execute_cb(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(
            f"Goal: pick({goal.pick_pose.x:.3f},{goal.pick_pose.y:.3f},{goal.pick_pose.z:.3f})"
        )

        fb = PickPlace.Feedback()
        fb.stage = "planning"
        fb.progress = 0.2
        goal_handle.publish_feedback(fb)

        fb.stage = "grasping"
        fb.progress = 0.6
        goal_handle.publish_feedback(fb)

        fb.stage = "placing"
        fb.progress = 0.9
        goal_handle.publish_feedback(fb)

        result = PickPlace.Result()
        result.success = True
        result.message = "Pick&Place stub done."
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    node = PickPlaceActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
