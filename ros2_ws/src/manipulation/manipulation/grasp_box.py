#!/usr/bin/env python3
import math
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from interfaces.action import GraspBox

from tf2_ros import Buffer, TransformListener

from manipulation.pose_utils import transform_pose, offset_along_pose_x, is_zero_quat
from manipulation.gripper_control import GripperClient

# ✅ MoveIt move_action action
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState,
)
from shape_msgs.msg import SolidPrimitive


class GraspBoxActionServer(Node):
    def __init__(self):
        super().__init__("grasp_box_action_server")

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ✅ move_action action client
        self._move_action = ActionClient(self, MoveGroup, "/move_action")

        # Params (튜닝)
        self.declare_parameter("planning_frame", "base_link")
        self.declare_parameter("ee_link_right", "gripper_right_grasping_frame")
        self.declare_parameter("ee_link_left", "gripper_left_grasping_frame")

        # planning group names
        self.declare_parameter("group_right", "arm_right")
        self.declare_parameter("group_left", "arm_left")

        # gripper action names (너 controller_manager 네이밍에 맞춰 조정 가능)
        self.declare_parameter("gripper_right_action", "/gripper_right_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_left_action", "/gripper_left_controller/follow_joint_trajectory")

        # finger joints (너가 ros2_control에 쓴 joint 이름 그대로)
        self.declare_parameter(
            "gripper_right_joints",
            ["gripper_right_left_finger_joint", "gripper_right_right_finger_joint"],
        )
        self.declare_parameter(
            "gripper_left_joints",
            ["gripper_left_left_finger_joint", "gripper_left_right_finger_joint"],
        )

        # open/close positions
        self.declare_parameter("gripper_open_pos",  [0.04, 0.04])
        self.declare_parameter("gripper_close_pos", [0.00, 0.00])
        self.declare_parameter("gripper_time_sec", 1.0)

        # motion tuning (move_action request)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("max_vel_scale", 0.2)
        self.declare_parameter("max_acc_scale", 0.2)
        self.declare_parameter("goal_pos_tolerance", 0.01)      # m
        self.declare_parameter("goal_ori_tolerance", 0.05)      # rad

        # Action server
        self._as = ActionServer(
            self,
            GraspBox,
            "grasp_box",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info("GraspBoxActionServer started (move_action action mode).")

    # ----------------------------
    # Action callbacks
    # ----------------------------
    def goal_callback(self, goal: GraspBox.Goal):
        if goal.timeout_sec <= 0.5:
            return GoalResponse.REJECT
        if goal.arm not in ["left", "right"]:
            return GoalResponse.REJECT
        if goal.pregrasp_distance < 0.0 or goal.pregrasp_distance > 1.0:
            return GoalResponse.REJECT
        if goal.lift_distance < 0.0 or goal.lift_distance > 0.5:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Cancel requested.")
        # move_action cancel은 여기서 안 건드리고(간단화), 상위 action만 cancel 처리
        return CancelResponse.ACCEPT

    # ----------------------------
    # Helpers: move_action request
    # ----------------------------
    def _make_pose_constraints(
        self,
        target: PoseStamped,
        ee_link: str,
        pos_tol: float,
        ori_tol: float,
    ) -> Constraints:
        """
        PoseStamped 목표를 move_action MotionPlanRequest의 Constraints로 변환.
        - PositionConstraint: 박스 형태(bounding box)로 tolerance 적용
        - OrientationConstraint: 절대 오차(roll/pitch/yaw tolerance) 적용
        """
        c = Constraints()
        c.name = "ee_pose_goal"

        # position constraint as a small box around target position
        pc = PositionConstraint()
        pc.header = target.header
        pc.link_name = ee_link
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        pc.weight = 1.0

        bv = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        # box dimensions: 2*tol in each axis
        box.dimensions = [2.0 * pos_tol, 2.0 * pos_tol, 2.0 * pos_tol]
        bv.primitives.append(box)
        bv.primitive_poses.append(target.pose)
        pc.constraint_region = bv

        # orientation constraint
        oc = OrientationConstraint()
        oc.header = target.header
        oc.link_name = ee_link
        oc.orientation = target.pose.orientation
        oc.absolute_x_axis_tolerance = ori_tol
        oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0

        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        return c

    async def _move_to_pose(
        self,
        group: str,
        target: PoseStamped,
        ee_link: str,
        timeout_sec: float,
    ) -> bool:
        """
        move_action 액션으로 plan+execute 수행.
        성공하면 True.
        """
        if not self._move_action.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("move_action action server not available")
            return False

        allowed_time = float(self.get_parameter("allowed_planning_time").value)
        max_vel = float(self.get_parameter("max_vel_scale").value)
        max_acc = float(self.get_parameter("max_acc_scale").value)
        pos_tol = float(self.get_parameter("goal_pos_tolerance").value)
        ori_tol = float(self.get_parameter("goal_ori_tolerance").value)

        req = MotionPlanRequest()
        req.group_name = group
        req.num_planning_attempts = 1
        req.allowed_planning_time = allowed_time
        req.max_velocity_scaling_factor = max_vel
        req.max_acceleration_scaling_factor = max_acc

        # ✅ IMPORTANT: start_state는 비워두는 편이 안전 (move_action이 current state 사용)
        req.start_state = RobotState()

        # goal constraints
        goal_c = self._make_pose_constraints(target, ee_link, pos_tol, ori_tol)
        req.goal_constraints = [goal_c]

        goal_msg = MoveGroup.Goal()
        goal_msg.request = req
        goal_msg.planning_options.plan_only = False               # ✅ plan + execute
        goal_msg.planning_options.replan = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.look_around_attempts = 0
        goal_msg.planning_options.max_safe_execution_cost = 0.0

        send_fut = self._move_action.send_goal_async(goal_msg)
        goal_handle = await send_fut
        if not goal_handle.accepted:
            self.get_logger().error(f"move_action goal rejected (group={group})")
            return False

        result_fut = goal_handle.get_result_async()
        try:
            result = await rclpy.task.FutureTimeoutWrapper(result_fut, timeout_sec=timeout_sec)  # type: ignore
        except Exception:
            # rclpy에 FutureTimeoutWrapper 없는 환경도 많아서 fallback
            # -> 아래 방식으로 timeout 체크는 execute_callback에서 전체 timeout으로 처리 권장
            result = await result_fut

        error_code = int(result.result.error_code.val)
        if error_code != 1:  # MoveItErrorCodes.SUCCESS == 1
            self.get_logger().warn(f"move_action failed: error_code={error_code}")
            return False
        return True

    # ----------------------------
    # Main execute
    # ----------------------------
    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        fb = GraspBox.Feedback()
        res = GraspBox.Result()

        start = self.get_clock().now()
        timeout = Duration(seconds=float(goal.timeout_sec))

        planning_frame = str(self.get_parameter("planning_frame").value)

        arm = goal.arm
        group = str(self.get_parameter("group_right").value) if arm == "right" else str(self.get_parameter("group_left").value)
        ee_link = str(self.get_parameter("ee_link_right").value) if arm == "right" else str(self.get_parameter("ee_link_left").value)

        # Gripper client
        gripper_action = str(self.get_parameter("gripper_right_action").value) if arm == "right" else str(self.get_parameter("gripper_left_action").value)
        gripper_joints = list(self.get_parameter("gripper_right_joints").value) if arm == "right" else list(self.get_parameter("gripper_left_joints").value)
        gripper = GripperClient(self, gripper_action, gripper_joints)

        if not gripper.wait(timeout_sec=5.0):
            goal_handle.abort()
            res.success = False
            res.message = f"Gripper action not available: {gripper_action}"
            return res

        def is_timed_out():
            return (self.get_clock().now() - start) > timeout

        try:
            # 1) TF box pose -> planning frame
            fb.state = "TF_BOX_POSE"
            fb.progress = 0.05
            goal_handle.publish_feedback(fb)

            box_pose_pf = transform_pose(self.tf_buffer, goal.box_pose, planning_frame)

            # 2) grasp pose (position=box pos, orientation = goal override or box)
            grasp_pose = PoseStamped()
            grasp_pose.header.frame_id = planning_frame
            grasp_pose.header.stamp = self.get_clock().now().to_msg()
            grasp_pose.pose.position = box_pose_pf.pose.position

            if not is_zero_quat(goal.grasp_orientation):
                grasp_pose.pose.orientation = goal.grasp_orientation
            else:
                grasp_pose.pose.orientation = box_pose_pf.pose.orientation

            # 3) pregrasp pose: back along local X
            pregrasp_pose = offset_along_pose_x(grasp_pose, dx=-float(goal.pregrasp_distance))

            # 4) open gripper
            fb.state = "GRIPPER_OPEN"
            fb.progress = 0.15
            goal_handle.publish_feedback(fb)

            open_pos = list(self.get_parameter("gripper_open_pos").value)
            ok = await gripper.command(open_pos, time_sec=float(self.get_parameter("gripper_time_sec").value))
            if not ok:
                goal_handle.abort()
                res.success = False
                res.message = "Failed to open gripper"
                return res

            if is_timed_out():
                goal_handle.abort()
                res.success = False
                res.message = "Timeout"
                return res

            # 5) move pregrasp
            fb.state = "MOVE_PREGRASP"
            fb.progress = 0.30
            goal_handle.publish_feedback(fb)

            ok = await self._move_to_pose(group, pregrasp_pose, ee_link, timeout_sec=float(goal.timeout_sec))
            if not ok:
                goal_handle.abort()
                res.success = False
                res.message = "move_action failed (pregrasp)"
                return res

            if is_timed_out():
                goal_handle.abort()
                res.success = False
                res.message = "Timeout"
                return res

            # 6) move grasp
            fb.state = "MOVE_GRASP"
            fb.progress = 0.55
            goal_handle.publish_feedback(fb)

            ok = await self._move_to_pose(group, grasp_pose, ee_link, timeout_sec=float(goal.timeout_sec))
            if not ok:
                goal_handle.abort()
                res.success = False
                res.message = "move_action failed (grasp)"
                return res

            # 7) close gripper
            fb.state = "GRIPPER_CLOSE"
            fb.progress = 0.75
            goal_handle.publish_feedback(fb)

            close_pos = list(self.get_parameter("gripper_close_pos").value)
            ok = await gripper.command(close_pos, time_sec=float(self.get_parameter("gripper_time_sec").value))
            if not ok:
                goal_handle.abort()
                res.success = False
                res.message = "Failed to close gripper"
                return res

            if is_timed_out():
                goal_handle.abort()
                res.success = False
                res.message = "Timeout"
                return res

            # 8) lift (+Z in planning frame)
            fb.state = "LIFT"
            fb.progress = 0.90
            goal_handle.publish_feedback(fb)

            lift_pose = PoseStamped()
            lift_pose.header.frame_id = planning_frame
            lift_pose.header.stamp = self.get_clock().now().to_msg()
            lift_pose.pose = grasp_pose.pose
            lift_pose.pose.position.z += float(goal.lift_distance)

            ok = await self._move_to_pose(group, lift_pose, ee_link, timeout_sec=float(goal.timeout_sec))
            if not ok:
                goal_handle.abort()
                res.success = False
                res.message = "move_action failed (lift)"
                return res

            # done
            fb.state = "DONE"
            fb.progress = 1.0
            goal_handle.publish_feedback(fb)

            goal_handle.succeed()
            res.success = True
            res.message = "Grasp sequence completed"
            res.final_ee_pose = lift_pose
            return res

        except Exception as e:
            goal_handle.abort()
            res.success = False
            res.message = f"Exception: {e}"
            return res


def main():
    rclpy.init()
    node = GraspBoxActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
