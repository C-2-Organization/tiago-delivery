#!/usr/bin/env python3
from dataclasses import dataclass
from typing import Optional, Tuple

import asyncio
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped
from sensor_msgs.msg import CameraInfo

from interfaces.action import DualArmGraspBox
from interfaces.msg import BoxDetection2D

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

import tf2_ros
from builtin_interfaces.msg import Time
from tf2_geometry_msgs import do_transform_point

from control_msgs.action import FollowJointTrajectory


def _now_msg(node: Node):
    return node.get_clock().now().to_msg()


def _identity_quat() -> Quaternion:
    q = Quaternion()
    q.w = 1.0
    return q


@dataclass
class BBox2D:
    x1: int
    y1: int
    x2: int
    y2: int
    confidence: float
    frame_id: str


class DualArmGraspBoxActionServer(Node):
    """
    Center + width 기반 Side Pinch (양옆 누르기) grasp:

    - Sub: /perception/box_point_cam (PointStamped)
        * box 중심에 해당하는 3D 포인트 (camera frame)
    - Sub: /perception/box_detection_2d (BoxDetection2D)
        * bbox pixel width로 박스 폭 추정
    - CameraInfo fx로 pinhole 폭 추정:
        width_m = pixel_w * depth_z_cam / fx
    - center를 TF로 base(planning_frame)로 변환한 뒤,
        left/right 목표를 center 기준 y로 벌려서 생성
    - 시퀀스:
        PREGRASP (바깥 + 뒤로) -> GRASP (안쪽 squeeze) -> LIFT

    NOTE:
    - gripper close는 아직 없음(마찰/물리로 버티는 MVP)
    - OrientationConstraint는 성공률 위해 기본 OFF
    """

    def __init__(self):
        super().__init__("dual_grasp_box_action_server")

        # --- perception ---
        self._last_bbox: Optional[BBox2D] = None
        self.create_subscription(BoxDetection2D, "/perception/box_detection_2d", self._on_bbox, 10)
        
        # --- joints ---
        self._fj_left = ActionClient(self, FollowJointTrajectory, "/arm_left_controller/follow_joint_trajectory")
        self._fj_right = ActionClient(self, FollowJointTrajectory, "/arm_right_controller/follow_joint_trajectory")


        self.declare_parameter("box_point_topic", "/perception/box_point_cam")
        self._last_point_cam: Optional[PointStamped] = None
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("box_point_topic").value),
            self._on_point_cam,
            10,
        )

        # CameraInfo fx
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self._fx: Optional[float] = None
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._on_caminfo,
            10,
        )
        self.declare_parameter("fx_fallback", 525.0)

        # planning frames / ee links
        self.declare_parameter("planning_frame", "base_link")
        self.declare_parameter("ee_link_left", "gripper_left_grasping_frame")
        self.declare_parameter("ee_link_right", "gripper_right_grasping_frame")

        # TF
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # move_action action name
        self.declare_parameter("move_action_action", "/move_action")

        # move group request tuning
        self.declare_parameter("allowed_planning_time", 8.0)
        self.declare_parameter("num_planning_attempts", 3)
        self.declare_parameter("max_velocity_scaling", 1.0)
        self.declare_parameter("max_acceleration_scaling", 1.0)

        # goal tolerance
        self.declare_parameter("pos_tol", 0.01)   # meters
        self.declare_parameter("ori_tol", 0.35)   # rad (unused if orientation constraint off)

        # side pinch tuning (추가 파라미터)
        self.declare_parameter("pregrasp_backoff_m", 0.15)   # x로 뒤로 빼서 충돌 줄임
        self.declare_parameter("pregrasp_extra_open_m", 0.15)  # y로 더 벌려서 접근
        self.declare_parameter("squeeze_m", 0.02)            # grasp 때 중심쪽으로 더 밀어넣기
        self.declare_parameter("min_half_width_m", 0.02)     # 너무 얇게 추정될 때 안정값
        self.declare_parameter("use_orientation_constraint", False)

        # (선택) reach guard
        self.declare_parameter("max_reach_x", 0.90)
        self.declare_parameter("min_reach_x", 0.20)
        self.declare_parameter("max_reach_z", 1.50)
        self.declare_parameter("min_reach_z", 0.02)
        self.declare_parameter("max_reach_abs_y", 0.60)

        # action server
        self._as = ActionServer(
            self,
            DualArmGraspBox,
            "dual_grasp_box",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # move_action clients (left/right)
        action_name = str(self.get_parameter("move_action_action").value)
        self._mg_left = ActionClient(self, MoveGroup, action_name)
        self._mg_right = ActionClient(self, MoveGroup, action_name)

        self.get_logger().info("DualArmGraspBoxActionServer started (center+width side pinch).")

    # -----------------------
    # Subscribers
    # -----------------------
    def _on_bbox(self, msg: BoxDetection2D):
        self._last_bbox = BBox2D(
            x1=int(msg.x1),
            y1=int(msg.y1),
            x2=int(msg.x2),
            y2=int(msg.y2),
            confidence=float(msg.confidence),
            frame_id=str(msg.header.frame_id),
        )

    def _on_point_cam(self, msg: PointStamped):
        self._last_point_cam = msg

    def _on_caminfo(self, msg: CameraInfo):
        if len(msg.k) >= 1:
            fx = float(msg.k[0])
            if fx > 1.0:
                self._fx = fx
                
    async def _exec_joint_traj(self, arm: str, traj) -> Tuple[bool, str]:
        # traj: moveit_msgs/msg/RobotTrajectory
        if traj is None or traj.joint_trajectory is None or len(traj.joint_trajectory.joint_names) == 0:
            return False, "empty_trajectory"

        client = self._fj_left if arm == "left" else self._fj_right
        if not client.wait_for_server(timeout_sec=2.0):
            return False, "follow_joint_trajectory server not available"

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj.joint_trajectory

        gh = await client.send_goal_async(goal)
        if not gh.accepted:
            return False, "goal rejected"

        res = await gh.get_result_async()
        status = int(res.result.error_code)
        # 0이 SUCCESS인 구현이 일반적이지만 환경마다 다를 수 있음 → 메시지로 반환
        if status == 0:
            return True, "success"
        return False, f"error_code={status}"

    # -----------------------
    # Action callbacks
    # -----------------------
    def goal_callback(self, goal: DualArmGraspBox.Goal):
        if goal.timeout_sec <= 1.0:
            return GoalResponse.REJECT
        if goal.lift_distance_m < 0.0 or goal.lift_distance_m > 0.5:
            return GoalResponse.REJECT
        if goal.pregrasp_outward_m < 0.0 or goal.pregrasp_outward_m > 0.5:
            return GoalResponse.REJECT
        if goal.side_margin_m < 0.0 or goal.side_margin_m > 0.2:
            return GoalResponse.REJECT
        if goal.support_z_offset_m < -0.5 or goal.support_z_offset_m > 0.2:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Cancel requested.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        fb = DualArmGraspBox.Feedback()
        res = DualArmGraspBox.Result()

        start = self.get_clock().now()
        timeout = Duration(seconds=float(goal.timeout_sec))

        planning_frame = str(self.get_parameter("planning_frame").value)
        ee_left = str(self.get_parameter("ee_link_left").value)
        ee_right = str(self.get_parameter("ee_link_right").value)

        def timed_out() -> bool:
            return (self.get_clock().now() - start) > timeout

        def wait_mg_clients() -> bool:
            if not self._mg_left.wait_for_server(timeout_sec=5.0):
                return False
            if not self._mg_right.wait_for_server(timeout_sec=5.0):
                return False
            return True

        # 0) ensure move_action action server available
        fb.state = "WAIT_move_action"
        fb.progress = 0.01
        goal_handle.publish_feedback(fb)

        if not wait_mg_clients():
            goal_handle.abort()
            res.success = False
            res.message = "move_action action server not available"
            return res

        # 1) wait for box_point_cam and bbox
        fb.state = "WAIT_perception"
        fb.progress = 0.05
        goal_handle.publish_feedback(fb)

        point_cam: Optional[PointStamped] = None
        bbox: Optional[BBox2D] = None

        while rclpy.ok() and not timed_out() and not goal_handle.is_cancel_requested:
            point_cam = self._last_point_cam
            bbox = self._last_bbox
            if point_cam is not None and bbox is not None:
                break
            rclpy.spin_once(self, timeout_sec=0.05)

        if goal_handle.is_cancel_requested:
            goal_handle.abort()
            res.success = False
            res.message = "Canceled"
            return res

        if point_cam is None:
            goal_handle.abort()
            res.success = False
            res.message = "No /perception/box_point_cam received (timeout)"
            return res

        if bbox is None:
            goal_handle.abort()
            res.success = False
            res.message = "No /perception/box_detection_2d received (timeout)"
            return res

        # 2) transform center point -> planning_frame
        fb.state = "TF_center_to_planning_frame"
        fb.progress = 0.10
        goal_handle.publish_feedback(fb)

        try:
            src_frame = point_cam.header.frame_id
            if not src_frame:
                raise RuntimeError("box_point_cam.header.frame_id is empty")

            tf = self._tf_buffer.lookup_transform(
                planning_frame,
                src_frame,
                Time(),  # latest
                timeout=Duration(seconds=0.2),
            )

            center_base: PointStamped = do_transform_point(point_cam, tf)

        except Exception as e:
            goal_handle.abort()
            res.success = False
            res.message = f"TF transform failed ({point_cam.header.frame_id} -> {planning_frame}): {e}"
            return res

        cx = float(center_base.point.x)
        cy = float(center_base.point.y)
        cz = float(center_base.point.z)

        # reach guard
        max_x = float(self.get_parameter("max_reach_x").value)
        min_x = float(self.get_parameter("min_reach_x").value)
        max_z = float(self.get_parameter("max_reach_z").value)
        min_z = float(self.get_parameter("min_reach_z").value)
        max_abs_y = float(self.get_parameter("max_reach_abs_y").value)

        if not (min_x <= cx <= max_x) or not (min_z <= cz <= max_z) or (abs(cy) > max_abs_y):
            goal_handle.abort()
            res.success = False
            res.message = f"Target out of reachable workspace: base(x={cx:.3f}, y={cy:.3f}, z={cz:.3f})"
            return res

        # 3) estimate width_m from bbox pixel width
        fb.state = "ESTIMATE_WIDTH"
        fb.progress = 0.15
        goal_handle.publish_feedback(fb)

        width_m = float(goal.box_width_m)
        if width_m <= 0.0:
            pixel_w = max(1, int(bbox.x2) - int(bbox.x1))
            fx = self._fx if self._fx is not None else float(self.get_parameter("fx_fallback").value)

            depth_z_cam = float(point_cam.point.z)
            if depth_z_cam <= 0.01:
                goal_handle.abort()
                res.success = False
                res.message = f"Invalid depth z in box_point_cam: {depth_z_cam}"
                return res

            width_m = float(pixel_w) * depth_z_cam / float(fx)

        fb.estimated_box_width_m = float(width_m)
        goal_handle.publish_feedback(fb)

        # 4) build side pinch targets (center + width)
        # half-width with margin
        min_half = float(self.get_parameter("min_half_width_m").value)
        half_w = max(min_half, 0.5 * float(width_m) - float(goal.side_margin_m))

        # z tweak (이제 support_z_offset은 "높이 미세 조정" 용도로)
        z_target = cz + float(goal.support_z_offset_m)

        # approach tuning
        backoff = float(self.get_parameter("pregrasp_backoff_m").value)
        extra_open = float(self.get_parameter("pregrasp_extra_open_m").value)
        squeeze = float(self.get_parameter("squeeze_m").value)

        # PREGRASP: y를 더 벌리고, x를 살짝 뒤로
        yL_pre = cy + (half_w + extra_open)
        yR_pre = cy - (half_w + extra_open)
        x_pre = cx - max(0.0, backoff)

        # GRASP: 중심쪽으로 squeeze (y를 조금 줄임)
        # 너무 과하면 박스를 관통/충돌하니 clamp
        squeeze_eff = max(0.0, squeeze)
        xL_grasp = cx + 0.15
        xR_grasp = cx + 0.15
        yL_grasp = cy + max(min_half, half_w - squeeze_eff)
        yR_grasp = cy - max(min_half, half_w - squeeze_eff)

        # make pose helper
        q = _identity_quat()

        def mk_pose(x: float, y: float, z: float) -> PoseStamped:
            ps = PoseStamped()
            ps.header.frame_id = planning_frame
            ps.header.stamp = _now_msg(self)
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(z)
            ps.pose.orientation = q
            return ps

        left_pre = mk_pose(x_pre, yL_pre, z_target)
        right_pre = mk_pose(x_pre, yR_pre, z_target)

        left_grasp = mk_pose(xL_grasp, yL_grasp, z_target)
        right_grasp = mk_pose(xR_grasp, yR_grasp, z_target)

        lift = float(goal.lift_distance_m)
        left_lift = mk_pose(xL_grasp, yL_grasp, z_target + lift)
        right_lift = mk_pose(xR_grasp , yR_grasp, z_target + lift)

        fb.left_target_pose = left_grasp
        fb.right_target_pose = right_grasp

        # 5) run sequence
        async def step_send_both(state: str, progress: float, left_pose: PoseStamped, right_pose: PoseStamped) -> Tuple[bool, str]:
            if goal_handle.is_cancel_requested:
                return False, "Canceled"
            if timed_out():
                return False, "Timeout"

            fb.state = state
            fb.progress = progress
            fb.left_target_pose = left_pose
            fb.right_target_pose = right_pose
            goal_handle.publish_feedback(fb)

            # ✅ 핵심: 두 팔 goal을 "먼저" 동시에 전송
            left_send_fut  = self._send_move_action_goal("arm_left",  ee_left,  left_pose,  plan_only=False, _return_futures=True)
            right_send_fut = self._send_move_action_goal("arm_right", ee_right, right_pose, plan_only=False, _return_futures=True)

            (left_ok, left_msg, left_result_fut), (right_ok, right_msg, right_result_fut) = await left_send_fut, await right_send_fut

            if not left_ok or not right_ok:
                return False, f"{state} send failed: left=({left_msg}), right=({right_msg})"

            # ✅ 둘 다 goal 수락된 뒤, result future도 "둘 다 이미 in-flight" 상태
            left_res  = await left_result_fut
            right_res = await right_result_fut

            left_code  = int(left_res.result.error_code.val)
            right_code = int(right_res.result.error_code.val)

            if left_code != 1 or right_code != 1:
                return False, f"{state} failed: left=(error_code={left_code}), right=(error_code={right_code})"

            return True, "ok"

        ok, msg = await step_send_both("MOVE_PREGRASP", 0.35, left_pre, right_pre)
        if not ok:
            goal_handle.abort()
            res.success = False
            res.message = msg
            return res

        ok, msg = await step_send_both("MOVE_GRASP", 0.65, left_grasp, right_grasp)
        if not ok:
            goal_handle.abort()
            res.success = False
            res.message = msg
            return res

        fb.state = "PLAN_LIFT"
        fb.progress = 0.80
        fb.left_target_pose = left_lift
        fb.right_target_pose = right_lift
        goal_handle.publish_feedback(fb)

        okL, msgL, trajL = await self._send_move_action_goal("arm_left", ee_left, left_lift, plan_only=True)
        okR, msgR, trajR = await self._send_move_action_goal("arm_right", ee_right, right_lift, plan_only=True)

        if not okL or not okR:
            goal_handle.abort()
            res.success = False
            res.message = f"PLAN_LIFT failed: left=({msgL}), right=({msgR})"
            return res

        fb.state = "EXEC_LIFT_SIMUL"
        fb.progress = 0.85
        goal_handle.publish_feedback(fb)

        left_ok, left_msg = await self._exec_joint_traj("left", trajL)
        right_ok, right_msg = await self._exec_joint_traj("right", trajR)

        if not left_ok or not right_ok:
            goal_handle.abort()
            res.success = False
            res.message = f"EXEC_LIFT failed: left=({left_msg}), right=({right_msg})"
            return res

        fb.state = "DONE"
        fb.progress = 1.0
        goal_handle.publish_feedback(fb)

        goal_handle.succeed()
        res.success = True
        res.message = "Dual-arm side pinch sequence completed (center+width)"
        res.left_final_pose = left_lift
        res.right_final_pose = right_lift
        return res

    # -----------------------
    # MoveGroup goal helper
    # -----------------------
    async def _send_move_action_goal(
        self,
        group: str,
        ee_link: str,
        target_pose: PoseStamped,
        *,
        plan_only: bool = False,
        _return_futures: bool = False,
    ) -> Tuple[bool, str, Optional[object]]:
        allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        num_attempts = int(self.get_parameter("num_planning_attempts").value)
        v_scale = float(self.get_parameter("max_velocity_scaling").value)
        a_scale = float(self.get_parameter("max_acceleration_scaling").value)
        pos_tol = float(self.get_parameter("pos_tol").value)
        ori_tol = float(self.get_parameter("ori_tol").value)
        use_ori = bool(self.get_parameter("use_orientation_constraint").value)

        c = Constraints()
        c.name = f"{group}_target"

        pc = PositionConstraint()
        pc.header = target_pose.header
        pc.link_name = ee_link

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [pos_tol * 2.0, pos_tol * 2.0, pos_tol * 2.0]
        pc.constraint_region.primitives = [prim]
        pc.constraint_region.primitive_poses = [target_pose.pose]
        pc.weight = 1.0
        c.position_constraints = [pc]

        if use_ori:
            oc = OrientationConstraint()
            oc.header = target_pose.header
            oc.link_name = ee_link
            oc.orientation = target_pose.pose.orientation
            oc.absolute_x_axis_tolerance = ori_tol
            oc.absolute_y_axis_tolerance = ori_tol
            oc.absolute_z_axis_tolerance = ori_tol
            oc.weight = 1.0
            c.orientation_constraints = [oc]
        else:
            c.orientation_constraints = []

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group
        goal_msg.request.num_planning_attempts = num_attempts
        goal_msg.request.allowed_planning_time = allowed_planning_time
        goal_msg.request.max_velocity_scaling_factor = v_scale
        goal_msg.request.max_acceleration_scaling_factor = a_scale
        goal_msg.request.goal_constraints = [c]
        goal_msg.planning_options.plan_only = bool(plan_only)
        goal_msg.planning_options.replan = False

        client = self._mg_left if group == "arm_left" else self._mg_right

        send_goal_future = client.send_goal_async(goal_msg)
        gh = await send_goal_future
        if not gh.accepted:
            return False, "goal rejected", None

        result_future = gh.get_result_async()

        # ✅ 여기서 끝내고 result_future를 넘겨버림 (동시성 확보용)
        if _return_futures:
            return True, "accepted", result_future

        # 기존 동작(끝까지 기다림)
        result = await result_future
        code = int(result.result.error_code.val)
        if code != 1:
            return False, f"error_code={code}", None

        traj = result.result.planned_trajectory if plan_only else None
        return True, "success", traj


def main():
    rclpy.init()
    node = DualArmGraspBoxActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
