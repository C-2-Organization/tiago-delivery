#!/usr/bin/env python3
# manipulation/lift_box.py
#
# LiftBoxServer: 박스를 테이블 바깥으로 밀고(후킹) 들어올려 운반 자세로 만드는
# "전체 시퀀스"를 단계별로 쌓아가는 Action Server.
#
# 현재 구현: Stage 1 (INIT_POSTURE) 까지만.
# 다음 단계들은 TODO로 스켈레톤만 남겨둠:
#  - APPROACH_TABLE
#  - RIGHT_HOOK_J6
#  - BACK_OFF
#  - BASE_ROTATE_CCW (cmd_vel)
#  - LEFT_ARM_LIFT_J2
#  - HOLD / DONE
#
# 컨셉:
# - MoveIt 대신 고정 자세/고정 환경(박스 크기, 테이블 높이)을 전제한 FJT 기반 제어
# - 동기 방식으로 구현 (spin_until_future_complete 사용)

import math
from dataclasses import dataclass
from typing import Dict, List
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from interfaces.action import LiftBox


def deg2rad(x: float) -> float:
    return float(x) * math.pi / 180.0


LEFT_ARM_CONTROLLER = "/arm_left_controller/follow_joint_trajectory"
RIGHT_ARM_CONTROLLER = "/arm_right_controller/follow_joint_trajectory"

LEFT_JOINTS: List[str] = [
    "arm_left_1_joint",
    "arm_left_2_joint",
    "arm_left_3_joint",
    "arm_left_4_joint",
    "arm_left_5_joint",
    "arm_left_6_joint",
    "arm_left_7_joint",
]

RIGHT_JOINTS: List[str] = [
    "arm_right_1_joint",
    "arm_right_2_joint",
    "arm_right_3_joint",
    "arm_right_4_joint",
    "arm_right_5_joint",
    "arm_right_6_joint",
    "arm_right_7_joint",
]


@dataclass
class ArmExecResult:
    ok: bool
    msg: str
    error_code: int = 0


class LiftBoxServer(Node):
    """
    LiftBox 전체 시퀀스 Action Server.

    Stage 1: INIT_POSTURE (좌/우 팔 고정 초기자세)
    이후 Stage를 여기에 계속 추가할 예정.
    """

    def __init__(self):
        super().__init__("lift_box_server")

        # Callback group for concurrent execution
        self._cb_group = ReentrantCallbackGroup()

        # Action clients (FollowJointTrajectory)
        self._fj_left = ActionClient(
            self,
            FollowJointTrajectory,
            LEFT_ARM_CONTROLLER,
            callback_group=self._cb_group,
        )
        self._fj_right = ActionClient(
            self,
            FollowJointTrajectory,
            RIGHT_ARM_CONTROLLER,
            callback_group=self._cb_group,
        )

        # Lock for thread safety
        self._exec_lock = Lock()

        # --------------------
        # Fixed postures (deg)
        # --------------------
        self._init_left_deg: Dict[str, float] = {
            "arm_left_1_joint": 90.0,
            "arm_left_2_joint": 15.0,
            "arm_left_3_joint": 90.0,
            "arm_left_4_joint": 30.0,
            "arm_left_5_joint": 0.0,
            "arm_left_6_joint": 0.0,
            "arm_left_7_joint": 0.0,
        }
        self._init_right_deg: Dict[str, float] = {
            "arm_right_1_joint": 90.0,
            "arm_right_2_joint": -15.0,
            "arm_right_3_joint": 0.0,
            "arm_right_4_joint": 0.0,
            "arm_right_5_joint": 30.0,
            "arm_right_6_joint": 0.0,
            "arm_right_7_joint": 0.0,
        }

        # --------------------
        # Parameters
        # --------------------
        self.declare_parameter("wait_controller_server_sec", 5.0)

        # 전체 시퀀스 default (박스 접근/밀기/리프트까지 포함하면 길어질 것)
        self.declare_parameter("default_total_timeout_sec", 200.0)

        # 각 stage default timeout
        self.declare_parameter("default_stage_timeout_sec", 30.0)

        # 초기 자세로 가는 traj duration
        self.declare_parameter("default_init_move_time_sec", 3.0)

        # --------------------
        # Action server
        # --------------------
        self._as = ActionServer(
            self,
            LiftBox,
            "lift_box",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info("LiftBoxServer started: /lift_box")

    # -----------------------
    # Action callbacks
    # -----------------------
    def goal_callback(self, goal: LiftBox.Goal) -> int:
        # goal에서 <=0이면 default 사용
        total_timeout = (
            float(goal.timeout_sec)
            if goal.timeout_sec > 0.0
            else float(self.get_parameter("default_total_timeout_sec").value)
        )
        stage_timeout = (
            float(goal.stage_timeout_sec)
            if goal.stage_timeout_sec > 0.0
            else float(self.get_parameter("default_stage_timeout_sec").value)
        )
        init_move_time = (
            float(goal.init_move_time_sec)
            if goal.init_move_time_sec > 0.0
            else float(self.get_parameter("default_init_move_time_sec").value)
        )

        # 전체는 길게 잡고, stage는 너무 길면 디버깅 어려움
        if total_timeout < 5.0 or total_timeout > 600.0:
            return GoalResponse.REJECT
        if stage_timeout < 2.0 or stage_timeout > 120.0:
            return GoalResponse.REJECT
        if init_move_time < 0.5 or init_move_time > 20.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> int:
        self.get_logger().warn("Cancel requested (best-effort stop after current stage).")
        return CancelResponse.ACCEPT

    # -----------------------
    # Utilities
    # -----------------------
    def _build_fjt_goal(
        self,
        joint_names: List[str],
        target_deg: Dict[str, float],
        move_time_sec: float,
    ) -> FollowJointTrajectory.Goal:
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = [deg2rad(target_deg[j]) for j in joint_names]
        pt.velocities = [0.0] * len(joint_names)
        pt.accelerations = [0.0] * len(joint_names)
        pt.time_from_start = Duration(seconds=float(move_time_sec)).to_msg()

        g.trajectory.points = [pt]
        return g

    def _exec_fjt_sync(
        self,
        arm: str,
        fj_goal: FollowJointTrajectory.Goal,
        stage_timeout_sec: float,
    ) -> ArmExecResult:
        """
        동기 방식으로 FollowJointTrajectory 실행.
        spin_until_future_complete 사용.
        """
        client = self._fj_left if arm == "left" else self._fj_right

        wait_sec = float(self.get_parameter("wait_controller_server_sec").value)
        if not client.wait_for_server(timeout_sec=wait_sec):
            return ArmExecResult(
                False, f"{arm} follow_joint_trajectory server not available", error_code=-1
            )

        # Send goal
        send_goal_future = client.send_goal_async(fj_goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=stage_timeout_sec)

        if not send_goal_future.done():
            return ArmExecResult(False, f"{arm} send_goal timeout", error_code=-2)

        goal_handle = send_goal_future.result()
        if goal_handle is None:
            return ArmExecResult(False, f"{arm} goal_handle is None", error_code=-3)

        if not goal_handle.accepted:
            return ArmExecResult(False, f"{arm} goal rejected", error_code=-4)

        self.get_logger().info(f"{arm} arm goal accepted, waiting for result...")

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=stage_timeout_sec)

        if not result_future.done():
            # Timeout - try to cancel
            try:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            except Exception as e:
                self.get_logger().warn(f"Cancel failed: {e}")
            return ArmExecResult(False, f"{arm} result timeout", error_code=-5)

        result = result_future.result()
        if result is None:
            return ArmExecResult(False, f"{arm} no result", error_code=-6)

        code = int(result.result.error_code)
        if code == 0:
            return ArmExecResult(True, "success", error_code=0)

        err_str = getattr(result.result, "error_string", "")
        return ArmExecResult(False, f"{arm} error_code={code} err='{err_str}'", error_code=code)

    def _publish_fb(self, goal_handle, fb: LiftBox.Feedback, stage: str, progress: float):
        fb.stage = stage
        fb.progress = float(progress)
        goal_handle.publish_feedback(fb)

    # -----------------------
    # Main sequence (동기 방식)
    # -----------------------
    def execute_callback(self, goal_handle):
        """
        동기 방식 execute callback.
        각 팔을 순차적으로 실행하거나, 스레드로 병렬 실행 가능.
        여기서는 순차 실행으로 구현 (안정성 우선).
        """
        goal = goal_handle.request
        fb = LiftBox.Feedback()
        res = LiftBox.Result()

        total_timeout_sec = (
            float(goal.timeout_sec)
            if goal.timeout_sec > 0.0
            else float(self.get_parameter("default_total_timeout_sec").value)
        )
        stage_timeout_sec = (
            float(goal.stage_timeout_sec)
            if goal.stage_timeout_sec > 0.0
            else float(self.get_parameter("default_stage_timeout_sec").value)
        )
        init_move_time_sec = (
            float(goal.init_move_time_sec)
            if goal.init_move_time_sec > 0.0
            else float(self.get_parameter("default_init_move_time_sec").value)
        )

        total_start = self.get_clock().now()
        total_timeout = Duration(seconds=float(total_timeout_sec))

        def total_timed_out() -> bool:
            return (self.get_clock().now() - total_start) > total_timeout

        # --------------------
        # Stage 0: WAIT_CONTROLLERS
        # --------------------
        self._publish_fb(goal_handle, fb, "WAIT_CONTROLLERS", 0.05)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout"
            return res

        # --------------------
        # Stage 1: INIT_POSTURE
        # --------------------
        self._publish_fb(goal_handle, fb, "INIT_POSTURE", 0.15)
        self.get_logger().info("Stage: INIT_POSTURE - Moving both arms to initial posture")

        left_goal = self._build_fjt_goal(LEFT_JOINTS, self._init_left_deg, init_move_time_sec)
        right_goal = self._build_fjt_goal(RIGHT_JOINTS, self._init_right_deg, init_move_time_sec)

        # 순차 실행 (왼팔 먼저, 오른팔 다음)
        # 병렬 실행이 필요하면 threading.Thread 사용 가능
        self.get_logger().info("Moving left arm...")
        left_res = self._exec_fjt_sync("left", left_goal, stage_timeout_sec)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled after left arm"
            return res

        if not left_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"INIT_POSTURE failed: left=({left_res.msg})"
            return res

        self.get_logger().info("Left arm done. Moving right arm...")
        right_res = self._exec_fjt_sync("right", right_goal, stage_timeout_sec)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            res.success = False
            res.message = "Canceled after right arm"
            return res

        if total_timed_out():
            goal_handle.abort()
            res.success = False
            res.message = "Total timeout"
            return res

        if not right_res.ok:
            goal_handle.abort()
            res.success = False
            res.message = f"INIT_POSTURE failed: right=({right_res.msg})"
            return res

        self.get_logger().info("Both arms moved to initial posture successfully")

        # 디버깅 플래그: 초기 자세만 하고 끝
        if bool(getattr(goal, "init_only", False)):
            self._publish_fb(goal_handle, fb, "DONE", 1.0)
            goal_handle.succeed()
            res.success = True
            res.message = "Init posture set (init_only=true)"
            return res

        # --------------------
        # Next stages (TODO)
        # --------------------
        # self._publish_fb(goal_handle, fb, "APPROACH_TABLE", 0.30)
        # ... cmd_vel로 접근 or 네비게이션 ...
        #
        # self._publish_fb(goal_handle, fb, "RIGHT_HOOK_J6", 0.45)
        # ... 오른팔 joint6 -80deg ...
        #
        # self._publish_fb(goal_handle, fb, "BACK_OFF", 0.55)
        # ... cmd_vel 후진 ...
        #
        # self._publish_fb(goal_handle, fb, "BASE_ROTATE_CCW", 0.70)
        # ... cmd_vel 회전 ...
        #
        # self._publish_fb(goal_handle, fb, "LEFT_ARM_LIFT_J2", 0.85)
        # ... 왼팔 joint2 올리기 ...
        #
        # self._publish_fb(goal_handle, fb, "HOLD", 0.95)

        # 지금은 여기까지 구현 안했으니 success로 종료(추후 단계 구현 시 여기 제거)
        self._publish_fb(goal_handle, fb, "DONE", 1.0)
        goal_handle.succeed()
        res.success = True
        res.message = "Init posture set (next stages TODO)"
        return res


def main():
    rclpy.init()
    node = LiftBoxServer()

    # MultiThreadedExecutor 사용 (ReentrantCallbackGroup과 함께)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
