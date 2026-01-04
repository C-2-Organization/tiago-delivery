from typing import List
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class GripperClient:
    def __init__(self, node, controller_action_name: str, joint_names: List[str]):
        self.node = node
        self.joint_names = joint_names
        self.client = ActionClient(node, FollowJointTrajectory, controller_action_name)

    def wait(self, timeout_sec=5.0) -> bool:
        return self.client.wait_for_server(timeout_sec=timeout_sec)

    async def command(self, positions: List[float], time_sec: float = 1.0) -> bool:
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start = Duration(sec=int(time_sec), nanosec=int((time_sec % 1.0)*1e9))
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        send_fut = self.client.send_goal_async(goal)
        await send_fut
        gh = send_fut.result()
        if not gh.accepted:
            self.node.get_logger().error("[Gripper] goal rejected")
            return False

        res_fut = gh.get_result_async()
        await res_fut
        res = res_fut.result().result
        return res.error_code == 0
