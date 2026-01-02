#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, PointStamped

# interfaces 패키지 액션
from interfaces.action import ApproachBox


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class Target:
    x: float
    z: float

# Command example:
# ros2 action send_goal --feedback /approach_box interfaces/action/ApproachBox \
# "{stop_distance: 0.5, timeout_sec: 20.0, min_confidence: 0.0, align_first: true}"
class ApproachBoxActionServer(Node):
    """
    Action: /approach_box
    Sub:   /perception/box_point_cam (PointStamped)
    Pub:   /cmd_vel (Twist)
    """

    def __init__(self):
        super().__init__("approach_box_action_server")

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.point_sub = self.create_subscription(
            PointStamped, "/perception/box_point_cam", self.on_point, 10
        )

        # latest target
        self.last_point: Optional[PointStamped] = None
        self.last_point_stamp = None  # node time

        # Action server
        self._action_server = ActionServer(
            self,
            ApproachBox,
            "approach_box",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # control params (default; can tune)
        self.declare_parameter("kp_ang", 1.8)        # omega = -kp_ang * x
        self.declare_parameter("kp_lin", 0.6)        # v = kp_lin * (z - stop)
        self.declare_parameter("max_linear", 0.35)
        self.declare_parameter("max_angular", 0.9)

        self.declare_parameter("x_deadband", 0.03)
        self.declare_parameter("z_deadband", 0.05)

        self.declare_parameter("min_depth_m", 0.15)
        self.declare_parameter("max_depth_m", 6.0)

        # measurement staleness
        self.declare_parameter("measurement_timeout", 0.5)  # sec
        
        self.declare_parameter("bearing_deadband", 0.05)   # rad (~2.9 deg)
        self.declare_parameter("center_for_forward_bearing", 0.12)  # rad (~6.9 deg)

        self.declare_parameter("max_align_angular", 0.45)  # rad/s (align 중엔 더 느리게)
        self.declare_parameter("v_align", 0.06)            # m/s (align 중에도 아주 천천히 전진)

        self.get_logger().info("ApproachBoxActionServer started.")

    def destroy_node(self):
        # ensure robot stops
        self.publish_stop()
        super().destroy_node()

    def publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    def on_point(self, msg: PointStamped):
        self.last_point = msg
        self.last_point_stamp = self.get_clock().now()

    # --- action callbacks ---

    def goal_callback(self, goal_request: ApproachBox.Goal):
        # You can reject unrealistic goals here
        if goal_request.stop_distance <= 0.0 or goal_request.stop_distance > 3.0:
            self.get_logger().warn("Reject goal: stop_distance out of range.")
            return GoalResponse.REJECT
        if goal_request.timeout_sec <= 0.5:
            self.get_logger().warn("Reject goal: timeout_sec too small.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn("Cancel requested.")
        self.publish_stop()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        goal = goal_handle.request

        stop_distance = float(goal.stop_distance)
        timeout_sec = float(goal.timeout_sec)
        align_first = bool(goal.align_first)

        kp_ang = float(self.get_parameter("kp_ang").value)
        kp_lin = float(self.get_parameter("kp_lin").value)
        max_v = float(self.get_parameter("max_linear").value)
        max_w = float(self.get_parameter("max_angular").value)

        x_db = float(self.get_parameter("x_deadband").value)
        z_db = float(self.get_parameter("z_deadband").value)

        min_d = float(self.get_parameter("min_depth_m").value)
        max_d = float(self.get_parameter("max_depth_m").value)

        meas_timeout = float(self.get_parameter("measurement_timeout").value)

        start_time = self.get_clock().now()
        rate_hz = 20.0
        period = 1.0 / rate_hz

        feedback = ApproachBox.Feedback()
        last_used_point = PointStamped()

        self.get_logger().info(
            f"[ApproachBox] start stop_distance={stop_distance:.2f} timeout={timeout_sec:.1f}s align_first={align_first}"
        )

        while rclpy.ok():
            # cancel?
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.publish_stop()
                result = ApproachBox.Result()
                result.success = False
                result.message = "Canceled"
                result.final_point_cam = last_used_point
                return result

            # timeout?
            if (self.get_clock().now() - start_time) > Duration(seconds=timeout_sec):
                self.publish_stop()
                goal_handle.abort()
                result = ApproachBox.Result()
                result.success = False
                result.message = f"Timeout after {timeout_sec:.1f}s"
                result.final_point_cam = last_used_point
                return result

            # measurement available?
            if self.last_point is None or self.last_point_stamp is None:
                self.publish_stop()
                feedback.state = "NO_TARGET"
                goal_handle.publish_feedback(feedback)
                rclpy.spin_once(self, timeout_sec=period)
                continue

            age = (self.get_clock().now() - self.last_point_stamp).nanoseconds * 1e-9
            if age > meas_timeout:
                self.publish_stop()
                feedback.state = f"NO_TARGET(stale {age:.2f}s)"
                goal_handle.publish_feedback(feedback)
                rclpy.spin_once(self, timeout_sec=period)
                continue

            # use measurement
            x = float(self.last_point.point.x)
            z = float(self.last_point.point.z)
            last_used_point = self.last_point

            # validity
            if not (math.isfinite(x) and math.isfinite(z)) or (z < min_d or z > max_d):
                self.publish_stop()
                feedback.state = "NO_TARGET(invalid_depth)"
                goal_handle.publish_feedback(feedback)
                rclpy.spin_once(self, timeout_sec=period)
                continue

            bearing = math.atan2(x, z)
            ez = z - stop_distance
            
            bearing_db = float(self.get_parameter("bearing_deadband").value)
            center_for_forward_bearing = float(self.get_parameter("center_for_forward_bearing").value)
            max_align_w = float(self.get_parameter("max_align_angular").value)
            v_align = float(self.get_parameter("v_align").value)
            
            # arrived?
            if abs(bearing) < bearing_db and abs(ez) < z_db:
                self.publish_stop()
                goal_handle.succeed()
                result = ApproachBox.Result()
                result.success = True
                result.message = f"Arrived: x={x:.3f} z={z:.3f} target_z={stop_distance:.2f}"
                result.final_point_cam = last_used_point
                return result

            # control
            omega = -kp_ang * bearing
            omega = clamp(omega, -max_w, max_w)

            v = kp_lin * ez
            v = clamp(v, -max_v, max_v)

            if ez < 0.0:
                v = 0.0

            if abs(bearing) < bearing_db:
                omega = 0.0

            if align_first and abs(bearing) > center_for_forward_bearing:
                v = max(0.0, min(v, v_align))
                v = max(v, 0.03)
                omega = clamp(omega, -max_align_w, max_align_w)
                feedback.state = "ALIGNING"
            else:
                feedback.state = "APPROACHING"

            cmd = Twist()
            cmd.linear.x = float(v)
            cmd.angular.z = float(omega)
            self.cmd_vel_pub.publish(cmd)

            feedback.x_cam = float(x)
            feedback.z_cam = float(z)
            feedback.distance_error = float(ez)
            feedback.angle_error = float(bearing)
            goal_handle.publish_feedback(feedback)

            # tick (callbacks 처리 + sleep 효과)
            rclpy.spin_once(self, timeout_sec=period)

        # shutdown
        self.publish_stop()
        goal_handle.abort()
        result = ApproachBox.Result()
        result.success = False
        result.message = "Node shutting down"
        result.final_point_cam = last_used_point
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ApproachBoxActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
