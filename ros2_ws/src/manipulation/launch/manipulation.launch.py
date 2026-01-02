from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="manipulation",
            executable="approach_box",
            name="approach_box_action_server",
            output="screen",
            parameters=[
                {"kp_ang": 1.8},
                {"kp_lin": 0.6},
                {"max_linear": 0.35},
                {"max_angular": 0.9},
                {"x_deadband": 0.03},
                {"z_deadband": 0.05},
                {"min_depth_m": 0.15},
                {"max_depth_m": 6.0},
                {"measurement_timeout": 0.5},
                {"bearing_deadband": 0.05},
                {"center_for_forward_bearing": 0.12},
                {"max_align_angular": 0.45},
                {"v_align": 0.06},
            ]
        ),
    ])
