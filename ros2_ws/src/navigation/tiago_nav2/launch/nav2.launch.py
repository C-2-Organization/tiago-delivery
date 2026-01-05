import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -----------------------------
    # Launch configurations
    # -----------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    goals_yaml = LaunchConfiguration('goals_yaml')
    destination_topic = LaunchConfiguration('destination_topic')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tiago_nav2_share = get_package_share_directory('tiago_nav2')

    return LaunchDescription([

        # =============================
        # Launch Arguments
        # =============================
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                tiago_nav2_share,
                'config',
                'nav2_params.yaml'
            )
        ),

        DeclareLaunchArgument(
            'map',
            default_value='/home/ros/tiago-delivery/ros2_ws/src/navigation/tiago_nav2/maps/lobby_map.yaml'
        ),

        DeclareLaunchArgument(
            'goals_yaml',
            default_value=os.path.join(
                tiago_nav2_share,
                'config',
                'goals.yaml'
            )
        ),

        DeclareLaunchArgument(
            'destination_topic',
            default_value='/delivery/destination_id'
        ),

        # =============================
        # 1) Nav2 Navigation
        # (AMCL + Planner + Controller)
        # =============================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    nav2_bringup_dir,
                    'launch',
                    'navigation_launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'map': map_yaml,
            }.items()
        ),

        # # =============================
        # # 2) Goal Dispatcher
        # # =============================
        # Node(
        #     package='tiago_nav2',
        #     executable='goal_dispatcher',
        #     name='goal_dispatcher_node',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #         'destination_topic': destination_topic,
        #         'goals_yaml': goals_yaml,
        #         'action_name': '/navigate_to_pose',
        #     }]
        # ),
    ])
