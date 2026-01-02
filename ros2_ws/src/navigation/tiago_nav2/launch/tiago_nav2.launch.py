import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    goals_yaml = LaunchConfiguration('goals_yaml')
    destination_topic = LaunchConfiguration('destination_topic')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        # -----------------------------
        # Launch arguments
        # -----------------------------
        DeclareLaunchArgument('use_sim_time', default_value='True'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('tiago_nav2'),
                'config',
                'nav2_params.yaml'
            )
        ),

        DeclareLaunchArgument(
            'map',
            default_value='/home/rokey/tiago_maps/tiago_map.yaml'
        ),

        # ✅ 방 좌표 저장 yaml (101~104)
        DeclareLaunchArgument(
            'goals_yaml',
            default_value=os.path.join(
                get_package_share_directory('tiago_nav2'),
                'config',
                'goals.yaml'
            )
        ),

        # ✅ QR 결과(목적지) 들어오는 토픽
        # qr_reader_node.py가 publish하는 토픽에 맞추면 됨
        DeclareLaunchArgument(
            'destination_topic',
            default_value='/delivery/destination_id'
        ),

        # -----------------------------
        # Nav2 bringup
        # -----------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,

                # ✅ 중요: PythonExpression eval() 이슈 방지 (False/True로)
                'slam': 'False',
                'autostart': 'True',

                # bringup에서 PythonExpression로 평가하는 옵션이 있을 때 대비
                'use_composition': 'False',
                'use_respawn': 'False',

                'map': map_yaml,
            }.items()
        ),

        # -----------------------------
        # Goal Dispatcher (QR -> room pose -> Nav2 action)
        # -----------------------------
        Node(
            package='tiago_nav2',
            executable='goal_dispatcher',
            name='goal_dispatcher_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'destination_topic': destination_topic,
                'goals_yaml': goals_yaml,
                'action_name': '/navigate_to_pose',
                'ignore_empty': True,
                'dedup_same_goal': True,
            }]
        ),
    ])
