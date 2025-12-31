# Usage:
#   1. Run controller: ~/tiago-delivery/external/tiago_isaac$ python3 tiago_example_controller.py
#   2. Run this launch: ~/tiago-delivery/ros2_ws$ ros2 launch tiago_cartographer tiago_cartographer.launch.py use_sim_time:=true
#   3. Save the map after scanning: ros2 run nav2_map_server map_saver_cli -f ~/map

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('tiago_cartographer') 
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # RViz 설정 파일 경로 지정 (패키지 내 rviz 폴더에 있다고 가정)
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'tiago_cartographer.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # 1. Lidar Fixer 노드
        Node(
            package='tiago_cartographer',
            executable='lidar_fixer.py',
            name='lidar_fixer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 2. Cartographer 노드
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', os.path.join(pkg_share, 'config'),
                    '-configuration_basename', 'tiago_rear_laser.lua'],
            remappings=[
                ('scan_1', '/scan_front_fixed'),
                ('scan_2', '/scan_rear_fixed'),
                ('odom', '/odom')
            ]),

        # 3. Occupancy Grid 노드
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']),

        # 4. RViz2 노드
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config_path],
            output='screen'),
    ])