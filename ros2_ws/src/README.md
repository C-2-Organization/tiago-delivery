# TIAGO Navigation / Goal 주행 실행 명령어 기록

## 1. ROS2 워크스페이스 기본 환경 설정
```bash
cd ~/tiago-delivery/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 2. Isaac Sim TIAGO 컨트롤러 실행
```bash
cd ~/tiago-delivery/external/tiago_isaac
python3 tiago_example_controller.py
```

---

## 3. Nav2 실행 (맵 기반 자율주행)
```bash
ros2 launch tiago_nav2 tiago_nav2.launch.py \
  use_sim_time:=true \
  map:=/home/rokey/tiago_maps/tiago_map_v4.yaml
```

---

## 4. 목적지 트리거 (QR 대신 수동 테스트)
```bash
ros2 topic pub -1 /delivery/destination_id std_msgs/msg/String "{data: '101'}"
```

---

## 5. AMCL 초기 자세 수동 입력 (RViz 없이)
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {z: 0.0, w: 1.0}
    }
  }
}"
```

---

## 6. Cartographer SLAM 실행 (맵 생성)
```bash
ros2 launch tiago_cartographer tiago_cartographer.launch.py use_sim_time:=true
```

---

## 7. 맵 저장
```bash
ros2 run nav2_map_server map_saver_cli -f ~/tiago_maps/tiago_map_current
```

---

## 8. Nav2 재실행 (새로 생성한 맵 사용)
```bash
ros2 launch tiago_nav2 tiago_nav2.launch.py \
  use_sim_time:=true \
  map:=/home/rokey/tiago_maps/tiago_map_current.yaml
```

---

## 9. goal_dispatcher 단독 실행
```bash
ros2 run tiago_nav2 goal_dispatcher
```

---

## 10. odom 기준 좌표 주행 노드 실행 (맵 없이)
```bash
ros2 run tiago_nav2 odom_goal_driver \
  --ros-args \
  -p destination_topic:=/delivery/destination_id \
  -p goals_yaml:=/home/rokey/tiago-delivery/ros2_ws/src/navigation/tiago_nav2/config/goals.yaml \
  -p odom_frame:=odom \
  -p base_frame:=base_link
```

---

## 11. 목적지 연속 주행 테스트
```bash
ros2 topic pub -1 /delivery/destination_id std_msgs/msg/String "{data: '101'}"
ros2 topic pub -1 /delivery/destination_id std_msgs/msg/String "{data: '102'}"
ros2 topic pub -1 /delivery/destination_id std_msgs/msg/String "{data: '103'}"
ros2 topic pub -1 /delivery/destination_id std_msgs/msg/String "{data: '104'}"
```
