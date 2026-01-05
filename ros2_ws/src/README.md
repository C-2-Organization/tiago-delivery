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
ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=true \
  map:=/home/rokey/tiago_maps/tiago_map_current.yaml

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true


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
## 안될 때 명령어

pkill -f slam_toolbox
pkill -f nav2
pkill -f rviz2
pkill -f map_server
pkill -f amcl

ros2 daemon stop
rm -rf ~/.ros/daemon
ros2 daemon start

sudo rm -f /dev/shm/fastrtps_*
sudo rm -f /dev/shm/fastdds_*

cd ~/tiago-delivery/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
---------------------------------------------------------------

# Isaac Sim (TIAGo) → Nav2 “Goal 찍으면 주행” 최소 실행 순서  
(LOC → RViz 초기위치 → NAV)

> ✅ 목표: Isaac Sim에서 TIAGo를 **RViz에서 Nav2 Goal 찍어서 주행**시키기  
> ✅ 핵심: **/clock 살아있음(Play)** + **LOC(AMCL) 먼저** + **RViz 2D Pose Estimate** + **NAV 실행**

---

## 0) 싹 정리 (권장)

```bash
pkill -f nav2
pkill -f slam_toolbox
pkill -f rviz2
ros2 daemon stop
rm -rf ~/.ros/daemon
ros2 daemon start
```

---

## 1) Isaac Sim 준비 (가장 중요)

- Isaac Sim은 **Play 상태**여야 함  
  (Pause면 `/clock` 멈춰서 Nav2 / TF 전체 꼬임)
- LiDAR / odom / tf 브릿지가 정상적으로 설정되어 있어야 함

### 필수 토픽 확인

```bash
ros2 topic list | grep -E "^/clock$|^/tf$|^/tf_static$|^/odom$|/scan_front_raw"
```

### `/clock` 1차 확인

```bash
ros2 topic echo --once /clock
```

---

## 2) ROS 환경 세팅 (모든 터미널 공통)

> 아래 명령은 **터미널 A / B / C 전부 동일**

```bash
cd ~/tiago-delivery/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 3) LOC 실행 (map_server + amcl)  
⚠️ **NAV보다 반드시 먼저 실행**

### 터미널 A

```bash
ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=true \
  map:=/home/rokey/tiago_maps/lobby_map.yaml \
  params_file:=/home/rokey/tiago-delivery/ros2_ws/src/navigation/tiago_nav2/config/nav2_params.yaml
```

### LOC 정상 체크

```bash
ros2 node list | grep -E "map_server|amcl"
ros2 topic echo --once /map
ros2 topic echo --once /amcl_pose
```

---

## 4) RViz 실행 + 초기 위치 잡기 (2D Pose Estimate)  
⚠️ **AMCL 초기화 필수 단계**

### 터미널 B

```bash
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### RViz 설정

- **Fixed Frame = `map`**
- **2D Pose Estimate** 버튼으로 로봇 위치 + 방향 지정

### 초기화 확인

```bash
ros2 topic echo --once /amcl_pose
```

---

## 5) NAV 실행 (Planner / Controller / BT / Behavior)

### 터미널 C

```bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/home/rokey/tiago-delivery/ros2_ws/src/navigation/tiago_nav2/config/nav2_params.yaml
```

### NAV 노드 확인

```bash
ros2 node list | grep -E \
"planner_server|controller_server|bt_navigator|behavior_server|velocity_smoother"
```

---

## 6) RViz에서 Nav2 Goal 보내기

- RViz 상단에서 **Nav2 Goal** 클릭
- 맵 위 목표 위치 지정 → 로봇 주행 시작

### 주행 중 토픽 확인

```bash
ros2 topic echo --once /cmd_vel_nav
ros2 topic echo --once /odom
```

---

## 7) (선택) 맵만 바꿔서 사용할 때

- **LOC 실행 시 `map:=...`만 교체**
- NAV는 그대로 재사용 가능

```bash
ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=true \
  map:=/home/rokey/tiago_maps/second_floor_map.yaml \
  params_file:=/home/rokey/tiago-delivery/ros2_ws/src/navigation/tiago_nav2/config/nav2_params.yaml
```

---

## 🔥 문제 발생 시 필수 체크 2가지

### 1) Isaac Sim이 Play 상태인가? (`/clock` 살아있나)

```bash
ros2 topic echo --once /clock
```

### 2) Nav2가 실제 속도 명령을 내고 있는가?

```bash
ros2 topic echo --once /cmd_vel_nav
```
