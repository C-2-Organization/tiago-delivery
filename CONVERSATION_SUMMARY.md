# 대화 내용 요약 - Tiago Delivery 프로젝트

## 📋 작업 이력

### 1. tiago_example_controller.py 찾기
- 경로: `~/tiago-delivery/external/tiago_isaac/tiago_example_controller.py`
- AIS-Bonn의 tiago_isaac 리포지토리에서 클론
- `dependencies.repos` 파일에 정의되어 있음

### 2. tiago_example_controller.py 분석
**파일 정보**: 396줄의 Python 파일

**주요 구성요소**:
- **ROS 2 노드**: `JointControlNode`
  - Publisher: `/joint_command` (관절 제어), `/cmd_vel` (속도 제어)
  - Subscriber: `/joint_states` (관절 상태)
  
- **tkinter GUI**: `JointControlGUI`
  - 21개 관절 제어 (왼팔 7, 오른팔 7, 그리퍼 4, 머리 2, 몸통 1)
  - 각 관절마다 슬라이더 3개:
    - 왼쪽: 현재 상태 (읽기 전용)
    - 가운데: 관절 이름
    - 오른쪽: 목표 각도 (사용자 조작)
  
- **키보드 제어**:
  - W/S: 전진/후진
  - A/D: 좌/우 이동
  - ←/→: 회전

- **포즈 저장/로드**: YAML 형식으로 `./assets/example_saved_poses/`에 저장

### 3. 박스 감지 및 접근 시스템 확인

#### Perception (인지)
**파일 위치**: `ros2_ws/src/perception/receiver_detection/`

1. **yolo_box_detector_node.py**
   - YOLO 모델로 2D 이미지에서 박스 감지
   - 입력: `/gemini2/color/image_raw`
   - 출력: `/perception/box_detection_2d`
   - 모델 경로: `/home/ros/tiago-delivery/ros2_ws/src/perception/models/best.pt`

2. **box_3d_from_depth_node.py**
   - 2D bbox + Depth 이미지로 3D 위치 계산
   - 입력: 
     - `/perception/box_detection_2d`
     - `/gemini2/depth/image_raw`
     - `/gemini2/depth/camera_info`
   - 출력: `/perception/box_point_cam` (PointStamped)
   - ROI 기반 median depth 추출 (robust)
   - Pinhole 카메라 모델 사용

3. **qr_reader_node.py**
   - QR 코드 읽기 (부가 기능)

#### Manipulation (제어)
**파일 위치**: `ros2_ws/src/manipulation/manipulation/approach_box.py`

- **Action Server**: `/approach_box`
- **Action 타입**: `interfaces/action/ApproachBox.action`
- **구독**: `/perception/box_point_cam`
- **발행**: `/cmd_vel`

**제어 알고리즘**:
```python
bearing = atan2(x, z)  # 박스까지의 각도
omega = -kp_ang * bearing  # 회전 속도
v = kp_lin * (z - stop_distance)  # 전진 속도
```

**상태**:
- `NO_TARGET`: 박스 미감지
- `ALIGNING`: 방향 정렬 중
- `APPROACHING`: 전진 중
- `ARRIVED`: 목표 도달

**파라미터**:
- `stop_distance`: 목표 정지 거리 (m)
- `timeout_sec`: 최대 실행 시간 (s)
- `align_first`: 정렬 후 접근 여부
- `kp_ang`: 회전 게인 (기본 1.8)
- `kp_lin`: 전진 게인 (기본 0.6)
- `max_linear`: 최대 선속도 (0.35 m/s)
- `max_angular`: 최대 각속도 (0.9 rad/s)

### 4. 실행 환경 구축

#### 생성된 파일들:

1. **launch_all.sh** (`~/tiago-delivery/launch_all.sh`)
   - 전체 시스템을 한 번에 실행하는 스크립트
   - Perception + Manipulation 노드 자동 실행

2. **manipulation.launch.py** (`ros2_ws/src/manipulation/launch/manipulation.launch.py`)
   - Manipulation 노드 실행용 launch 파일
   - 모든 파라미터 설정 포함

3. **README_QUICK_START.md** (`~/tiago-delivery/README_QUICK_START.md`)
   - 빠른 시작 가이드
   - 실행 방법, 테스트 방법, 트러블슈팅 포함

#### 빌드 완료:
```bash
cd ~/tiago-delivery/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install  # ✅ 완료
```

**해결한 이슈**:
- empy 버전 문제: `pip3 install empy==3.3.4`로 해결

## 🚀 실행 방법

### 간편 실행
```bash
cd ~/tiago-delivery
./launch_all.sh
```

### 개별 실행
```bash
# Terminal 1
source ~/tiago-delivery/ros2_ws/install/setup.bash
ros2 launch receiver_detection perception.launch.py

# Terminal 2
source ~/tiago-delivery/ros2_ws/install/setup.bash
ros2 launch manipulation manipulation.launch.py
```

### 박스 접근 테스트
```bash
ros2 action send_goal --feedback /approach_box interfaces/action/ApproachBox \
"{stop_distance: 0.5, timeout_sec: 20.0, min_confidence: 0.0, align_first: true}"
```

### Tiago 컨트롤러 GUI
```bash
cd ~/tiago-delivery/external/tiago_isaac
python3 tiago_example_controller.py
```

## 📁 주요 파일 구조

```
~/tiago-delivery/
├── external/tiago_isaac/
│   └── tiago_example_controller.py  # GUI 컨트롤러
├── ros2_ws/
│   └── src/
│       ├── interfaces/
│       │   ├── action/ApproachBox.action
│       │   └── msg/BoxDetection2D.msg
│       ├── perception/receiver_detection/
│       │   ├── yolo_box_detector_node.py
│       │   ├── box_3d_from_depth_node.py
│       │   └── qr_reader_node.py
│       └── manipulation/
│           └── approach_box.py
├── launch_all.sh  # 전체 실행 스크립트
└── README_QUICK_START.md  # 빠른 시작 가이드
```

## 🔄 전체 파이프라인

```
RGB Camera → YOLO → 2D bbox → Depth 융합 → 3D 위치 → Action Server → cmd_vel → 로봇 이동
```

## ⚠️ 다음 작업 (EC2 환경 설정 대기 중)

사용자가 EC2 인스턴스를 켜면:
- EC2 환경 정보 확인 (사용자명, 경로, ROS 버전 등)
- 경로 및 설정을 EC2 환경에 맞게 수정

## 📝 토픽 요약

### 구독 (Subscribe)
- `/gemini2/color/image_raw` - RGB 이미지
- `/gemini2/depth/image_raw` - Depth 이미지
- `/gemini2/depth/camera_info` - 카메라 정보
- `/perception/box_detection_2d` - 2D 박스 감지 결과
- `/perception/box_point_cam` - 3D 박스 위치
- `/joint_states` - 로봇 관절 상태

### 발행 (Publish)
- `/perception/box_detection_2d` - 2D 박스 (YOLO)
- `/perception/box_point_cam` - 3D 박스 위치
- `/joint_command` - 관절 명령
- `/cmd_vel` - 속도 명령

### Action
- `/approach_box` - 박스 접근 액션
