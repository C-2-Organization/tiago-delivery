# Tiago Delivery - Quick Start Guide

## 🏗️ 시스템 구성

### Perception (박스 감지)
- **yolo_box_detector**: YOLO로 2D 박스 감지
- **box_3d_from_depth**: Depth 카메라로 3D 위치 계산
- **qr_reader**: QR 코드 읽기

### Manipulation (박스 접근)
- **approach_box**: Action server로 박스 앞까지 자동 이동

## 🚀 실행 방법

### 1. 빌드 (최초 1회)
```bash
cd ~/tiago-delivery/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. 전체 시스템 실행 (간편)
```bash
cd ~/tiago-delivery
chmod +x launch_all.sh
./launch_all.sh
```

### 3. 개별 실행
```bash
# Terminal 1: Perception
source ~/tiago-delivery/ros2_ws/install/setup.bash
ros2 launch receiver_detection perception.launch.py

# Terminal 2: Manipulation
source ~/tiago-delivery/ros2_ws/install/setup.bash
ros2 launch manipulation manipulation.launch.py
```

## 🎯 박스 접근 테스트

### Action 호출
```bash
ros2 action send_goal --feedback /approach_box interfaces/action/ApproachBox \
"{stop_distance: 0.5, timeout_sec: 20.0, min_confidence: 0.0, align_first: true}"
```

### 파라미터 설명
- `stop_distance`: 박스로부터 정지할 거리 (미터)
- `timeout_sec`: 최대 실행 시간 (초)
- `min_confidence`: 최소 신뢰도 (0.0~1.0)
- `align_first`: true면 먼저 방향 정렬 후 접근

## 📊 토픽 모니터링

### 박스 감지 확인
```bash
# 2D 박스 감지 결과
ros2 topic echo /perception/box_detection_2d

# 3D 박스 위치
ros2 topic echo /perception/box_point_cam

# 로봇 속도 명령
ros2 topic echo /cmd_vel
```

### Action 상태 확인
```bash
# Action 서버 확인
ros2 action list

# Action 정보
ros2 action info /approach_box
```

## 🎮 Tiago 컨트롤러 GUI 실행

```bash
cd ~/tiago-delivery/external/tiago_isaac
python3 tiago_example_controller.py
```

### 키보드 제어
- `W/S`: 전진/후진
- `A/D`: 좌/우 이동  
- `←/→`: 회전

## 🔧 트러블슈팅

### 빌드 에러
```bash
# 의존성 설치
sudo apt install python3-pip
pip3 install ultralytics opencv-python pyzbar pyyaml numpy
```

### YOLO 모델 없음
```bash
# 모델 다운로드 (필요시)
cd ~/tiago-delivery/ros2_ws/src/perception
# 모델 경로: models/best.pt 확인
```

### 카메라 토픽 확인
```bash
ros2 topic list | grep gemini2
# /gemini2/color/image_raw
# /gemini2/depth/image_raw
# /gemini2/depth/camera_info
```

## 📁 주요 파일 위치

- Perception: `ros2_ws/src/perception/receiver_detection/`
- Manipulation: `ros2_ws/src/manipulation/`
- Interfaces: `ros2_ws/src/interfaces/`
- Tiago Controller: `external/tiago_isaac/tiago_example_controller.py`

## 🎨 시스템 파이프라인

```
카메라 → YOLO 감지 → 2D bbox → Depth 융합 → 3D 위치 → Action 제어 → 로봇 이동
```
