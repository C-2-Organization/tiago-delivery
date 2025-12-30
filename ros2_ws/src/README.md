# TIAGO Delivery – Perception Module (2D Box + QR + 3D Stub)

이 문서는 `tiago-delivery/ros2_ws/src/` 안에서 **Perception 담당자가 관리하는 패키지들**을
처음 보는 팀원도 **그대로 따라 실행**할 수 있게 정리한 README입니다.

> ✅ 목표
> - YOLO로 박스 2D BBox 검출 → `/perception/box_detection_2d` publish
> - QR 체크 노드(카메라 입력 기반) → 필요 토픽 publish
> - 3D는 현재 **Stub 기반**으로 `/perception/box_pose_3d` publish (센서 없이도 인터페이스 검증 가능)
>
> ❌ Perception 범위 아님
> - MoveIt / 로봇 제어 / 상태머신 / Pick&Place 실행 (orchestrator, manipulation 담당)

---

## 1. 현재 `ros2_ws/src/` 전체 구조 (팀 공통)

```text
ros2_ws/src/
├── interfaces/          # 공용 msg / srv / action 정의 (모든 패키지 공통)
├── perception/          # Perception "그룹 폴더" (⚠️ ROS 패키지 아님)
│   ├── tiago_box_check/ # ✅ 2D Box + QR (ROS2 Python 패키지)
│   ├── Box_pose_3d/     # ✅ 3D Box Pose (Stub 중심, ROS2 Python 패키지)
│   ├── build/           # ⚠️ 과거 perception 폴더에서 잘못 colcon build 해서 생긴 잔여물
│   ├── install/         # ⚠️ 사용 안 함 (워크스페이스 루트 install만 사용)
│   └── log/             # ⚠️ 사용 안 함
├── orchestrator/        # Task / 상태 관리 (다른 팀원 담당)
├── manipulation/        # Pick & Place 실행 (다른 팀원 담당)
├── tiago_rviz2/         # RViz 설정
└── README.md
```

### 중요

* `perception/` 자체는 **ROS 패키지가 아닙니다.**
* 실제 ROS 패키지는 아래 2개입니다.

  * `perception/tiago_box_check`
  * `perception/Box_pose_3d`
* **빌드는 반드시 `ros2_ws` 루트에서만** 수행합니다.

---

## 2. Perception 내부 패키지 구조 상세 (현재 상태 반영)

```text
perception/
├── tiago_box_check/                         # ROS2 패키지 (2D Detection + QR)
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── resource/
│   │   └── tiago_box_check
│   ├── test/
│   └── tiago_box_check/
│       ├── __init__.py
│       ├── config/                          # (필요 시) 모델/파라미터/규칙 파일
│       ├── io/
│       │   ├── __init__.py
│       │   ├── publishers.py
│       │   └── make_qr.py
│       ├── nodes/
│       │   ├── __init__.py
│       │   ├── yolo_box_detector_node.py    # ✅ YOLO 2D 박스 검출 노드
│       │   └── qr_checker.py                # ✅ QR 체크 노드
│       ├── preprocessing/
│       │   ├── __init__.py
│       │   └── image_preprocess.py
│       └── parsing/
│           ├── __init__.py
│           └── destination_parser.py
│
└── Box_pose_3d/                             # ROS2 패키지 (3D Pose Estimation)
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    │   └── Box_pose_3d
    ├── test/
    ├── config/
    │   ├── T_cam_to_base.txt                # (향후 확장용) extrinsic placeholder
    │   └── T_lidar_to_cam.txt               # (향후 확장용) extrinsic placeholder
    └── Box_pose_3d/
        ├── __init__.py
        ├── Box_3d_estimator_node.py         # ⭐ 현재 사용 중인 3D Stub
        └── box_3d_estimator_real.py         # (향후 Depth/LiDAR 확장용 스켈레톤)
```

---

## 3. Perception 파이프라인 개요 (현재 팀 합의 기준)

```text
[ RGB Camera ]
      ↓
YOLO (tiago_box_check)
→ 2D Bounding Box
      ↓
(Box_pose_3d)
→ 3D Position (Stub / 향후 Depth·LiDAR)
      ↓
Publish
→ /perception/box_pose_3d
```

---

## 4. 사용되는 ROS2 토픽 & 인터페이스

### 4.1 2D Box Detection 결과

* 토픽: `/perception/box_detection_2d`
* 타입: `interfaces/msg/BoxDetection2D`

### 4.2 3D Box Pose 결과

* 토픽: `/perception/box_pose_3d`
* 타입: `interfaces/msg/BoxPose3D`

> Orchestrator / Manipulation 팀은
> 최소한 `/perception/box_pose_3d`만 구독하면 다음 단계로 넘어갈 수 있습니다.

---

## 5. 역할 분리 기준 (팀 협업 규칙)

### Perception 담당 (이 레포에서 하는 일)

* YOLO 2D 박스 검출
* QR 인식
* 3D 위치 추정 (Stub 또는 Real)
* 위 결과를 **ROS2 토픽으로 publish 까지만 책임**

### Orchestrator 담당

* `/perception/box_pose_3d` 구독
* 상태 머신 관리
* Manipulation 호출

### Manipulation 담당

* 실제 로봇 Pick & Place 수행

❌ **Perception에서는 MoveIt, 로봇 제어, 상태 머신을 구현하지 않습니다.**

---

## 6. 실행 전 필수 환경 설정 (모든 터미널 공통)

```bash
source /opt/ros/humble/setup.bash
source ~/tiago-delivery/ros2_ws/install/setup.bash
```

> 빌드 직후엔 `install/setup.bash`를 다시 source 해야 합니다.

---

## 7. 빌드 방법 (중요: 항상 워크스페이스 루트에서만)

```bash
cd ~/tiago-delivery/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 8. 노드 실행 방법

### 8.1 YOLO 2D Box Detector 실행

```bash
ros2 run tiago_box_check yolo_box_detector --ros-args \
  -p image_topic:=/gemini2/color/image_raw \
  -p bbox_topic:=/perception/box_detection_2d
```

* `image_topic`은 환경에 따라 바뀔 수 있으니 **파라미터로 교체 가능**하게 되어있습니다.
* YOLO 모델은 기본 `yolov8n.pt`이며, 최초 실행 시 자동 다운로드될 수 있습니다.

> ⚠️ 주의 (자주 하는 실수)
> 아래처럼 `\` 뒤에 공백이 있으면 명령이 깨집니다.
>
> * 틀림: `\ ` (백슬래시 뒤 공백)
> * 맞음: `\` (백슬래시 바로 줄바꿈)

### 8.2 QR Checker 실행

```bash
ros2 run tiago_box_check qr_checker --ros-args \
  -p image_topic:=/gemini2/color/image_raw
```

---

### 8.3 3D Box Pose Stub 실행 (센서 없어도 테스트 가능)

```bash
ros2 run Box_pose_3d box_3d_estimator_stub
```

* 이 노드는 “실센서가 없어도” `/perception/box_pose_3d` 인터페이스를 먼저 고정하기 위한 목적입니다.
* Orchestrator/Manipulation이 먼저 연결 테스트 가능하도록 만든 구성입니다.

---

## 9. 토픽/인터페이스 확인 (처음 세팅한 사람 필수)

### 토픽 확인

```bash
ros2 topic list | grep perception
```

### 인터페이스 확인

```bash
ros2 interface show interfaces/msg/BoxDetection2D
ros2 interface show interfaces/msg/BoxPose3D
```

---

## 10. 더미 데이터로 전체 파이프라인 테스트 (카메라/YOLO 없이도 가능)

### 10.1 더미 2D BBox publish

```bash
ros2 topic pub --once /perception/box_detection_2d interfaces/msg/BoxDetection2D "{
  header: {frame_id: 'camera_link'},
  source: 'dummy',
  label: 'box',
  confidence: 0.9,
  x1: 100, y1: 100, x2: 300, y2: 300,
  ocr_text: '',
  ocr_confidence: 0.0
}"
```

### 10.2 Stub 출력 확인

```bash
ros2 topic echo /perception/box_pose_3d
```

---

## 11. 자주 발생하는 문제 / 해결

### 11.1 `interfaces/msg/... is invalid` 또는 토픽 타입을 못 찾는 경우

대부분 아래 중 하나입니다.

1. workspace 환경을 source 안 함

```bash
source /opt/ros/humble/setup.bash
source ~/tiago-delivery/ros2_ws/install/setup.bash
```

2. build 후 source를 다시 안 함

```bash
cd ~/tiago-delivery/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

3. 다른 터미널에서 아직 옛 환경을 사용 중

* 새 터미널 열고 source부터 다시 하세요.

---

### 11.2 `Failed to parse global arguments` (파라미터 인자 파싱 실패)

예: `-p class_name:=""` 같은 형태가 깨질 수 있습니다.

* 빈 문자열을 넣고 싶으면 아예 파라미터를 주지 않거나,
* 안전하게 작은따옴표로 감싸세요.

```bash
# 권장: 파라미터를 생략(필터링 없이 전체 검출)
ros2 run tiago_box_check yolo_box_detector --ros-args \
  -p image_topic:=/gemini2/color/image_raw \
  -p bbox_topic:=/perception/box_detection_2d

# 혹은 빈문자열을 꼭 넣어야 한다면
ros2 run tiago_box_check yolo_box_detector --ros-args \
  -p class_name:=''
```

---

### 11.3 NumPy / cv_bridge 관련 에러(예: `_ARRAY_API not found`)가 났던 이력

* 시스템에 따라 `numpy` 버전, `opencv-python`, `cv_bridge` 조합이 꼬일 수 있습니다.
* 해결 방향은 환경마다 다르므로 “팀 공통 기준”을 정해서 맞추는 게 안전합니다.

**권장 운영**

* ROS2(apt) 기반 `cv_bridge`를 쓰는 환경이면 pip로 opencv/numpy를 과하게 바꾸지 않는 방식 권장
* 이미 꼬였으면 “프로젝트 venv / docker”로 고정하는 것이 가장 안전

> 이 README는 “동작하는 구조와 실행법”을 문서화한 것이고,
> 특정 PC에서의 numpy/opencv 충돌 해결은 별도 이슈로 관리하는 것을 권장합니다.

---

## 12. 팀원이 수정해야 할 포인트 (환경 맞춤)

### 12.1 토픽 이름

실제 카메라 토픽이 다르면 아래 파라미터만 바꾸면 됩니다.

* YOLO:
  * `image_topic`
  * `bbox_topic`

* QR checker:
  * `image_topic`

> 토픽 이름은 하드코딩하지 않고 파라미터로 교체하도록 설계되어 있습니다.

### 12.2 모델 파일

YOLO 모델을 바꾸려면(예: `yolov8s.pt`):

```bash
ros2 run tiago_box_check yolo_box_detector --ros-args \
  -p image_topic:=/gemini2/color/image_raw \
  -p bbox_topic:=/perception/box_detection_2d \
  -p model_path:=yolov8s.pt
```

---

## 13. 요약

* `perception`은 “그룹 폴더”이고 ROS 패키지가 아님
* 실제 ROS 패키지는:
  * `tiago_box_check`
  * `Box_pose_3d`
* Perception의 최종 출력(핵심 토픽):
  * `/perception/box_detection_2d`
  * `/perception/box_pose_3d`
* 빌드는 항상 `ros2_ws` 루트에서만 수행
