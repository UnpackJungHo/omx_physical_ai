# OMX 프로젝트 — 다음 세션 이어하기

<!-- AUTO_PHASE_SUMMARY_START -->
## 단계 상태
- `A` 완료 — 레퍼런스 코드 읽기
- `B` 완료 — 로봇 모델 표시
- `C` 완료 — ros2_control 붙이기 (mock 모드)
- `D` 완료 — MoveIt2 붙이기
- `E` 롤백 — 통합 launch 파일 (의미 없다고 판단, 삭제)

## 현재 작업 단계
- `omx_motion_server` 완료. 다음은 `omx_perception` (4단계)

## 승인 대기
- 없음
<!-- AUTO_PHASE_SUMMARY_END -->

---

## 4단계: omx_perception

### 목적
카메라로 블록(빨강/파랑/초록)의 위치를 감지하고 `GetBlockPoses` 서비스로 제공한다.
`omx_skill_executor`는 이 서비스만 호출하고 카메라를 직접 참조하지 않는다.

### 언어 전략
Python (`ament_python`). OpenCV 컬러 감지는 Python이 적합하다.

### 구현할 인터페이스
| 인터페이스 | 토픽/서비스 | 역할 |
|------------|-------------|------|
| `GetBlockPoses` 서비스 | `/omx/get_block_poses` | 블록 위치 반환 |
| `BlockPose` 메시지 | (서비스 내부) | 색상 + PoseStamped + confidence |

### 구현 순서

#### G-1. 카메라 입력 확인
- `usb_cam` 또는 `v4l2_camera` 노드로 `/camera/image_raw` 확인
- 카메라 캘리브레이션 파라미터 확인

#### G-2. 컬러 기반 블록 감지
- HSV 색공간에서 빨강/파랑/초록 마스크 생성
- 컨투어로 블록 중심 픽셀 좌표 추출
- 깊이 또는 단순 평면 가정으로 3D 좌표 변환

#### G-3. GetBlockPoses 서비스 서버
- 감지된 블록 목록을 `omx_interfaces/BlockPose[]`로 반환
- color 파라미터 빈 문자열이면 전체 반환

### 확인 방법
```bash
ros2 service call /omx/get_block_poses omx_interfaces/srv/GetBlockPoses "{color: ''}"
# 기대: 감지된 블록 목록 반환
```

---

## omx_bringup 완료 상태 요약

bringup은 두 개의 독립 launch로 운영한다.

```bash
# 터미널 1: ros2_control + 컨트롤러 + home 이동
ros2 launch omx_bringup omx_control.launch.py use_mock_hardware:=true

# 터미널 2: MoveIt2 move_group + workspace guard + RViz
ros2 launch omx_bringup omx_moveit.launch.py
```

| 파일 | 역할 |
|------|------|
| `launch/display_robot.launch.py` | URDF + RViz (Phase B) |
| `launch/omx_control.launch.py` | ros2_control + spawner + home pose (Phase C) |
| `launch/omx_moveit.launch.py` | move_group + workspace_guard + RViz MoveIt (Phase D) |
| `omx_bringup/workspace_guard.py` | floor/ceiling collision object 추가 |

---

## 2단계 완료: omx_interfaces

### 완료 상태
패키지 간 통신 계약을 코드보다 먼저 고정했다.
상위 패키지(skill, planner, LLM)는 이후 이 인터페이스를 통해서만 하위 레이어를 호출한다.

확인된 패키지 파일
- `src/omx_interfaces/CMakeLists.txt`
- `src/omx_interfaces/package.xml`
- `src/omx_interfaces/msg/BlockPose.msg`
- `src/omx_interfaces/srv/GetBlockPoses.srv`
- `src/omx_interfaces/action/MoveToNamed.action`
- `src/omx_interfaces/action/MoveToPose.action`
- `src/omx_interfaces/action/GripperCommand.action`
- `src/omx_interfaces/action/PickPlace.action`

### 정의된 인터페이스

#### Action — `action/MoveToNamed.action`
```
# Goal
string name          # "home" | "ready" | "pre_grasp" | "stow"
---
# Result
bool success
string message
---
# Feedback
string status
```

#### Action — `action/MoveToPose.action`
```
# Goal
geometry_msgs/PoseStamped target_pose
float32 velocity_scale   # 0.0 ~ 1.0, 기본값 0.5
---
# Result
bool success
string message
---
# Feedback
float32 progress         # 0.0 ~ 1.0
string status
```

#### Action — `action/GripperCommand.action`
```
# Goal
float32 position         # 0.0 = 완전 닫힘, 1.0 = 완전 열림
float32 max_effort       # N, 기본값 0 (컨트롤러 기본값 사용)
---
# Result
bool success
float32 position         # 실제 도달 위치
string message
---
# Feedback
float32 position
```

#### Action — `action/PickPlace.action`
```
# Goal
string object_color      # "red" | "blue" | "green"
string target_box        # "left" | "right"
bool retry_on_fail       # 실패 시 재시도 허용 여부
---
# Result
bool success
string message
int32 attempts
---
# Feedback
string phase             # "detecting" | "approaching" | "grasping" | "placing"
string status
```

#### Message — `msg/BlockPose.msg`
```
std_msgs/Header header
string color             # "red" | "blue" | "green"
geometry_msgs/PoseStamped pose
float32 confidence       # 0.0 ~ 1.0
```

#### Service — `srv/GetBlockPoses.srv`
```
string color             # 빈 문자열이면 전체 색상 반환
---
omx_interfaces/BlockPose[] blocks
```

### 확인 방법
```bash
colcon build --packages-select omx_interfaces
ros2 interface list | grep omx
ros2 interface show omx_interfaces/action/MoveToNamed
```

기대 결과
- `omx_interfaces` 빌드 성공
- action / msg / srv 계약이 패키지 레벨에서 고정됨

---

## 3단계: omx_motion_server

### 목적
MoveIt2를 감싸는 안전한 모션 API 서버를 제공한다.
이 노드가 동작하면 `ros2 action send_goal` 한 줄로 팔이 실제로 움직인다.
skill 레이어는 이 API만 호출하고, MoveIt2를 직접 참조하지 않는다.

주의
- `PickPlace.action`은 이미 인터페이스에 정의돼 있지만, 실제 구현 책임은 `omx_skill_executor`에 둔다.
- `omx_motion_server`는 `MoveToNamed`, `MoveToPose`, `GripperCommand` 세 액션에 집중한다.

### 언어 전략
C++ (`ament_cmake`). MoveGroupInterface, MoveItCpp, 그리퍼 제어는 실시간 경로에 속한다.

### 패키지 생성
```bash
cd ~/omx_ws/src
ros2 pkg create omx_motion_server --build-type ament_cmake \
  --dependencies rclcpp rclcpp_action omx_interfaces \
  moveit_ros_planning_interface geometry_msgs
```

### 구현할 action server

| Action | Topic | 역할 |
|--------|-------|------|
| `MoveToNamed` | `/omx/move_to_named` | named pose로 이동 |
| `MoveToPose` | `/omx/move_to_pose` | Cartesian pose로 이동 |
| `GripperCommand` | `/omx/gripper_command` | 그리퍼 열기/닫기 |

### named pose 목록
| 이름 | 설명 |
|------|------|
| `home` | joint2=-1.57, joint3=1.57, joint4=1.57 |
| `ready` | 작업 시작 전 대기 자세 |
| `stow` | 접힌 안전 자세 |

### 핵심 구현 순서

#### F-1. MoveToNamed action server
- `MoveGroupInterface::setNamedTarget()` 사용
- SRDF에 등록된 named state와 1:1 대응
- 없는 이름 요청 시 `success=false` + message 반환

#### F-2. MoveToPose action server
- `MoveGroupInterface::setPoseTarget()` 사용
- workspace bound 벗어난 pose 요청 시 즉시 거부
- `velocity_scale` 파라미터로 속도 제한

#### F-3. GripperCommand action server
- `gripper_controller/GripperCommand` action으로 전달
- position 범위 클램핑 (0.0 ~ 1.0)

### 확인 방법

```bash
# bringup 먼저
ros2 launch omx_bringup omx_control.launch.py use_mock_hardware:=true
ros2 launch omx_bringup omx_moveit.launch.py start_rviz:=false

# motion server 실행
ros2 run omx_motion_server motion_server

# home으로 이동
ros2 action send_goal /omx/move_to_named omx_interfaces/action/MoveToNamed \
  "{name: 'home'}"

# 그리퍼 열기
ros2 action send_goal /omx/gripper_command omx_interfaces/action/GripperCommand \
  "{position: 1.0, max_effort: 0.0}"
```

기대 결과: RViz에서 arm이 해당 포즈로 이동하는 것 확인.

---

## 현재 파일 구조

```text
omx_ws/src/
├── omx_bringup/              ← 완료
│   ├── launch/
│   │   ├── display_robot.launch.py
│   │   ├── omx_control.launch.py
│   │   └── omx_moveit.launch.py
│   └── omx_bringup/
│       └── workspace_guard.py
├── omx_interfaces/           ← 완료
│   ├── action/
│   ├── msg/
│   └── srv/
└── omx_motion_server/        ← 다음 작업
```

## 의존 패키지 (설치 확인)
```bash
ros2 pkg list | grep open_manipulator
# open_manipulator_bringup
# open_manipulator_description
# open_manipulator_moveit_config
```
