# OMX 프로젝트 — 다음 세션 이어하기

<!-- AUTO_PHASE_SUMMARY_START -->
## 단계 상태
- `A` 완료 — 레퍼런스 코드 읽기
- `B` 완료 — 로봇 모델 표시
- `C` 완료 — ros2_control 붙이기 (mock 모드)
- `D` 완료 — MoveIt2 붙이기
- `E` 롤백 — 통합 launch 파일 (의미 없다고 판단, 삭제)

## 현재 작업 단계
- `omx_perception` 완료 (4단계), `omx_skill_executor` 구현 완료 (5단계)
- 다음: 실기기에서 end-to-end 테스트 및 파라미터 튜닝

## 승인 대기
- 없음
<!-- AUTO_PHASE_SUMMARY_END -->

---

## 내일 바로 볼 핵심 메모

### 현재 판단
- 지금은 로컬 단일 머신 기능을 전부 끝내는 단계가 아니다.
- `대표 시나리오 1개를 안정적으로 닫고 바로 edge-cloud 구조로 넘어가는 것`이 포트폴리오 가치가 가장 높다.

### 당장 닫아야 할 최소 범위
1. `omx_perception` 안정화
2. `omx_skill_executor`에서 `빨간 블록을 왼쪽 상자에 넣기` 구현
3. 최소 `retry` 또는 `safe stop` 동작 추가
4. 반복 실행으로 성공률 확인

### 여기서 멈추고 다음으로 넘어가는 기준
- perception -> skill -> motion이 end-to-end로 연결된다.
- 실기기에서 대표 시나리오가 반복 가능하게 성공한다.
- 실패 시 최소한의 설명 가능한 복구 동작이 있다.
- 영상, 로그, 성공률 중 최소 두 가지로 증명 가능하다.

### 그 다음 바로 할 일
1. `Pi 5 + 노트북 서버` 역할 분리
2. edge에 `perception`, `motion_server`, 최소 `recovery`
3. server에 `task_planner`, `mission_control`, `telemetry`
4. QoS 프로파일 설계
5. network degradation 실험 추가

### 하지 말아야 할 것
- 로컬 머신에서 `align`, `stack`, `LLM`까지 다 끝내고 넘어가려는 것
- QoS/DDS를 말만 하고 실험 없이 문서에만 적는 것
- 실시간 제어 경로에 서버 또는 LLM 의존성을 넣는 것

---

## 포트폴리오 전환 메모

### 새 기준 문서
- `docs/PORTFOLIO_STRATEGY.md`를 먼저 본다.

### 프로젝트 한 줄 정의
- `ROS2/DDS 기반으로 OMX 매니퓰레이터를 엣지 제어 노드와 서버 관제 노드로 분리하고, 네트워크 장애와 QoS까지 다루는 network-adaptive manipulation system`

### 왜 이 방향인가
- 현재 구조는 이미 괜찮은 로보틱스 프로젝트다.
- 그러나 취업 포트폴리오에서 차이를 만드는 것은 기능 개수보다 `시스템 설계 판단`이다.
- 따라서 `단일 로봇 데모`에서 `edge-cloud 분산 시스템`으로 축을 옮긴다.

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

### 완료 후 바로 이어질 작업
- `omx_skill_executor`를 생성한다.
- 최초 목표는 `PickPlace.action` 전체 구현이 아니라 대표 시나리오 1개를 rule-based로 끝까지 연결하는 것이다.
- 이 단계에서 필요한 recovery는 최소한의 retry / safe stop 수준으로 제한한다.

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
