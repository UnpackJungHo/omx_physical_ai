# Box + Cup Pose Perception 재설계

- 작성일: 2026-05-04
- 대상 패키지: `omx_perception`, `omx_interfaces`
- 트리거: 새 YOLOv8-Pose 2-class 모델 (`runs/pose/box_cup_pose_2class_96/weights/best.pt`) 도입. 기존 단일 클래스(Box) 파이프라인을 다중 클래스(Box+Cup)로 확장.

## 1. 목표 / 비목표

**목표**
- YOLO 추론 노드가 `Box`(class 0)와 `Cup`(class 1) 두 클래스를 모두 출력.
- World-pose 노드가 클래스별 3D 모델로 PnP를 풀어 `BlockPose[]` 응답에 두 종류를 모두 포함.
- 토픽/서비스/실행자 이름을 새 컨셉(`box_cup_*`)에 맞게 정리.
- 기존 Box 동작과의 회귀 없음(Box-only 입력에서 동일한 결과).

**비목표**
- Cup 자체를 잡는 동작(grasping cup). Cup은 박스를 떨어뜨려 넣을 placement target임.
- Cup의 6-DOF orientation 활용. 이번 단계에서는 림 중심 좌표만 사용.
- `BlockPose` 메시지 스키마 변경. `color` 필드를 `"box"`/`"cup"`으로 재해석하여 사용.
- 색 기반 블록 검출 파이프라인 통합.

## 2. 영향 범위

코드 검색(`grep`) 결과, `Top4Box` / `GetTop4Keypoints` / `top4_pose` / `get_top4_world_poses`는 `omx_perception`과 `omx_interfaces`에서만 참조됨. `omx_skill_executor` / `omx_motion_server`는 `/omx/get_block_poses` (별도 서비스)를 사용하므로 영향 없음. → 이름 변경 안전.

## 3. 인터페이스 변경 (`omx_interfaces`)

### 3.1 새 메시지: `KeypointDetection.msg`
기존 `Top4Box.msg`를 대체. 클래스 정보 추가.
```
uint8 CLASS_BOX = 0
uint8 CLASS_CUP = 1

uint8 class_id              # 0=Box, 1=Cup
string class_name           # "box" | "cup"
float32 detection_confidence

# 4 keypoints: [x, y, conf] * 4 픽셀좌표.
# class_id == CLASS_BOX: 큐브 윗면 4 모서리 (top-left, top-right, bottom-right, bottom-left).
# class_id == CLASS_CUP: 컵 림(rim)의 4 cardinal 점.
float32[12] keypoints
```

### 3.2 새 서비스: `GetKeypointDetections.srv`
기존 `GetTop4Keypoints.srv`를 대체.
```
bool publish_debug
---
std_msgs/Header header
bool success
string message
omx_interfaces/KeypointDetection[] detections
```

### 3.3 `BlockPose.msg`
스키마 유지, 의미 확장. 코멘트만 갱신.
```
string color   # "box" | "cup"  (legacy: was red/blue/green)
```

### 3.4 `CMakeLists.txt`
- `msg/Top4Box.msg` 제거 → `msg/KeypointDetection.msg` 등록
- `srv/GetTop4Keypoints.srv` 제거 → `srv/GetKeypointDetections.srv` 등록
- 옛 메시지/서비스 파일은 git에서 제거.

## 4. 노드 변경 (`omx_perception`)

### 4.1 파일/실행자/노드명
| 종류 | 기존 | 신규 |
|---|---|---|
| 파일 | `top4_keypoints_node.py` | `box_cup_pose_node.py` |
| 파일 | `top4_world_pose_node.py` | `box_cup_world_pose_node.py` |
| executable | `top4_keypoints_node` | `box_cup_pose_node` |
| executable | `top4_world_pose_node` | `box_cup_world_pose_node` |
| ROS node name | `top4_keypoints` | `box_cup_pose` |
| ROS node name | `top4_world_pose` | `box_cup_world_pose` |

### 4.2 토픽/서비스
| 종류 | 기존 | 신규 |
|---|---|---|
| Image (annotated) | `/image/raw/top4_pose` | `/image/raw/box_cup_pose` |
| Service | `/perception/get_top4_keypoints` | `/perception/get_box_cup_keypoints` |
| Service | `/perception/get_top4_world_poses` | `/perception/get_box_cup_world_poses` |

응답 srv 타입은 world 쪽이 `GetBlockPoses` 그대로(스키마 변화 없음).

### 4.3 `box_cup_pose_node` 동작

기존 `top4_keypoints_node`와 거의 동일하되:
- YOLO 결과에서 `result.boxes.cls`(클래스 인덱스)도 함께 추출.
- `KeypointDetection.class_id`/`class_name` 채움 (`{0:"box", 1:"cup"}`).
- 각 detection에 대해 4 keypoint를 그대로 메시지에 실음.
- 디버그 시각화: 클래스별 색을 분리.
  - Box: 노랑/시안/녹/마젠타 등 기존 색팔레트 유지(인스턴스별 순환).
  - Cup: 별도 색(예: 주황 단일색 또는 별도 팔레트). 라벨 텍스트에 `box`/`cup` 표시.
  - Box는 4점을 사변형으로 잇는 선 유지. Cup은 4점만 점으로 표시(폴리곤 라인 생략).
- 서비스 응답: `detections`에 두 클래스 인스턴스가 confidence 내림차순으로 들어감.

### 4.4 `box_cup_world_pose_node` 동작

각 `KeypointDetection`마다 클래스별 분기:

**Box (`class_id == 0`)**
- 기존 로직 유지.
- object_points: `half = cube_size_m / 2`로 큐브 윗면 4 corners
  ```
  [(-half, -half, 0), (half, -half, 0), (half, half, 0), (-half, half, 0)]
  ```
- `solvePnP IPPE` (ITERATIVE fallback).
- 결과 z = `box_output_z_m` (default 0.015 = 큐브 중심 높이).
- `BlockPose.color = "box"`.

**Cup (`class_id == 1`)**
- object_points: 림 4 cardinal 점, 반지름 `cup_radius_m` (default 0.07)
  ```
  [(-r, 0, 0), (0, -r, 0), (r, 0, 0), (0, r, 0)]
  ```
  (Box와 keypoint index 0~3 의미가 다르더라도 회전만 다르고 translation은 동일하므로 cup center 계산에 문제 없음.)
- `solvePnP IPPE`.
- 결과 z = `cup_output_z_m` (default **0.08** = 컵 림 높이; 박스를 떨어뜨릴 drop target).
- `BlockPose.color = "cup"`.
- `BlockPose.pose`와 `BlockPose.grasp_pose`는 동일한 림 중심 pose로 채움(드롭 타겟; 다운스트림은 `color`로 box/cup을 분기).

**공통**
- `min_keypoint_confidence` 필터, TF 변환, frame 처리는 기존과 동일.
- 응답 confidence는 `KeypointDetection.detection_confidence`.

### 4.5 새/변경 파라미터 (`box_cup_world_pose_node`)
| 이름 | 기본값 | 비고 |
|---|---|---|
| `cube_size_m` | 0.030 | 기존 유지 |
| `box_output_z_m` | 0.015 | 기존 `output_z_m`에서 이름 변경 |
| `cup_radius_m` | 0.07 | 종이컵 림 반지름 |
| `cup_height_m` | 0.08 | 메타데이터(직접 사용 안 함, 로깅용) |
| `cup_output_z_m` | 0.08 | 림 중심 높이 = drop target z |
| `keypoint_order` | `[0,1,2,3]` | Box/Cup 공통 |
| `min_keypoint_confidence` | 0.10 | 기존 유지 |
| 서비스/프레임 파라미터 | 기존 유지 | 이름은 `box_cup_*`로 갱신 |

## 5. 빌드/런치 변경

### 5.1 `omx_perception/setup.py`
console_scripts:
```python
"box_cup_pose_node = omx_perception.box_cup_pose_node:main",
"box_cup_world_pose_node = omx_perception.box_cup_world_pose_node:main",
```

### 5.2 `omx_perception/launch/perception.launch.py`
- launch arg: `top4_*` → `box_cup_*` 이름 변경.
- default `model_path`: `/home/kjhz/omx_ws/runs/pose/box_cup_pose_2class_96/weights/best.pt`.
- 노드 이름/executable/토픽/서비스 모두 신규 이름 적용.
- 새 cup 파라미터 노출.

## 6. 데이터 흐름

```
usb_cam ──▶ /image/raw
            │
            ▼
   box_cup_pose_node
   ├─ /image/raw/box_cup_pose       (annotated debug Image, class별 색 구분)
   └─ /perception/get_box_cup_keypoints (srv: KeypointDetection[])
            │
            ▼
   box_cup_world_pose_node
   └─ /perception/get_box_cup_world_poses (srv: BlockPose[]; color="box"|"cup")
```

다운스트림(향후 skill_executor의 pick&place 로직)은 `color == "box"`로 grasp 후보를, `color == "cup"`으로 drop 타겟을 분리해 사용.

## 7. 검증

1. **빌드:**
   ```
   colcon build --symlink-install --packages-select omx_interfaces omx_perception
   ```
2. **소스:** `source install/setup.bash`
3. **정적 점검:** workspace에서 `Top4Box`, `GetTop4Keypoints`, `top4_pose`, `top4_keypoints`, `top4_world_pose`, `get_top4_world_poses` grep → 0 hit (인터페이스 빌드 산출물 제외).
4. **런타임:** `ros2 launch omx_perception perception.launch.py`
   - `ros2 topic list | grep box_cup_pose` → annotated 토픽 확인.
   - rqt_image_view로 box+cup overlay 확인.
   - `ros2 service call /perception/get_box_cup_keypoints omx_interfaces/srv/GetKeypointDetections "{publish_debug: true}"` → 두 클래스 detection 모두 포함.
   - `ros2 service call /perception/get_box_cup_world_poses omx_interfaces/srv/GetBlockPoses "{color: ''}"` → `BlockPose.color`에 `"box"`/`"cup"` 모두 출현.
5. **회귀:** `omx_skill_executor` / `omx_motion_server`는 빌드만 통과 확인(코드 변경 없음).

## 8. 리스크

- **Cup keypoint cardinal 매핑 정확도:** 라벨링 컨벤션이 카메라 각도에 따라 cardinal 점이 정확히 N/E/S/W에 대응하지 않을 수 있음. 이번 작업은 center(translation)만 사용하므로 문제 없음. orientation을 쓰는 시점에 추가 캘리브레이션/검증 필요.
- **`BlockPose.color` 의미 재정의:** 추후 색 기반 블록 검출과 충돌 가능. 현재 색 기반 검출 코드 없음 — 충돌 시 `BlockPose`에 `class_name` 별도 필드 추가가 깔끔하지만 이번 범위 밖.
- **모델 가중치 경로 의존:** `runs/pose/box_cup_pose_2class_96/weights/best.pt` 절대경로를 launch default로 사용. 가중치 파일을 패키지 share로 옮기는 것은 향후 과제.

## 9. 미해결 / 향후 과제

- Cup orientation을 활용한 정밀 drop pose(예: 림이 기울어진 컵 처리).
- 색 기반 블록 검출 도입 시 `BlockPose`에 `class`/`color` 분리.
- 모델 가중치 패키징(설치 경로 관리).
