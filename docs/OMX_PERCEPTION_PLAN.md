# OMX Perception 패키지 구조 및 단계별 구현 계획

## 문서 목적
- `omx_perception` 패키지를 한 번에 크게 구현하지 않고, 실패 위험이 낮은 순서로 단계적으로 완성하기 위한 작업 기준 문서다.
- 목표는 wrist-mounted RGB 카메라만으로 `3cm 정육면체 블록`의 `color`, `center x/y/z`, `yaw`, `confidence`를 안정적으로 추정해 `GetBlockPoses` 서비스로 제공하는 것이다.
- 최종적으로는 edge 쪽에서 동작 가능한 구조를 유지하되, 초기 학습과 튜닝은 개발 PC를 활용할 수 있게 설계한다.

## 범위와 전제
- 입력 센서: `RGB only`
- 대상 물체: `box` 한 종류만 인식
- 지원 색상: `red`, `green`, `blue`
- 블록 크기: 한 변 `0.03 m` 고정
- 배치 환경: 블록은 항상 테이블 위에 놓이며, 기본적으로 upright 상태를 가정
- 출력 정확도 초기 목표:
  - `x,y`: 평균 오차 `±5 mm` 이내
  - `z`: `table_z + 0.015 m` 모델 기반 추정
  - `yaw`: 평균 오차 `±5 deg` 이내
- 서비스 응답 기준:
  - 최신 유효 결과만 반환
  - stale 데이터는 motion에서 사용하지 않도록 age/time threshold를 둔다

## 설계 원칙
- perception의 실시간 경로는 edge에 남긴다.
- 딥러닝은 `검출과 keypoint 추정`에 사용하고, 최종 좌표 계산은 `기하학 + TF`로 수행한다.
- `x,y,z,yaw`를 네트워크가 직접 회귀하지 않는다.
- 로봇 motion이 참조할 결과는 반드시 `camera frame`이 아니라 `robot world frame` 기준으로 반환한다.
- 디버그 이미지는 실제 edge를 그리는 방식보다, 추정된 pose를 기준으로 `3D cuboid wireframe`을 재투영하는 방식으로 생성한다.

## 추천 아키텍처

```text
/camera/image_raw
/camera/camera_info
        |
        v
[detector_node]
  - DL 추론
  - box/color/top-face corners 추정
        |
        v
[pose_estimator_node]
  - CameraInfo 사용
  - table plane + TF + geometry
  - x/y/z/yaw 계산
        |
        v
[tracker_node]
  - smoothing
  - stale 판정
  - confidence 보정
        |
        +--> /omx/perception/blocks
        +--> /omx/perception/debug_image
        |
        v
[get_block_poses_server]
  - 최신 유효 결과 캐시 반환
```

## 패키지 구조 제안

초기 구현은 Python 기반 `ament_python`으로 진행한다. 딥러닝 추론, OpenCV 후처리, 데이터셋 유틸리티까지 한 패키지 안에서 빠르게 반복하기 쉽다.

```text
src/omx_perception/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── omx_perception
├── launch/
│   ├── perception.launch.py
│   ├── perception_debug.launch.py
│   └── perception_replay.launch.py
├── config/
│   ├── perception.yaml
│   ├── colors.yaml
│   ├── camera_extrinsics.yaml
│   └── tracker.yaml
├── models/
│   ├── README.md
│   └── block_detector.onnx
├── rviz/
│   └── perception.rviz
├── scripts/
│   ├── export_onnx.py
│   ├── replay_bag.sh
│   └── collect_dataset.py
├── test/
│   ├── test_geometry.py
│   ├── test_color_logic.py
│   └── test_pose_filter.py
└── omx_perception/
    ├── __init__.py
    ├── detector_node.py
    ├── pose_estimator_node.py
    ├── tracker_node.py
    ├── get_block_poses_server.py
    ├── debug_overlay.py
    ├── camera_model.py
    ├── table_plane.py
    ├── geometry.py
    ├── color_classifier.py
    ├── confidence.py
    ├── filtering.py
    ├── ros_conversions.py
    └── interfaces.py
```

## 파일별 책임

### launch/
- `perception.launch.py`
  - 실기기 기본 실행
  - detector, pose_estimator, tracker, service server 실행
- `perception_debug.launch.py`
  - debug image, additional logs, rviz 연동 포함
- `perception_replay.launch.py`
  - rosbag 기반 오프라인 검증용

### config/
- `perception.yaml`
  - 토픽 이름, frame id, stale threshold, target rate, debug enable
- `colors.yaml`
  - 색상 판별 fallback 규칙
  - red/green/blue HSV 범위나 RGB 통계 기준
- `camera_extrinsics.yaml`
  - hand-eye 보정 결과를 저장하는 임시/최종 설정
- `tracker.yaml`
  - smoothing 계수, confidence 하한, reject 기준

### models/
- 학습 완료 모델을 `ONNX`로 저장
- 초기에는 개발 PC에서 학습하고, 최종 추론은 edge용 경량 모델로 별도 관리

### omx_perception/
- `detector_node.py`
  - RGB 이미지 수신
  - DL 모델 추론
  - 각 블록의 bbox, score, corners, raw class 출력
- `pose_estimator_node.py`
  - CameraInfo와 TF를 사용해 `world` 기준 `PoseStamped` 계산
  - `z`는 table plane과 블록 크기 기반으로 결정
  - `yaw`는 top-face corner 정렬 결과에서 계산
- `tracker_node.py`
  - 프레임 간 object association
  - smoothing
  - stale 제거
  - confidence 보정
- `get_block_poses_server.py`
  - 최신 유효 블록 목록 캐시
  - `GetBlockPoses` 서비스 제공
- `debug_overlay.py`
  - 중심점, color, confidence, yaw 텍스트
  - 추정 pose 기반 3D wireframe 재투영
- `camera_model.py`
  - CameraInfo 파싱
  - undistort/project/back-project 유틸
- `table_plane.py`
  - 테이블 평면 모델
  - ray-plane intersection
- `geometry.py`
  - keypoint ordering, yaw 계산, pose fitting
- `color_classifier.py`
  - 모델 출력 보강용 색상 판정
- `confidence.py`
  - detector score, reprojection error, occlusion ratio 기반 confidence 산출
- `filtering.py`
  - One Euro Filter 또는 Kalman filter
- `ros_conversions.py`
  - 내부 구조체 <-> ROS msg 변환

## ROS 인터페이스 계획

### 구독
- `/camera/image_raw`
- `/camera/camera_info`

### 발행
- `/omx/perception/blocks`
  - 내부 디버그/모니터링용
- `/omx/perception/debug_image`
  - 디버그 오버레이 이미지

### 서비스
- `/omx/get_block_poses`
  - `omx_interfaces/srv/GetBlockPoses`

### frame 기준
- 최종 응답 `PoseStamped.header.frame_id`는 `world`를 기본값으로 사용
- 필요 시 파라미터로 `world`를 허용할 수 있으나 초기 구현은 `world` 하나로 고정한다

## 기술 선택

### 1. detector
초기 구현 추천 순서:
1. 경량 keypoint detector
2. instance segmentation 보강
3. occlusion 대응 tracker 고도화

후보:
- `YOLOv8n-pose` 또는 동급 경량 keypoint 모델
- `YOLOv8n-seg` 또는 동급 경량 segmentation 모델

권장 방향:
- 최종 목표는 `color + top-face 4 corners`를 직접 예측하는 모델
- 다만 첫 번째 실행 단계에서는 개발 속도를 위해 `segmentation + geometry` 조합도 허용한다

### 2. pose estimation
- 입력: `4 top-face corners`, `CameraInfo`, `TF(camera -> world)`, `table plane`
- 출력: `center x/y/z`, `yaw`
- 가정:
  - 블록은 upright
  - `roll`, `pitch`는 무시하거나 0으로 고정
  - 중심 높이는 `table_z + 0.015`

### 3. tracking
- 초기에는 nearest-neighbor association으로 충분
- confidence가 낮거나 detection age가 threshold를 넘으면 stale 처리

## 단계별 구현 계획

한 번에 끝내지 않는다. 아래 단계는 앞 단계가 검증돼야 다음 단계로 넘어간다.

### Step 0. 좌표계와 캘리브레이션 기준 고정 ✅ DONE

목표:
- `CameraInfo`와 `TF(default_cam -> world)`를 확인
- 테이블 평면 기준을 정한다
- 최종 반환 frame을 `world`로 확정한다

산출물:
- `calibration/wrist_cam.yaml` — fx=fy=900, cx=640, cy=360 (1280x720 추정값, 왜곡 없음)

완료 기준:
- ✅ 최종 반환 frame: `world`
- ✅ TF 경로: `default_cam → world` (tf2_echo로 확인 가능)
- ✅ table_z = 0 (world 원점 기준 가정)
- ✅ CameraInfo intrinsics 파일 생성 완료
- ✅ 픽셀 → ray 계산 가능 조건 충족
- 정밀 캘리브레이션은 Step 4 오차 개선 시점에 수행

### Step 1. 비-DL 프로토타입으로 파이프라인 뼈대 먼저 구축 ✅ DONE

목표:
- ROS2 노드 구조와 서비스 흐름을 먼저 완성
- 추후 detector만 교체 가능한 구조 확보

산출물:
- `detector_node.py` — HSV + contour 기반 검출, TF ray-plane intersection으로 world 좌표 계산, debug_image 발행
- `tracker_node.py` — 10Hz pass-through, staleness + confidence 필터
- `get_block_poses_server.py` — stale 판정 후 `GetBlockPoses` 서비스 제공
- `launch/perception.launch.py`, `config/perception.yaml`

완료 기준:
- ✅ `ros2 service call /omx/get_block_poses` 동작 확인
- ✅ world 기준 BlockPose[] 반환 (frame_id=world, z=0.015)
- ✅ green/blue color 구분 동작
- ✅ stamp, confidence 모두 채워짐
- 오탐은 Step 4(HSV 튜닝)에서 개선

### Step 2. 데이터셋 수집 및 라벨 기준 확정
목표:
- 실제 환경 기준의 학습 데이터셋을 만든다

라벨 포맷 확정:
- 모델: YOLOv8-pose
- classes: red(0), green(1), blue(2)
- 각 인스턴스: bbox + top-face 4 keypoints (tl, tr, br, bl) + visibility
- 포맷: `datasets/block_detection/data.yaml` 기준

작업:
- `scripts/collect_dataset.py`로 이미지 수집 (SPACE=저장, Q=종료)
- 수집 장면: 단일 블록, 복수 블록, 부분 가림, 다양한 각도/조명
- 라벨링: Roboflow 또는 LabelImg (YOLO pose 포맷)
- train 80% / val 20% split

산출물:
- `datasets/block_detection/images/train|val/`
- `datasets/block_detection/labels/train|val/`
- `datasets/block_detection/data.yaml`

완료 기준:
- 최소 300장 이상 real image 확보
- edge case(부분 가림, 붙어있는 블록)가 val set에 포함된다

### Step 3. DL detector 1차 적용
목표:
- 임시 contour detector를 경량 딥러닝 detector로 교체

작업:
- small model 학습
- ONNX export
- `detector_node.py`에 ONNX runtime 또는 TensorRT 경로 연동
- segmentation 또는 keypoint 출력 파싱

산출물:
- `models/block_detector.onnx`
- detector inference 로그

완료 기준:
- 고정 조명 환경에서 `red/green/blue` 분리가 안정적이다
- 겹침이 없는 장면에서 center/yaw 추정이 contour 단계보다 개선된다

### Step 4. pose estimation 정밀화
목표:
- `x,y,z,yaw` 정확도를 실제 grasp 가능한 수준으로 끌어올린다

작업:
- top-face corner ordering 안정화
- ray-plane intersection 정밀화
- yaw 계산 규칙 고정
- reprojection error 기반 reject 추가

산출물:
- `geometry.py`, `confidence.py` 보강
- 정량 평가 로그

완료 기준:
- `x,y ±5 mm`, `yaw ±5 deg` 수준 달성
- pick 전 pre-grasp offset 계산에 사용 가능

### Step 5. tracker / stale / failure handling 추가
목표:
- perception 결과를 motion이 안전하게 사용할 수 있게 만든다

작업:
- object tracking
- age threshold
- confidence threshold
- 낮은 confidence 또는 stale 시 결과 제외

산출물:
- stale 판단 규칙
- tracker 파라미터

완료 기준:
- motion이 오래된 결과를 참조하지 않는다
- occlusion 이후 재검출 시 pose jump가 과도하지 않다

### Step 6. 디버그/평가/포트폴리오 품질 보강
목표:
- 면접과 데모에서 설명 가능한 시각화와 지표를 확보한다

작업:
- debug image에 3D wireframe, center, yaw, confidence 표시
- latency 측정
- success rate 측정
- replay 기반 재현 실험

산출물:
- rosbag replay 검증 루틴
- 샘플 디버그 이미지
- 성능 표

완료 기준:
- 반복 실험 결과를 수치와 영상으로 제시 가능
- 다음 단계인 `omx_skill_executor`가 이 결과를 안정적으로 사용 가능

## 단계별 우선순위 요약

```text
Step 0  캘리브레이션 / frame 기준 확정
Step 1  ROS 파이프라인 뼈대 + 임시 detector
Step 2  데이터셋 수집 / 라벨 기준 확정
Step 3  DL detector 1차 적용
Step 4  x/y/z/yaw 정밀화
Step 5  tracker / stale / confidence 안전화
Step 6  디버그 / 평가 / 포트폴리오 보강
```

## 지금 바로 시작할 최소 범위

현재 세션 기준으로 가장 먼저 닫아야 할 범위는 아래다.

1. `omx_perception` 패키지 생성
2. `perception.launch.py`와 기본 파라미터 파일 생성
3. 임시 contour detector 기반 `GetBlockPoses` 서비스 뼈대 구현
4. `world` 기준 pose 반환 경로 검증
5. debug image에 center와 color 표시

즉, 첫 번째 구현 목표는 `최종 detector 품질`이 아니라 `service contract + 좌표 변환 + 디버그 가시화`다.

## 다음 단계로 넘기는 조건

Step 1에서 Step 2로 넘어가는 조건:
- ROS 노드 구조가 고정됐다
- `GetBlockPoses` 서비스가 실제 값을 반환한다
- world 기준 pose 계산 경로가 검증됐다

Step 3 이후 `omx_skill_executor`와 연결할 조건:
- 최소 한 색상 블록에 대해 pick 가능한 정확도가 나온다
- stale 결과를 걸러낼 수 있다
- debug image와 로그로 실패 이유를 설명할 수 있다

## 보류 항목
- multi-camera
- full 6DoF pose estimation
- remote inference
- LLM 기반 scene understanding
- stacking/align 전용 perception 로직

위 항목은 현재 단계에서 넣지 않는다.
