# Box Color Detection 설계

- 작성일: 2026-05-04
- 대상 패키지: `omx_perception`, `omx_interfaces`
- 트리거: `pick_detected_server`가 여전히 `red/green/blue` 라벨로 블록을 선택하지만, 현재 `box_cup_world_pose_node`는 `BlockPose.color = "box"`만 채워 색상 기반 선택이 동작하지 않음. YOLO Pose가 제공하는 박스 윗면 4 keypoints(`top_4_pose`) 안쪽 픽셀로 색상을 추출해 다시 살린다.

## 1. 목표 / 비목표

**목표**
- 박스 detection마다 `red / green / blue / unknown` 중 하나를 산출하고, `BlockPose.color`에 그대로 채워 `pick_detected_server`가 색상으로 박스를 선택할 수 있게 한다.
- 조명 변화·그림자·specular 하이라이트·페인트 비균일성·나무결 바닥에 강건한 분류 로직을 만든다.
- 카메라/조명이 바뀌었을 때 코드 수정 없이 reference만 갱신할 수 있도록 yaml-driven reference + 자동 캘리브레이션 도구를 제공한다.
- Cup detection의 기존 동작은 변경하지 않는다 (`BlockPose.color = "cup"` 유지).

**비목표**
- 컵의 색상 인식.
- 박스 4면 전체 색상 추론. 윗면(`top_4_pose`) 안쪽 픽셀만 사용한다.
- 분류 결과를 picking 외 다른 정책(예: 정렬, 우선순위)에 사용하는 로직.
- 새 색상(노랑/검정 등) 추가. yaml에서 기술적으로 가능하지만 이번 단계에서는 R/G/B 3종만 등록한다.

## 2. 영향 범위

- `omx_interfaces/msg/KeypointDetection.msg`: 필드 2개 추가 (`color`, `color_confidence`). 빈 문자열/0.0 default라 기존 소비자 호환.
- `omx_interfaces/msg/BlockPose.msg`: 필드 변경 없음. 코멘트만 갱신.
- `omx_perception/box_cup_pose_node.py`: ROI 색상 분류 로직 추가, 디버그 이미지에 색상 라벨 overlay.
- `omx_perception/box_cup_world_pose_node.py`: `det.color`를 `BlockPose.color`로 그대로 전달 (Cup이면 `"cup"`).
- `omx_perception/launch/perception.launch.py`: `box_color_reference_path` 파라미터 추가.
- `omx_perception/setup.py`: `box_color_calibrate` console_script 등록.
- `omx_skill_executor`: 변경 없음. 기존 `VALID_COLORS = {red, green, blue}` 비교가 다시 의미를 가지게 됨.

## 3. 아키텍처

```
[camera] → /image/raw
              │
              ▼
   box_cup_pose_node                           ← ROI 색상 분류 추가
   - YOLO keypoints (기존)
   - top_4 polygon → inset → illumination filter
     → LAB a*/b* nearest reference (color_classifier)
   - service: /perception/get_box_cup_keypoints
        → KeypointDetection[] (+ color, color_confidence)
   - publishes annotated image with color label overlay
              │
              ▼
   box_cup_world_pose_node                     ← 최소 변경
   - 기존 PnP + TF
   - BlockPose.color = det.color (Cup이면 "cup")
   - service: /perception/get_box_cup_world_poses → BlockPose[]

   box_color_calibrate (새 CLI 노드, 일회성)
   - sequential 색상 등록
   - 한 색상당 N프레임 누적 → median(LAB a*, b*) → yaml 저장
```

분류 로직은 공유 모듈 `omx_perception/color_classifier.py`로 추출해 pose_node와 calibrate CLI가 동일한 함수를 호출한다.

## 4. 색상 분류 알고리즘

### 4.1 입력
- BGR 이미지 1장
- 박스 윗면 4 keypoints (이미지 좌표, [TL, TR, BR, BL] 순서)
- reference table (yaml에서 로드된 `[(name, a*, b*), ...]`)

### 4.2 단계
1. `polygon_inset(pts, ratio)` — 무게중심 기준으로 polygon을 `ratio`만큼 축소 (default 0.7).
2. inset polygon을 mask로 채운 뒤 ROI 픽셀 추출.
3. ROI를 LAB로 변환. illumination filter로 mask 갱신:
   - `L < L_low_pct` (예: 하위 10%) 픽셀 제거 → 진한 그림자.
   - `L > L_high_pct` (예: 상위 5%) 픽셀 제거 → specular 하이라이트.
   - HSV.S `< saturation_min` (예: 30/255) 픽셀 제거 → 회색/흰/검 (wood grain, 손, 매트).
4. 유효 픽셀 수 `< min_valid_pixels`이면 `("unknown", 0.0)` 반환.
5. 남은 픽셀 LAB의 `(median(a*), median(b*))` 계산. mean 대신 median으로 잔여 outlier 영향 추가 억제.
6. 각 reference에 대해 a*/b* 평면 거리 `dist = sqrt((a-ref.a)^2 + (b-ref.b)^2)` 계산하여 최소값 선택.
7. `confidence = clamp(1 - dist / distance_threshold, 0.0, 1.0)`.
8. `dist > distance_threshold`면 `("unknown", 0.0)`, 아니면 `(ref.name, confidence)`.

### 4.3 설계 근거
- LAB의 a*/b* 평면만 사용 — L(밝기) 변화에 강건. 그림자·하이라이트의 주된 영향은 L에 실리므로 a/b 평면 거리는 거의 보존된다.
- HSV.S 하한 + L 상/하위 percentile 컷 — wood grain·손·매트·반사·짙은 그림자를 통계적으로 제거.
- inset 0.7 — 모서리의 페인트 닳음, 그림자 가장자리, edge에 걸친 specular hot-spot 회피.
- median 통계 — 남은 outlier에 추가 면역.
- distance threshold 기반 unknown 폴백 — 등록되지 않은 색이나 모호한 ROI에서 잘못된 라벨을 강제하지 않음.

### 4.4 reference yaml

`config/box_color_reference.yaml` (초기값은 dataset 이미지에서 한 번 추출해 채워둠, 이후 캘리브레이션 도구로 갱신).
```yaml
inset_ratio: 0.7
saturation_min: 30
luminance_low_percentile: 10
luminance_high_percentile: 95
min_valid_pixels: 60
distance_threshold: 18.0
references:
  - name: red
    lab_ab: [42.5, 25.1]
  - name: green
    lab_ab: [-30.0, 18.0]
  - name: blue
    lab_ab: [10.0, -45.0]
```

위 yaml 예시의 a*/b* 수치는 형식을 보여주기 위한 예시값이다. **구현 첫 단계로 dataset 이미지에서 색상별 ROI 픽셀의 LAB median(a*, b*)를 추출해 위 값들을 실제 reference 값으로 교체한다.** 그 이후 환경이 바뀌면 캘리브레이션 도구로 갱신한다.

`distance_threshold`는 OpenCV LAB(8-bit, 0~255 범위)에서의 a*/b* 평면 유클리드 거리 단위이다.

## 5. 캘리브레이션 워크플로우

```
$ ros2 run omx_perception box_color_calibrate \
      --colors red green blue \
      --frames-per-color 30 \
      --output src/omx_perception/config/box_color_reference.yaml
```

진행:
1. CLI는 `/image/raw` 와 `/perception/get_box_cup_keypoints` 서비스를 사용 (perception 노드가 떠 있어야 함).
2. 색상 하나씩 prompt: `Place RED box in view, then press [Enter]…`
3. N(=30) 프레임 동안 keypoint service 호출:
   - 박스가 정확히 1개 검출된 프레임만 사용. ≠1이면 그 프레임 skip + warn (카운트 미증가).
   - inset polygon → illumination filter 적용 → 유효 픽셀의 LAB 누적.
4. 색상별 누적 픽셀 전체의 `(median(a*), median(b*))` → 해당 색상의 reference로 저장.
5. 모든 색상 완료 후 yaml 저장. 기존 yaml은 `<output>.bak`로 백업.

운영자 가이드:
- 카메라 시야에서 박스 외 다른 빨/초/파 객체(예: 화장품)가 보이지 않게 한다.
- 박스를 시야 중앙·가장자리·다양한 각도/거리에서 N프레임 동안 천천히 움직이게 한다 → reference 픽셀의 조명 다양성을 자연스럽게 누적.

## 6. 인터페이스 변경

### 6.1 `omx_interfaces/msg/KeypointDetection.msg` (추가)
```
+ string color              # "red"|"green"|"blue"|"unknown"|"" (Cup이면 "")
+ float32 color_confidence  # 0.0~1.0; color="" 또는 "unknown"이면 0.0
```

### 6.2 `omx_interfaces/msg/BlockPose.msg` (코멘트만 갱신)
```
- string color  # "box" | "cup"  (legacy: was "red" | "blue" | "green")
+ string color  # "red"|"green"|"blue"|"unknown"|"cup"
```

### 6.3 `box_cup_world_pose_node.py`
`_detection_to_block_pose`에서 `BlockPose.color` 결정 로직만 변경:
- `det.class_id == CLASS_BOX` → `block.color = det.color or "unknown"`
- `det.class_id == CLASS_CUP` → `block.color = "cup"`
- 그 외 → 기존대로 drop.

## 7. 파일 레이아웃

```
src/omx_perception/
├── omx_perception/
│   ├── color_classifier.py        ← 새 파일 (공유 모듈)
│   ├── box_cup_pose_node.py       ← 수정: 색상 분류 사용 + annotated 라벨
│   ├── box_cup_world_pose_node.py ← 수정: det.color → BlockPose.color
│   └── box_color_calibrate.py     ← 새 파일 (CLI 노드)
├── config/
│   ├── camera_params.yaml         (기존)
│   ├── camera_intrinsics.yaml     (기존)
│   └── box_color_reference.yaml   ← 새 파일
├── launch/
│   └── perception.launch.py       ← 수정: box_color_reference_path 파라미터
└── setup.py                        ← 수정: box_color_calibrate console_script
```

## 8. 실패 모드와 처리

| 상황 | 처리 |
|---|---|
| keypoint confidence < min | 기존대로 detection drop (변화 없음) |
| ROI inset 후 유효 픽셀 < `min_valid_pixels` | `color="unknown"`, `color_confidence=0.0` |
| LAB a*/b* 거리 > `distance_threshold` | `color="unknown"`, `color_confidence=0.0` |
| reference yaml 없거나 로드 실패 | 색상 분류 비활성 (`color=""`), `warn` 로그. keypoint/PnP 동작은 정상 유지 |
| 캘리브레이션 중 박스가 ≠1개 검출 | 프레임 skip, 카운트 미증가, `warn` 로그 |
| Cup detection | 색상 분류 skip, `color=""`, `color_confidence=0.0` |
| polygon이 이미지 밖으로 일부 벗어남 | 이미지 크기로 clip 후 진행. 유효 픽셀 부족하면 `unknown` 폴백 |

## 9. 검증 계획

- **단위 테스트**: `color_classifier.classify(image, polygon, refs) -> (name, confidence)`를 dataset 이미지 N장 (`red/green/blue` 각 ≥10장)에 대해 라벨 파일과 비교. 정확도 목표 ≥ 95%. 빠른 spot-check만, 정식 pytest 추가는 선택사항.
- **통합 빌드/실행**:
  ```
  colcon build --symlink-install --packages-select omx_perception omx_interfaces
  source install/setup.bash
  ros2 launch omx_perception perception.launch.py
  ros2 service call /perception/get_box_cup_world_poses omx_interfaces/srv/GetBlockPoses "{color: ''}"
  ```
  응답의 `blocks[*].color`가 `red/green/blue/unknown/cup` 중 하나로 채워지는지 확인.
- **회귀**: `pick_detected_server` 실행 후 CLI prompt가 `Detected colors: ['blue', 'green', 'red']` 형태로 표시되는지 (실제 환경에서 박스 3개 모두 시야 안에 둘 때).
- **캘리브레이션 도구**: 새 환경에서 한 번 실행하고 reference yaml이 정상 갱신되는지, 갱신 후 분류가 회복되는지 확인.

## 10. QoS / 동시성

- `box_cup_pose_node`의 callback group/QoS는 변경하지 않는다. 색상 분류는 기존 image callback 안에서 inline으로 수행 (per-frame 추가 비용은 ROI inset + LAB 변환 + median 계산으로 작음).
- `box_color_calibrate`는 일회성 CLI이므로 `SingleThreadedExecutor` + 짧은 timeout으로 service 호출. 실패 시 즉시 종료, yaml 미수정.
- `box_cup_world_pose_node`는 동시성 모델 변경 없음.
