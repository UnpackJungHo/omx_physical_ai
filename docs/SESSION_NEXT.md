# OMX 프로젝트 — 다음 세션 이어하기

<!-- AUTO_PHASE_SUMMARY_START -->
## 단계 상태
- `A` 완료 — 레퍼런스 코드 읽기
- `B` 완료 — 로봇 모델 표시
- `C` 완료 — ros2_control 붙이기 (mock 모드)
- `D` 완료 — MoveIt2 붙이기
- `E` 롤백 — 통합 launch 파일 (의미 없다고 판단, 삭제)

## 현재 작업 단계
- `omx_perception` Step 0~3 진행 중 (4단계)
- Step 0 ✅, Step 1 ✅, Step 2 ✅, Step 3 🔧 (모델 성능 이슈)
<!-- AUTO_PHASE_SUMMARY_END -->

---

## 내일 바로 할 일 (최우선)

### 문제 상황
YOLOv8 학습은 완료(mAP50=0.82)됐지만 실제 추론 성능이 나쁨.
**원인: 조명이 너무 밝아서 빨강→핑크, 파랑→청록으로 보임 → 라벨도 틀렸을 가능성 높음.**

### Step 1 — 조명(노출) 확인
usb_cam 실행 스크립트가 이미 `exposure:=200`으로 설정돼 있음:
```bash
~/omx_ws/run_usb_cam.sh
```
rqt_image_view로 색상이 실제 빨강/초록/파랑으로 제대로 보이는지 확인.
```bash
rqt_image_view /image/raw
```
- 색상이 제대로 보이면 → Step 2로
- 아직 이상하면 `exposure` 값 더 낮춰서 조정 (100, 50 등)

### Step 2 — 데이터 재수집 또는 재라벨링
**A안 (권장): 노출 고친 후 데이터 재수집**
```bash
# 기존 데이터 백업
mv ~/omx_ws/datasets/block_detection/images/train ~/omx_ws/datasets/block_detection/images/train_old

# 새 데이터 수집
python3 ~/omx_ws/src/omx_perception/scripts/collect_dataset.py
```
300장 재수집 → 자동 라벨 생성 → 검수 → 재학습

**B안: 기존 데이터 라벨 재검수**
```bash
python3 ~/omx_ws/src/omx_perception/scripts/label_review.py
```
틀린 색상 라벨 수정 (핑크→red, 청록→blue 확인)

### Step 3 — 재학습
```bash
conda activate base
python3 ~/omx_ws/src/omx_perception/scripts/train_detector.py
```
`train_v2` 이름으로 저장됨. augmentation 강화 설정 포함.

학습 완료 후 ONNX export:
```bash
conda activate base
python3 -c "
from ultralytics import YOLO
model = YOLO('/home/kjhz/omx_ws/datasets/block_detection/train_v2/weights/best.pt')
model.export(format='onnx', imgsz=640, simplify=True)
"
cp ~/omx_ws/datasets/block_detection/train_v2/weights/best.onnx \
   ~/omx_ws/src/omx_perception/models/block_detector.onnx
```

빌드 후 테스트:
```bash
colcon build --packages-select omx_perception
source ~/omx_ws/install/setup.bash
ros2 launch omx_perception perception.launch.py
ros2 service call /omx/get_block_poses omx_interfaces/srv/GetBlockPoses "{color: ''}"
rqt_image_view /omx/perception/debug_image
```

---

## 오늘 한 일 요약 (2026-04-19)

### omx_perception Step 0 완료
- `calibration/wrist_cam.yaml` 생성 (fx=fy=900, cx=640, cy=360)
- 최종 반환 frame: `world`, table_z=0 확정
- usb_cam 실행 스크립트: `~/omx_ws/run_usb_cam.sh`

### omx_perception Step 1 완료
- `detector_node.py` (HSV+contour → ray-plane intersection)
- `tracker_node.py` (10Hz, staleness+confidence 필터)
- `get_block_poses_server.py` (GetBlockPoses 서비스)
- `launch/perception.launch.py`, `config/perception.yaml`
- `/omx/get_block_poses` 서비스 동작 확인

### omx_perception Step 2 완료
- 데이터 수집: 298장 (`datasets/block_detection/images/train/`)
- 자동 라벨링: `scripts/auto_label.py`
- 라벨 검수 도구: `scripts/label_review.py`
- train/val split: train 239장 / val 59장

### omx_perception Step 3 진행 중
- YOLOv8n 학습 완료 (mAP50=0.82, `datasets/block_detection/train_v1/`)
- ONNX export: `models/block_detector.onnx`
- `detector_node.py` ONNX 기반으로 교체 완료
- **이슈: 조명 과다노출로 색상 오인식 → 재수집 또는 재라벨링 필요**

---

## 주요 파일 위치

| 파일 | 설명 |
|------|------|
| `~/omx_ws/run_usb_cam.sh` | usb_cam 실행 (exposure=200) |
| `src/omx_perception/models/block_detector.onnx` | 현재 모델 |
| `datasets/block_detection/train_v1/` | 1차 학습 결과 |
| `src/omx_perception/scripts/collect_dataset.py` | 이미지 수집 |
| `src/omx_perception/scripts/auto_label.py` | 자동 라벨 생성 |
| `src/omx_perception/scripts/label_review.py` | 라벨 검수/수정 |
| `src/omx_perception/scripts/train_detector.py` | 학습 스크립트 (train_v2 설정) |
| `docs/OMX_PERCEPTION_PLAN.md` | Step별 구현 계획 |

---

## 전체 개발 순서 (현재 위치)
```
bringup ✅ → motion API ✅ → perception 🔧 → skill 1개 → recovery → edge/server 분리 → QoS
```
