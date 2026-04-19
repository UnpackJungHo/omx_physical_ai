# OMX Perception Step 0 Calibration Memo

## 목적
- `OMX_PERCEPTION_PLAN.md`의 Step 0을 실제 작업 기준으로 고정한다.
- `omx_perception`이 이후 단계에서 반환할 최종 pose frame을 실제 모델 기준 프레임으로 통일한다.
- detector 구현 전에 `CameraInfo`, `camera -> world TF`, `table plane` 기준을 먼저 확정한다.

## Step 0에서 확정하는 것
- 최종 반환 frame: `world`
- 카메라 광학 중심 원점: `default_cam`
- 로봇 루트 링크: `link0`
- 테이블 평면 기준 frame: `world`
- 블록 중심 높이 계산 기준: `table_z_m + 0.015`

## 현재 모델 기준 프레임
- URDF 루트에 `world` 링크가 있고 `world -> link0`는 fixed joint다.
- SRDF에서 arm group의 chain base는 `link0`다.
- 따라서 perception 결과의 외부 기준 프레임은 `world`로 두는 것이 자연스럽고, 로봇 기구학 루트 링크는 `link0`로 이해하면 된다.
- `world -> default_cam`은 wrist-mounted 카메라 특성상 동적 TF다. 로봇 자세가 바뀌면 값도 바뀐다.
- 따라서 `world -> default_cam` 수치를 YAML에 저장하지 않는다. 런타임에 TF로 조회해야 한다.

## 산출물
- [camera_extrinsics.yaml](/home/kjhz/omx_ws/src/omx_perception/config/camera_extrinsics.yaml)
- 이 문서
- `pixel_ray_tool` 보조 유틸리티

## 측정 절차
1. 카메라 intrinsic calibration을 수행하고 `/camera/camera_info`가 실제 보정값을 내보내는지 확인한다.
2. `world -> default_cam` 변환이 TF에서 정상적으로 조회되는지 확인한다.
3. 테이블 위 기준점 3개 이상을 `world` 기준으로 측정한다.
4. `camera_extrinsics.yaml`에는 frame 이름과 테이블 평면 기준만 기록한다.
5. 기준 픽셀 3개 이상을 클릭해 ray와 table intersection이 대략 맞는지 확인한다.

## 권장 측정 기준
- `world`는 외부 기준 프레임, `link0`는 로봇 루트 링크로 구분한다.
- `default_cam`은 카메라 optical frame과 혼동되지 않게 실제 TF 이름 하나로 고정한다.
- 테이블 평면 normal은 초기에는 `[0, 0, 1]`로 두고, 장착 오차가 크면 실측 normal로 갱신한다.
- 검증용 기준점은 로봇이 접근 가능한 작업면 중앙과 좌우 끝점을 포함한다.

## 빠른 검증 방법
카메라 calibration YAML과 `camera_extrinsics.yaml`이 준비되면 아래처럼 특정 시점의 TF 스냅샷을 넣어 한 픽셀의 world-frame ray를 계산할 수 있다.

```bash
PYTHONPATH=src/omx_perception \
python3 -m omx_perception.pixel_ray_tool \
  --camera-info-yaml /path/to/camera_calibration.yaml \
  --config-yaml /home/kjhz/omx_ws/src/omx_perception/config/camera_extrinsics.yaml \
  --translation 0.036 0.006 0.310 \
  --quaternion 0.733 -0.646 0.140 -0.161 \
  --pixel-u 320 \
  --pixel-v 240
```

출력 확인 기준:
- `ray_origin_reference_m`가 카메라 위치와 대략 맞아야 한다.
- `ray_direction_reference`가 카메라 시야 방향과 일관돼야 한다.
- `table_intersection_reference_m`가 실제 클릭 지점 근처로 떨어져야 한다.

## Step 0 완료 판단
- `/camera/camera_info`의 intrinsic이 신뢰 가능하다.
- `world -> default_cam` 변환이 TF에서 조회된다.
- `table_z_m`와 평면 기준점이 문서화돼 있다.
- 픽셀 클릭 하나를 world-frame ray와 table intersection으로 변환해 sanity check가 가능하다.

## 아직 하지 않는 것
- HSV contour detector
- `GetBlockPoses` 서비스
- tracker / stale handling
- debug image overlay

위 항목은 Step 1 이후 범위다.
