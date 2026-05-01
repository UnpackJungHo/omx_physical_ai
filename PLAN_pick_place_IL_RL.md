# OMX-F: 박스 → 종이컵 정밀 Pick & Place 계획

> 자연어 → OMX 매니퓰레이터 제어 시스템의 한 task로서,
> 30mm × 30mm 박스를 종이컵 안에 정확히 넣는 동작을 IL + RL 기반으로 구현.

---

## 0. 전제 (이미 구현됨)

- [x] **박스 위치 좌표 인식** (perception 파이프라인, Top4 pose service)
- [x] **`move_to_pose` 액션** (좌표 → MoveIt → IK → 실행)
- [x] **planning scene service** (workspace guard, 충돌 회피)
- [x] **`setJointValueTarget(pose, link) + position_only_ik` 패턴** (5-DOF arm 대응)

이 자산을 최대한 재활용한다.

---

## 1. Step 1 — Grasp Detection (그리퍼가 박스를 잡았는가?)

### 목표
Dynamixel current 기반 grasp 판단 노드/서비스 구현. IL 데이터 라벨링과 RL 보상 신호의 전제.

### 방법
- **Operating Mode**: Current-based Position Control (mode 5)
- **Goal Current**: ~100 mA (~37 unit, 30mm 박스 + 종이컵 안 손상 안 가는 수준)
- **판단 조건 (3-신호 AND, 100~200ms 안정화)**:
  - `|present_current| > CURRENT_THRESH` (예: 70~120 mA)
  - `|goal_pos − present_pos| > POS_ERR_THRESH` (예: 30~50 tick)
  - `|present_velocity| < VEL_THRESH` (정지 상태)

### 산출물
- ROS2 노드: `grasp_detector`
- 토픽: `/gripper/is_grasping` (Bool, latched)
- 서비스: `/gripper/check_grasp` (요청 시 즉시 평가)
- (선택) 토픽: `/gripper/grasp_force_estimate` (current → 추정 힘)

### 검증
- 빈 그리퍼 닫기 → False
- 30mm 박스 잡기 → True
- 박스 잡고 lift 후 재확인 → True 유지 (미끄러짐 시 False)

### TODO
- [ ] dynamixel_hardware_interface에서 effort state 노출 확인
- [ ] threshold 튜닝 (실물 박스로 10회 측정 후 결정)
- [ ] grasp_detector 노드 구현 + launch 통합
- [ ] `/gripper/check_grasp` 서비스 인터페이스 정의 (interfaces 패키지)

---

## 2. Step 2 — Classical Pipeline: Approach → Grasp → Lift → Hover

### 범위 재정의 (중요)
원안의 "좌표 이동 + 잡기 + 컵 위 이동"은 **classical IK + move_to_pose 조합으로 충분히 풀린다**.
IL은 이 영역에 쓰지 않고, **마지막 정밀 release 단계 (Step 3)에 집중**한다.

### Sub-steps
1. `box_pose` 수신 → 박스 위 5cm pre-grasp pose 계산
2. `move_to_pose(pre_grasp)`
3. `move_to_pose(grasp_pose)` (박스 중심)
4. 그리퍼 닫기 → `check_grasp` 호출
5. 실패 시 retry / 사용자에게 보고
6. `move_to_pose(lift_pose)` (현재 위치 + Z 10cm)
7. `cup_pose` 수신 → 컵 위 5cm hover pose 계산
8. `move_to_pose(hover_pose)`

### 산출물
- ROS2 액션: `pick_and_hover` (입력: box_id/pose, cup_pose / 출력: 성공 여부, hover 도달 후 종료)
- 이 액션이 끝나면 → Step 3 IL/RL 정책으로 핸드오프

### TODO
- [ ] `pick_and_hover` 액션 인터페이스 정의
- [ ] retry 로직 (grasp 실패 시 박스 재인식)
- [ ] hover pose 계산 정책 결정 (컵 중심 위 / 박스 크기 고려)

---

## 3. Step 3 — LeRobot IL: Hover → 컵 안 정렬 → Release

### 목표
hover 자세에서 컵 안에 정확히 박스를 떨어뜨리는 짧은 horizon (5~8초) 정책 학습.

### Task scope
- 시작: hover pose (컵 위 ~5cm, 박스 잡고 있음)
- 종료: 박스가 컵 안에 떨어짐 (grasp_detector False + 시각 확인)

### 정책 후보
1. **Diffusion Policy** (LeRobot 기본, 안정적) — 1순위
2. **ACT** (action chunking, 짧은 horizon 효율적) — 비교 베이스라인
3. **π0 fine-tune** — VRAM 8GB로 빡셈, 후순위

### 데이터 사양

**Observation (timestep마다):**
| 키 | 형태 | 출처 |
|---|---|---|
| `observation.images.wrist` | RGB 240×320×3 | 손목 카메라 |
| `observation.images.top` | RGB 240×320×3 | 외부 카메라 |
| `observation.state` | float[6] | 5 joint + gripper pos |
| `observation.box_pose` | float[7] | perception (privileged input) |
| `observation.cup_pose` | float[3] | perception |
| `observation.gripper_current` | float | dynamixel |

**Action:**
- `action`: float[6] = delta joint position (5) + gripper command (1)

**Episode 메타:**
- `task`: "place the 30mm box into the paper cup"
- `fps`: 15
- `success`: bool (자동 라벨)

**수집 사양:**
- 에피소드 수: 시뮬 200~1000 + 실물 30~50
- 주파수: 15 Hz
- 길이: 5~8초 (75~120 step)
- 변동성: 박스 위치 randomize, 컵 위치 randomize, 조명 randomize (시뮬)

### 텔레옵 도구 결정 필요
- [ ] 옵션 A: SpaceMouse 구매 (~20만원)
- [ ] 옵션 B: 시뮬 자동 데모 생성 (Step 4와 시너지)
- [ ] 옵션 C: 키보드/게임패드 (품질 낮음, 임시용)

### 산출물
- LeRobotDataset (parquet + mp4): `datasets/box_to_cup_v1/`
- 학습된 정책: `models/diffusion_policy_box_to_cup.ckpt`
- 추론 노드: `lerobot_policy_runner` (LeRobot 정책 → ROS2 action)

### TODO
- [ ] LeRobot 설치 + OMX config 확인 (5-DOF 변형 호환성)
- [ ] ROS2 → LeRobotDataset writer 노드 (시간 동기화 포함)
- [ ] 텔레옵 도구 결정 + 셋업
- [ ] 시뮬 자동 데모 스크립트 (motion-planned)
- [ ] 학습 파이프라인 (`lerobot-train` config)
- [ ] 추론 노드 (LeRobot policy → ROS2 action 인터페이스)

---

## 4. Step 4 — MuJoCo 시뮬 환경 + Residual RL (HIL-SERL)

### 목표
1. MuJoCo에서 OMX-F + 박스 + 종이컵 환경 구축 (Step 3 데이터 생성용 + Step 4 RL용)
2. Step 3의 IL 정책 위에 residual RL 얹어 sub-cm 정밀도까지 끌어올림

### MuJoCo 환경 구성
- OMX-F URDF → MJCF 변환
- 30mm 박스 (rigid)
- 종이컵 (rigid 근사, deformable 미적용)
- 카메라: wrist, top (실물과 동일 위치)
- Domain randomization: 조명, 마찰, 박스/컵 위치, 카메라 노이즈

### Residual RL: HIL-SERL
- IL 정책 = base policy (frozen)
- RL 정책 = residual delta action (작게)
- Critic은 ground-truth pose 받음 (asymmetric actor-critic)
- 보상: 컵 안 위치 도달 + grasp_detector 신호 + smoothness

### 학습 순서
1. 시뮬에서 IL 사전학습 (Step 3 데이터)
2. 시뮬에서 HIL-SERL residual 학습 (수 시간)
3. 실물 데모 30~50개로 IL fine-tune
4. 실물에서 HIL-SERL 추가 학습 (1~2시간, 안전 가드 필수)

### Sim-to-Real
- Domain randomization (필수)
- 카메라 캘리브레이션 (실물 ↔ 시뮬 동일)
- joint angle / gripper command 단위 통일
- workspace guard (Step 0 자산 활용)

### TODO
- [ ] OMX-F MJCF 작성
- [ ] Gymnasium 환경 wrapper (`OmxBoxToCupEnv-v0`)
- [ ] Domain randomization 파라미터 정의
- [ ] HIL-SERL config 작성
- [ ] 시뮬 ↔ 실물 데이터/액션 단위 일치 확인
- [ ] 실물 RL 안전 가드 (workspace, current limit, e-stop)

---

## 5. Step 5 — 전체 워크플로우 통합

### 시나리오: 자연어 → 실행
```
사용자: "박스를 컵에 넣어줘"
  ↓
LLM/router (기존)
  ↓
1. perception: detect_box → box_pose
2. perception: detect_cup → cup_pose
3. classical: pick_and_hover(box_pose, cup_pose)
4. learned: lerobot_policy_runner (IL + RL residual)
5. confirm: grasp_detector False + visual check → 성공
6. 사용자에게 결과 보고
```

### 산출물
- 통합 액션: `place_box_in_cup`
- 실패 시 fallback (어느 단계에서 실패했는지 보고)

### TODO
- [ ] `place_box_in_cup` 액션 정의
- [ ] 단계별 timeout / 실패 처리
- [ ] 자연어 → 액션 라우팅 (기존 시스템 통합)
- [ ] 통합 테스트 시나리오 작성 (10회 성공률 목표 ≥80%)

---

## 6. 마일스톤 / 우선순위

| 순서 | 항목 | 의존성 | 예상 기간 |
|---|---|---|---|
| M1 | Step 1: grasp_detector | — | 3~5일 |
| M2 | Step 2: pick_and_hover 액션 | M1 | 3~5일 |
| M3 | Step 4 일부: MuJoCo 환경 + 자동 데모 | — (병행 가능) | 1~2주 |
| M4 | Step 3: LeRobot IL 시뮬 학습 | M3 | 1~2주 |
| M5 | Step 4: HIL-SERL residual 시뮬 | M4 | 1주 |
| M6 | 실물 fine-tune + 통합 | M2, M5 | 2~3주 |

총 예상: **6~10주**

---

## 7. 미결정 / 결정 필요 항목

- [ ] **카메라 구성**: RGB만 vs RGB-D 추가? (DP3 갈지 RGB DP로 갈지)
- [ ] **텔레옵 도구**: SpaceMouse 구매 여부
- [ ] **시뮬레이터**: MuJoCo MJX vs Isaac Lab (현재 MuJoCo 가정)
- [ ] **OMX-F가 LeRobot 지원 OMX와 동일 config인지** (5-DOF 변형 확인 필요)
- [ ] **success 라벨링 자동화 방법**: visual? grasp_detector False + Z 위치?
- [ ] **데이터 저장 위치 / 용량 산정**: 1000 에피소드 × 8초 × 15Hz × 2카메라 ≈ 수 GB

---

## 8. 리스크

| 리스크 | 완화책 |
|---|---|
| Sim-to-real gap이 커서 실물 정책 실패 | Domain randomization 강하게, 실물 demo로 fine-tune |
| 5-DOF 한계로 컵 안 정렬 자세 제약 | hover pose를 5-DOF 도달 가능 영역으로 제한 |
| 종이컵 deformation 모델링 어려움 | rigid 근사 + 실물 demo로 보정 |
| RL 학습 중 실물 손상 | workspace guard + current limit + 사람 감독 (HIL) |
| 데이터 수집 시간 폭증 | 시뮬 자동 demo 우선, 실물은 fine-tune용 최소량 |
