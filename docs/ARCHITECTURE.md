# OMX 시스템 아키텍처

## 목표 아키텍처

이 프로젝트의 최종 형태는 `단일 머신 데모`가 아니라
`엣지 제어 + 서버 관제`로 분리된 network-adaptive robotic system이다.

## 단계별 아키텍처

### 1단계: 로컬 검증 아키텍처

```text
operator / test script
    ↓
[omx_skill_executor]   대표 시나리오 실행 (pick/place)
    ↓
[omx_motion_server]    MoveIt2 계획 + Servo 실행 + 그리퍼 제어
    ↕
[omx_perception]       컬러 감지 -> 블록 위치 추정
    ↕
[omx_recovery_manager] 최소 retry / safe stop
```

### 2단계: 최종 edge-cloud 아키텍처

```text
remote operator / mission UI
    ↓
[server: omx_task_planner]
    ↓
[server: mission control / telemetry / optional omx_llm_interface]
    ↕
==================== DDS / Network ====================
    ↕
[edge: omx_skill_executor]
    ↓
[edge: omx_motion_server]
    ↕
[edge: omx_perception]
    ↕
[edge: omx_recovery_manager]
```

## 레이어 책임
- `omx_perception`: 카메라 입력에서 블록 위치를 추정한다. 실시간성이 높으므로 엣지에 둔다.
- `omx_motion_server`: MoveIt2와 Servo를 통해 실제 모션 계획과 실행을 담당한다. safety critical 경로이므로 엣지에 둔다.
- `omx_skill_executor`: 대표 시나리오 단위의 실행 순서를 담당한다. 초기에는 로컬, 최종적으로는 엣지에 둔다.
- `omx_recovery_manager`: timeout, stale data, grasp 실패, server disconnect에 대한 retry / fallback / safe stop을 담당한다.
- `omx_task_planner`: 서버에서 고수준 작업 시퀀스를 생성한다. 실시간 제어권은 갖지 않는다.
- `omx_llm_interface`: 자연어를 구조화된 태스크 JSON으로 변환한다. 선택적 계층이며 비실시간 서버 경로에만 둔다.
- `mission control / telemetry`: 원격 조작, 상태 관제, 로그/메트릭 집계를 담당한다.

## 핵심 원칙
- LLM은 의미 해석만 한다. 조인트 직접 예측이나 실시간 제어는 하지 않는다.
- 복구는 LLM이 아니라 명시적 상태 머신과 규칙 기반 로직이 담당한다.
- perception과 motion은 planner, LLM과 느슨하게 결합한다.
- 실시간 제어와 safety는 엣지에 남기고, 고수준 planning/관제는 서버에 둔다.
- 로컬 환경에서는 대표 시나리오 1개를 먼저 닫고, 그 이후에 edge-cloud 구조로 확장한다.

## QoS / 통신 원칙
- perception stream은 최신성 우선이므로 `BEST_EFFORT`, `KEEP_LAST`, 작은 depth를 우선 검토한다.
- command, action goal/result, critical state는 `RELIABLE`을 우선한다.
- stale data, timeout, 서버 단절은 명시적으로 감지해야 한다.
- degraded mode 진입 조건과 정상 복귀 조건을 정의한다.

## LLM 출력 계약

```json
{
  "task": "pick_place",
  "object": "red_block",
  "target": "left_box",
  "retry_on_fail": true
}
```

## 아키텍처 변경 기준
- 레이어 책임이 바뀌는 수정은 이 문서를 먼저 갱신한 뒤 구현한다.
- 임시 우회 구현으로 레이어 경계를 무너뜨리지 않는다.
- 로컬 기능을 추가할 때도, 최종 edge-cloud 분리와 충돌하는 구조는 피한다.
