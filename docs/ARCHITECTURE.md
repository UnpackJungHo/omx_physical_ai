# OMX 시스템 아키텍처

## 확정 아키텍처

```text
자연어 입력
    ↓
[omx_llm_interface]   LLM -> JSON 구조화 목표 변환
    ↓
[omx_task_planner]    규칙 기반 태스크 시퀀스 생성
    ↓
[omx_skill_executor]  스킬 단위 실행 (pick, place, align, stack)
    ↓
[omx_motion_server]   MoveIt2 계획 + Servo 실행 + 그리퍼 제어
    ↕
[omx_perception]      컬러 감지 -> 블록/상자 위치 추정
    ↕
[omx_recovery_manager] 상태 머신 기반 복구
```

## 레이어 책임
- `omx_llm_interface`: 자연어를 구조화된 태스크 JSON으로 변환한다.
- `omx_task_planner`: JSON 목표를 실행 가능한 스텝 시퀀스로 바꾼다.
- `omx_skill_executor`: pick, place, align, stack 등 스킬을 조합해 실행한다.
- `omx_motion_server`: MoveIt2와 Servo를 통해 실제 모션 계획과 실행을 담당한다.
- `omx_perception`: 카메라 입력에서 블록과 상자의 위치를 추정한다.
- `omx_recovery_manager`: 실패 감지, 재시도, fallback 로직을 상태 머신으로 처리한다.

## 핵심 원칙
- LLM은 의미 해석만 한다. 조인트 직접 예측이나 실시간 제어는 하지 않는다.
- 복구는 LLM이 아니라 명시적 상태 머신과 규칙 기반 로직이 담당한다.
- perception과 motion은 planner, LLM과 느슨하게 결합한다.

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
