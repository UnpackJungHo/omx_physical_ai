# OMX 프로젝트 — CLAUDE.md

## 역할
- 당신은 OMX 프로젝트를 구현하는 AI 에이전트다.
- 목표는 OMX 프로젝트를 취업 포트폴리오로서 경쟁력 있는 `edge-cloud robotic system`으로 완성하는 것이다.
- 세션 시작 시 이 문서를 기준으로 현재 우선순위와 제약을 파악하고, 상세 내용은 관련 문서에서 확인한다.

## 프로젝트 한 줄 정의
ROS2/DDS 기반으로 OMX 매니퓰레이터를 `엣지 제어 노드 + 서버 관제 노드`로 분리하고, QoS/네트워크 장애/복구 전략까지 포함한 `network-adaptive manipulation system`을 만든다.

## 현재 최우선 목표
<!-- AUTO_PHASE_STATUS_START -->
- 현재 구현 우선순위는 `omx_perception` (4단계), 컬러 기반 블록 위치 감지 API 구축이다.
- 단계 현황: `1 done`, `2 done`, `3 done (omx_motion_server)`, `4 in_progress (omx_perception)`
- motion API 완료. 다음은 `omx_perception` — GetBlockPoses 서비스 구현이다.
- 승인 대기: 없음
- 세부 상태 보드는 `docs/STATUS.md`, 실행 컨텍스트는 `docs/SESSION_NEXT.md`를 기준으로 본다.
<!-- AUTO_PHASE_STATUS_END -->

## 반드시 지킬 운영 원칙
- 개발 순서는 반드시 `bringup -> motion API -> perception -> 대표 skill 1개 -> 최소 recovery -> edge/server 분리 -> QoS/telemetry -> 확장 skill/planner/LLM` 순서를 따른다.
- LLM은 의미 해석만 담당하고, 저수준 제어와 복구는 규칙 기반 로직과 MoveIt2/Servo가 담당한다.
- 로컬 단일 머신에서 모든 기능 폭을 끝내기보다, 대표 시나리오 1개가 안정화되면 빠르게 분산 시스템 축으로 전환한다.
- LLM 없이도 동작 가능한 단계까지 완성한 뒤 다음 계층으로 올라간다.
- 하드웨어 보호를 위해 workspace 제한과 안전 제약을 먼저 설계한다.
- 실시간 제어와 safety는 엣지에 남기고, 고수준 planning/관제/선택적 LLM 계층은 서버에 둔다.
- QoS, DDS, timeout, stale data, degraded mode, fallback은 포트폴리오의 핵심 설계 대상이다.
- ROS2 노드 이름은 프로세스 내 역할별로 유일해야 하며, 액션 서버 노드와 MoveIt/Transform helper 노드에 같은 이름을 재사용하지 않는다.
- 패키지 간 계약, 언어 분리, 코드 규칙은 이 문서가 아니라 전용 문서를 기준으로 유지한다.

## 완료 승인 프로토콜
- 에이전트가 특정 Phase 완료 승인을 요청할 때는 응답 마지막 줄에 정확히 `[PHASE_COMPLETE_REQUEST:<PHASE>]` 마커를 남긴다.
- 예시: `[PHASE_COMPLETE_REQUEST:C]`
- 사용자가 `승인: C 완료`처럼 승인하면 Hook이 상태 파일과 문서를 자동 갱신한다.
- 자연어 승인도 일부 허용하지만, 오탐 방지를 위해 `승인: C 완료` 형식을 권장한다.
- 자동 갱신 대상의 관리 블록은 수동 편집하지 않는다.

## 문서 인덱스
- `docs/GOALS.md`: 데모 목표, 성공 기준, 타겟 액션
- `docs/ARCHITECTURE.md`: 확정 아키텍처, 레이어 책임, LLM 입출력 경계
- `docs/PORTFOLIO_STRATEGY.md`: 포트폴리오 방향 전환, edge-cloud 목표, 우선순위 재정의
- `docs/STRUCTURE.md`: 패키지 구조, 개발 순서, 패키지별 책임
- `docs/CONSTRAINTS.md`: 하드웨어 제약, 안전 제약, 비기능 요구사항
- `docs/LANGUAGE_STRATEGY.md`: C++/Python 분리 기준
- `docs/CODING_RULES.md`: 인터페이스 규약, 반환 규칙, 구현 규칙
- `docs/STATUS.md`: Phase 상태 보드와 승인 이력
- `docs/SESSION_NEXT.md`: 다음 세션 이어하기 정보
- `docs/AUTOMATION.md`: Hook 기반 자동 상태 갱신 규칙

## 작업 시작 체크
- 구현 전에 현재 작업과 직접 관련된 문서를 먼저 읽는다.
- 문서 간 충돌이 있으면 임의 구현보다 문서 정합성부터 맞춘다.
- 포트폴리오 관련 의사결정은 `docs/PORTFOLIO_STRATEGY.md`를 기준으로 판단한다.
- 구조 변경이 생기면 `CLAUDE.md`에는 요약만 남기고 상세 내용은 해당 전용 문서를 수정한다.
