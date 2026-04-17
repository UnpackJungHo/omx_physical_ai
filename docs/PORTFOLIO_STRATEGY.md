# OMX 포트폴리오 전략 전환

## 문서 목적
- 이 문서는 현재 OMX 프로젝트를 취업용 포트폴리오 관점에서 어떻게 재정의할지 정리한다.
- 핵심 목표는 단순 로봇 데모가 아니라, `ROS2/DDS/QoS/엣지-서버 분산 제어` 역량이 드러나는 작품으로 올리는 것이다.

## 현재 프로젝트에 대한 냉정한 평가

### 현재 강점
- `omx_bringup`, `omx_interfaces`, `omx_motion_server`, `omx_perception`까지 레이어 분리가 비교적 선명하다.
- MoveIt2 기반 액션 서버와 perception 서비스 구조가 이미 존재한다.
- 하위 레이어를 먼저 고정하고 상위 레이어를 나중에 붙이는 개발 철학이 명확하다.

### 현재 한계
- 아직은 `잘 만든 단일 로봇 애플리케이션`에 가깝다.
- 네트워크 제약, DDS/QoS 설계, 장애 대응, 엣지-서버 역할 분리, 운영 지표가 작품의 중심이 아니다.
- 기능 수는 늘어날 수 있어도, 지금 상태만으로는 "시스템을 설계할 줄 아는 사람"이라는 인상을 강하게 주기 어렵다.

### 포트폴리오 관점 평가
- 로보틱스 구현 프로젝트로는 충분히 의미가 있다.
- 그러나 취업 포트폴리오로 `1 -> 10` 수준의 매력도를 만들려면 분산 시스템 축이 반드시 추가되어야 한다.

## 새로운 프로젝트 정의

### 목표 재정의
기존 목표:
- 자연어 명령을 받아 OMX 매니퓰레이터가 액션을 완수하는 Physical AI 데모

수정 목표:
- `네트워크 상태 변화와 서버 장애에도 작업을 지속하거나 안전 정지할 수 있는 Network-Adaptive Edge-Cloud Manipulation System`을 만든다.

### 최종적으로 보여줘야 할 것
- 엣지 노드와 서버 노드가 분리된 ROS2/DDS 시스템
- QoS 정책을 목적별로 구분해 적용한 설계
- 지연, 패킷 손실, 서버 단절 상황에서의 degraded mode 또는 fallback
- 실기기 manipulation과 운영 지표를 함께 보여주는 데모

## 시스템 포지셔닝

### 권장 한 줄 설명
`ROS2/DDS 기반으로 엣지 디바이스와 서버를 분리하고, 네트워크 지연·손실·서버 장애 상황에서도 QoS 재구성과 로컬 fallback으로 작업을 지속하는 OMX 매니퓰레이터 시스템`

### 면접에서 전달해야 할 메시지
- 실시간성이 필요한 제어는 엣지에 남긴다.
- 고수준 planning, 관제, 로깅, 선택적 LLM 계층은 서버에 둔다.
- 네트워크가 흔들려도 시스템은 fail-stop이 아니라 safe-degrade 하도록 설계한다.

## 엣지 컴퓨터 선택 결론

### 최종 권장안
- 개발과 초기 검증은 `노트북`
- 최종 데모와 포트폴리오 포지셔닝은 `Raspberry Pi 5 + 노트북 서버`

### 이유
- 노트북만 쓰면 개발은 쉽지만 `PC 2대 연결 데모`처럼 보일 가능성이 크다.
- Raspberry Pi 5를 엣지 노드로 쓰면 실제 배포형 시스템, 자원 제약, 경량화 판단, 장애 대응 설계를 함께 설명할 수 있다.
- 다만 처음부터 Pi 5에 모든 개발을 몰면 일정 리스크가 커지므로, 개발 단계와 최종 데모 단계를 분리한다.

## 구현 우선순위 조정

### 하지 말아야 할 것
- 로컬 단일 머신에서 `skill/planner/recovery/LLM` 전부를 완벽히 만든 뒤 분산 시스템으로 넘어가는 방식

### 해야 할 것
- 로컬 환경에서 `대표 시나리오 1개`를 먼저 닫는다.
- 그 다음 바로 `edge-cloud` 구조로 넘어간다.

### 로컬 단계의 중간 종료선
아래가 만족되면 로컬 기능 확장을 멈추고 분산 시스템으로 전환한다.
- `perception -> skill(rule) -> motion`이 end-to-end로 연결된다.
- 대표 시나리오 1개가 실기기에서 반복 가능하게 성공한다.
- 최소한의 retry 또는 safe stop 동작이 있다.
- 로그나 영상으로 재현성과 성공률을 증명할 수 있다.

## 대표 시나리오

### 최소 시나리오
- `빨간 블록을 왼쪽 상자에 넣어줘`

### 이 시나리오로 충분한 이유
- perception, target filtering, motion, gripper, placement까지 모두 드러난다.
- 분산 시스템 전환 후에도 동일 시나리오로 QoS, network degradation, fallback을 측정할 수 있다.

## 권장 분산 아키텍처

### Edge: Raspberry Pi 5
- `omx_perception`
- `omx_motion_server`
- 최소 `omx_recovery_manager`
- safety stop
- 마지막 유효 상태 보존

### Server: Notebook or PC
- `omx_task_planner`
- `mission control`
- `telemetry/log aggregator`
- optional `omx_llm_interface`
- dashboard or operator UI

### 분리 원칙
- 실시간 제어와 안전은 엣지
- 고수준 의사결정과 관제는 서버
- LLM은 반드시 비실시간, 비안전 경로에만 배치

## QoS / DDS 관점에서 반드시 보여줄 것
- perception stream: `BEST_EFFORT`, `KEEP_LAST`, 낮은 depth
- command/action/result: `RELIABLE`
- state/telemetry: 목적에 따라 별도 프로파일
- stale data 감지와 timeout 기준
- 서버 disconnect 시 edge fallback 또는 safe stop
- degraded mode 전환 조건과 복귀 조건

## 반드시 측정할 운영 지표
- end-to-end task latency
- perception freshness / stale ratio
- action success rate
- retry count
- timeout count
- network condition별 task completion rate
- normal mode / degraded mode 전환 횟수

## 데모 시나리오
1. 정상 네트워크에서 서버 planner가 작업을 지시하고 엣지가 수행한다.
2. 네트워크 지연 또는 손실이 증가하면 QoS 또는 실행 전략이 바뀐다.
3. 서버가 끊기면 엣지가 현재 작업을 안전하게 마무리하거나 정지한다.
4. 서버 복구 후 상태를 재동기화한다.
5. 대시보드에서 latency, retry, degraded mode 전환 기록을 확인한다.

## 현재 워크스페이스 기준 다음 우선순위
1. `omx_perception`을 현재 목표 수준까지 안정화한다.
2. `omx_skill_executor`에서 대표 시나리오 1개를 구현한다.
3. 최소 `retry` 또는 `safe stop` 수준의 recovery를 붙인다.
4. 여기까지 확인되면 곧바로 edge/server 분리 작업으로 넘어간다.
5. 이후 QoS 실험, fault injection, telemetry를 붙인다.
6. `align`, `stack`, `LLM`은 그 다음이다.

## 포트폴리오에서 피해야 할 포인트
- LLM이 로봇을 직접 제어하는 것처럼 보이게 만들기
- QoS/DDS를 말만 하고 실험과 수치가 없는 상태
- 기능 수만 많고 시스템 경계와 복구 전략이 없는 상태
- Pi 5를 썼지만 왜 그렇게 나눴는지 설명하지 못하는 상태

## 최종 결론
- OMX 프로젝트는 버릴 필요가 없다.
- 다만 작품의 중심을 `자연어 데모`에서 `network-adaptive edge-cloud robotic system`으로 옮겨야 한다.
- 현재 로컬 구현은 대표 시나리오 하나를 완성하는 수준까지만 밀고, 이후 바로 분산 시스템으로 확장하는 것이 포트폴리오 가치가 가장 높다.
