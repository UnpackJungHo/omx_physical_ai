# OMX 프로젝트 목표

## 프로젝트 목표
ROBOTIS OMX 매니퓰레이터를 `ROS2/DDS 기반 edge-cloud robotic system`으로 확장해,
네트워크 상태 변화와 서버 장애에도 작업을 지속하거나 안전 정지할 수 있는 취업용 포트폴리오 데모를 만든다.

## 단계별 성공 기준

### 1차 성공 기준: 로컬 대표 시나리오 완성
- `perception -> skill(rule) -> motion`이 end-to-end로 연결된다.
- 대표 시나리오 1개가 실기기에서 반복 가능하게 성공한다.
- 최소한의 `retry` 또는 `safe stop` 동작이 존재한다.
- 로그, 영상, 성공률로 재현성을 증명할 수 있다.

### 2차 성공 기준: edge-cloud 전환
- 엣지 노드와 서버 노드가 분리된 ROS2/DDS 구조로 동작한다.
- 실시간 제어와 safety는 엣지에 남고, 고수준 planning/관제는 서버에 배치된다.
- QoS 정책이 데이터 성격에 맞게 분리 적용된다.
- 네트워크 지연, 손실, 서버 단절 상황에서 degraded mode 또는 fallback이 동작한다.
- latency, success rate, retry count, timeout 같은 운영 지표를 수집하고 보여줄 수 있다.

## 설계 원칙
- LLM은 마지막에 붙인다. 의미 해석만 담당하며 실시간 제어권을 갖지 않는다.
- 기능 breadth보다 시스템 depth를 우선한다.
- 로컬 단일 머신에서 모든 기능을 끝내기보다, 대표 시나리오 1개가 닫히면 빠르게 분산 시스템으로 확장한다.
- QoS, DDS, timeout, stale data, fallback은 부가 기능이 아니라 핵심 포트폴리오 요소다.

## 타겟 액션
1. 블록 분류
   "빨간 블록을 왼쪽 상자에 넣어줘"
   색상은 빨강, 파랑, 초록을 우선 지원하고 타겟은 좌, 우 상자다.
2. 블록 정렬
   흩어진 블록 3개를 수평 한 줄로 정렬한다.
3. 블록 쌓기
   블록 3개를 수직으로 쌓고, 낙하 감지 시 자동 재시도한다.

## 대표 시나리오 우선순위
- 0차(스킬 skeleton): `PickDetected` — 지정된 색 블록을 감지→집어올려 스캔 포즈에서 release.
  perception → skill → motion → gripper 경로의 end-to-end 구조와 실패 경로(탐지 실패 시 스윕, 최종 fail 시 home 복귀)를 먼저 닫는다.
- 1차(대표 시나리오): `PickPlace` — `빨간 블록을 왼쪽 상자에 넣기`.
  이 시나리오 하나로 perception, target filtering, motion, gripper, placement, retry, telemetry까지 드러낼 수 있다.
- `align`, `stack`, `LLM`은 edge-cloud 전환 이후 확장 항목으로 둔다.

## 최종 데모에서 보여줄 것
1. 정상 네트워크에서 서버가 작업을 지시하고 엣지가 수행한다.
2. 네트워크 지연 또는 손실이 증가하면 QoS 또는 실행 전략이 바뀐다.
3. 서버가 끊기면 엣지가 현재 작업을 안전하게 마무리하거나 정지한다.
4. 서버 복구 후 상태를 재동기화한다.
5. 대시보드에서 latency, retry, degraded mode 전환 기록을 확인할 수 있다.
