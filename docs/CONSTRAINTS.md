# OMX 제약 사항

## 하드웨어 제약
- GPU: RTX 4060 Ti
- VRAM: 8GB
- 로봇: ROBOTIS OMX
- OMX는 ROS2 native 기반이며 LeRobot 공식 지원 대상이다.
- 최종 포트폴리오 데모의 엣지 노드는 `Raspberry Pi 5`를 우선 목표로 한다.
- 개발과 초기 검증은 노트북에서 진행하되, 최종 구조는 `Pi 5 + 노트북 서버` 분리를 기준으로 설계한다.

## 안전 제약
- MoveIt2 Planning Scene으로 workspace 제한을 반드시 유지한다.
- 바닥, 테이블, 작업 영역 제한은 bringup 단계부터 고려한다.
- 하드웨어 보호 관련 제약은 기능 확장보다 우선한다.
- 서버 또는 네트워크 이상 시 엣지는 fail-dangerous가 아니라 `safe stop` 또는 로컬 fallback으로 내려가야 한다.
- stale perception 데이터로 motion을 계속 진행하지 않는다.

## 비기능 요구사항
- 로컬 환경에서 구동 가능한 모델 크기와 지연 시간을 우선 고려한다.
- perception은 초기에는 컬러 감지를 우선하고, 필요 시 ArUco나 depth 보조를 검토한다.
- 복구 로직은 설명 가능한 규칙 기반으로 유지한다.
- 실시간 제어 경로에 LLM이나 원격 서버 의존성을 두지 않는다.
- 네트워크 조건 변화에 따른 degraded mode와 timeout 기준을 명시해야 한다.
- QoS 설정은 토픽/서비스/액션의 목적에 따라 분리 설계해야 한다.
- 포트폴리오 검증을 위해 latency, success rate, retry count, timeout count를 측정 가능해야 한다.

## 네트워크 / 시스템 제약
- perception 스트림은 최신성 우선, 명령/결과는 신뢰성 우선으로 다룬다.
- 서버 단절 시 엣지는 마지막 명령을 무한 재시도하지 않는다.
- 원격 관제는 실시간 제어 루프와 분리한다.
- 분산 구조는 `개발 편의`보다 `면접에서 설명 가능한 설계 판단`을 우선한다.

## 핵심 기술 스택
| 레이어 | 기술 |
|--------|------|
| 로봇 제어 | ROS2, ros2_control, MoveIt2, MoveIt Servo |
| 비전 | OpenCV, usb camera |
| 분산 통신 | DDS, QoS, ROS2 action/service/topic |
| 관제 / 운영 | telemetry, dashboard, logging, fault injection |
| LLM | 선택적 로컬 instruct 모델 + 양자화 |
| 학습 | LeRobot, SmolVLA (450M), Imitation Learning |
