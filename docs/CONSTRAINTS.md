# OMX 제약 사항

## 하드웨어 제약
- GPU: RTX 4060 Ti
- VRAM: 8GB
- 로봇: ROBOTIS OMX
- OMX는 ROS2 native 기반이며 LeRobot 공식 지원 대상이다.

## 안전 제약
- MoveIt2 Planning Scene으로 workspace 제한을 반드시 유지한다.
- 바닥, 테이블, 작업 영역 제한은 bringup 단계부터 고려한다.
- 하드웨어 보호 관련 제약은 기능 확장보다 우선한다.

## 비기능 요구사항
- 로컬 환경에서 구동 가능한 모델 크기와 지연 시간을 우선 고려한다.
- perception은 초기에는 컬러 감지를 우선하고, 필요 시 ArUco나 depth 보조를 검토한다.
- 복구 로직은 설명 가능한 규칙 기반으로 유지한다.

## 핵심 기술 스택
| 레이어 | 기술 |
|--------|------|
| 로봇 제어 | ROS2, ros2_control, MoveIt2, MoveIt Servo |
| 비전 | OpenCV, depth camera |
| LLM | 로컬 instruct 모델 + 양자화 |
| 학습 | LeRobot, SmolVLA (450M), Imitation Learning |
