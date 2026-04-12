# OMX 언어 전략

## 기본 원칙
- C++는 실시간 제어와 성능이 중요한 로직에 사용한다.
- Python은 노드 오케스트레이션, 퍼셉션, LLM 연동처럼 생산성이 중요한 영역에 사용한다.
- 언어 선택은 성능과 책임 경계를 기준으로 결정한다.

## C++ 우선 패키지
- `omx_motion_server`: MoveIt2, Servo, 그리퍼 제어
- `omx_skill_executor`: 스킬 실행 핵심 로직
- `omx_recovery_manager`: 상태 머신, 재시도 로직
- `omx_task_planner`: 액션 서버, 규칙 기반 시퀀싱

## Python 우선 패키지
- `omx_perception`: OpenCV, 카메라 처리
- `omx_llm_interface`: LLM API, JSON 변환
- `omx_bringup`: launch 파일과 실행 조립
- `omx_interfaces`: 언어보다 ROS2 인터페이스 계약 안정성을 우선해 관리

## 예외 기준
- 성능 병목이 확인되면 Python 구현을 C++로 이전할 수 있다.
- 프로토타이핑은 Python으로 시작할 수 있지만, 실시간성 요구가 생기면 재배치한다.
