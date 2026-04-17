# OMX 패키지 구조와 개발 순서

## 패키지 구조

```text
omx_ws/src/
├── omx_bringup          # 1단계: 로봇 기동 (ros2_control + MoveIt2)
├── omx_interfaces       # 2단계: 커스텀 ROS2 msg/srv/action 정의
├── omx_motion_server    # 3단계: MoveIt2 + Servo + 그리퍼 API
├── omx_perception       # 4단계: 카메라 + 컬러 기반 블록 인식
├── omx_skill_executor   # 5단계: 대표 시나리오 스킬 실행
├── omx_recovery_manager # 6단계: retry / fallback / safe stop
├── omx_task_planner     # 7단계: 서버 측 태스크 시퀀서
├── omx_mission_control  # 8단계: 원격 조작 / 관제 / 명령 라우팅
├── omx_telemetry        # 9단계: 메트릭 / 로그 / 상태 집계
└── omx_llm_interface    # 10단계: 선택적 자연어 계층
```

## 개발 순서
1. `omx_bringup`
   로봇 모델, ros2_control, MoveIt2, launch 통합을 먼저 안정화한다.
2. `omx_interfaces`
   이후 패키지가 공유할 msg, srv, action 계약을 고정한다.
3. `omx_motion_server`
   안전한 모션 실행 API를 제공한다.
4. `omx_perception`
   블록과 상자 위치를 안정적으로 추정한다.
5. `omx_skill_executor`
   하위 모듈을 조합해 대표 시나리오 1개를 end-to-end로 완성한다.
6. `omx_recovery_manager`
   최소 retry / safe stop / stale data 대응을 규칙 기반으로 완성한다.
7. `omx_task_planner`
   서버 측 고수준 작업 시퀀싱을 담당한다.
8. `omx_mission_control`
   원격 조작, 관제, 명령 라우팅 계층을 만든다.
9. `omx_telemetry`
   latency, retry, timeout, 상태 전환 같은 운영 지표를 수집한다.
10. `omx_llm_interface`
   마지막에 자연어 계층을 선택적으로 붙인다.

## 개발 철학
- bringup이 불안정한 상태에서 상위 계층으로 올라가지 않는다.
- motion API가 완성되기 전에는 skill 레이어를 확장하지 않는다.
- 로컬 단일 머신에서 모든 기능 breadth를 먼저 끝내지 않는다.
- 대표 시나리오 1개가 안정화되면 edge-cloud 구조로 빠르게 전환한다.
- 실시간 제어와 safety는 엣지, 고수준 planning과 관제는 서버에 둔다.
- LLM은 마지막에 붙인다. LLM 없이도 태스크가 동작해야 한다.

## 현재 기준 우선순위
1. `omx_perception` 안정화
2. `omx_skill_executor`로 `빨간 블록 -> 왼쪽 상자` 시나리오 구현
3. `omx_recovery_manager` 최소 기능 추가
4. edge/server 역할 분리
5. QoS, telemetry, fault injection 추가
6. 이후 `align`, `stack`, `LLM` 확장
