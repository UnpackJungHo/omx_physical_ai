# omx_dynamixel_hw

`dynamixel_sdk` 위에 `hardware_interface::SystemInterface`를 **직접 구현**한 ROS 2 Jazzy
ros2_control 하드웨어 패키지. 기성 `dynamixel_hardware_interface/DynamixelHardware`에 의존하지 않고
control table 통신/단위 변환/진단/안전정지를 손으로 작성해, OMX-F(5-DOF + gripper) DYNAMIXEL
버스를 MoveIt2 + JointTrajectoryController 스택에 드롭인으로 붙인다.

## 목적

- ros2_control `SystemInterface` lifecycle을 직접 구현(`on_init`/`on_configure`/`on_activate`/
  `read`/`write`/`on_deactivate`).
- `dynamixel_sdk`의 PortHandler/PacketHandler/GroupSyncRead/GroupSyncWrite를 얇은 래퍼(`DxlBus`)로 격리.
- 진단(temperature/voltage/hardware error/present current)을 토픽으로 발행하고,
  별도 프로세스 `dxl_watchdog`가 임계 초과 시 컨트롤러를 deactivate 해 안전정지.

## 아키텍처

```
MoveIt2 → JointTrajectoryController → controller_manager
   └─(command position)→ OmxDynamixelSystem.write() → GroupSyncWrite → DXL (ID 11..16)
   ←─(state pos/vel/effort)── OmxDynamixelSystem.read() ← GroupSyncRead ← DXL
                                   └→ /omx_dxl_hw_diag/dynamixel_diagnostics
                                          → dxl_watchdog → switch_controller(deactivate) 안전정지
```

구성 요소:

| 파일 | 역할 |
|------|------|
| `control_table.{hpp,cpp}` | X-series 2.0 레지스터 주소(프로토콜 상수) + tick↔rad / current↔mA / velocity↔rad·s⁻¹ / voltage 단위 변환(순수 함수, 단위 테스트) |
| `dxl_bus.{hpp,cpp}` | `dynamixel_sdk` 호출 격리. ping / read·write1·2·4 / GroupSyncRead(현재 current·velocity·position 연속 블록) / GroupSyncWrite(goal position) |
| `dynamixel_system.{hpp,cpp}` | `OmxDynamixelSystem : SystemInterface`. joint↔gpio 1:1 매핑, operating mode/PID/profile/limit/goal current 초기화, 진단 round-robin read + RealtimePublisher |
| `watchdog_logic.hpp` | 안전정지 임계 판정 순수 함수(header-only, 단위 테스트) |
| `dxl_watchdog_node.cpp` | 진단 구독 → 연속 통신실패/과열/저전압/HW error 판정 → `switch_controller` deactivate |
| `dxl_ping_tool.cpp` | 실HW ping/scan 스모크 도구(standalone) |

## 사용법 (opt-in)

기존 `DynamixelHardware` 경로는 그대로 두고, `use_custom_hw:=true`로만 전환한다.

```bash
# 실HW ping 스모크 (다른 ros2_control_node가 포트를 점유하지 않은 상태)
ros2 run omx_dynamixel_hw dxl_ping_tool /dev/ttyACM1 1000000

# 커스텀 HW로 전체 스택 구동 (+ watchdog 자동 실행)
ros2 launch omx_bringup omx_control.launch.py use_custom_hw:=true port_name:=/dev/ttyACM1

# 상태 확인
ros2 topic echo /joint_states --once
ros2 control list_hardware_interfaces
ros2 topic echo /omx_dxl_hw_diag/dynamixel_diagnostics --once
```

기본값은 `use_custom_hw:=false` → 기존 동작 불변. `use_mock_hardware:=true`(mock),
`use_sim:=true`(Gazebo) 경로도 그대로다.

## 하드웨어 사실 (omx_f_grasp.ros2_control.xacro)

- ID 11=joint1(mode4, extended position), 12~15=joint2~5(mode3, position),
  16=gripper_joint_1(mode5, current-based position).
- Protocol 2.0, baud 1,000,000, 기본 포트 `/dev/ttyACM1`.
- gripper: Current Limit 600 / Goal Current 550 unit, Present Current를
  `/joint_states.effort[gripper_joint_1]`로 노출(unit→mA 변환).

모터별 ID/Operating Mode/PID/Profile/Limit/Goal Current는 xacro의 `<gpio dxl11..16>` 블록에서
읽는다(joint 선언 순서와 gpio 선언 순서를 1:1로 매핑). 레지스터 **주소**는 프로토콜 상수로
`control_table.hpp`에 고정하고, PID/current/limit/profile/baud/port/진단주기/임계값 등 **튜닝값**은
URDF 파라미터·`config/dxl_watchdog.yaml`로 노출한다(코드 하드코딩 금지).

## 진단 / 안전정지 파라미터 (`config/dxl_watchdog.yaml`)

| 파라미터 | 기본값 | 의미 |
|----------|--------|------|
| `max_temperature_c` | 70 | Present Temperature[°C] 초과 시 정지 |
| `min_input_voltage_v` | 9.5 | Present Input Voltage[V] 미만 시 정지 |
| `max_comm_fail_streak` | 5 | 연속 통신 실패 횟수 초과 시 정지(단발 드롭은 무시) |
| `diagnostics_topic` | `/omx_dxl_hw_diag/dynamixel_diagnostics` | 진단 토픽 |
| `switch_controller_service` | `controller_manager/switch_controller` | 안전정지 경로(상대경로) |
| `deactivate_controllers` | `[arm_controller, gripper_controller]` | 정지 시 deactivate 대상 |

HW의 진단 발행 주기는 URDF `<param name="diagnostics_publish_decimation">`(read 사이클 수, 기본 50)로 조절.

## 테스트

```bash
colcon build --packages-select omx_dynamixel_hw
colcon test --packages-select omx_dynamixel_hw && colcon test-result --verbose
```

- `test_control_table`: 레지스터 주소 상수, tick↔rad round-trip, current/velocity/voltage 변환, signed decode.
- `test_watchdog_logic`: nominal/과열/HW error/comm streak/저전압 판정.

watchdog 통합(실HW 불필요): 진단 토픽을 수동 발행해 안전정지 로그를 확인.

```bash
ros2 run omx_dynamixel_hw dxl_watchdog_node &
ros2 topic pub --once /omx_dxl_hw_diag/dynamixel_diagnostics omx_interfaces/msg/DynamixelDiagnostics \
  "{ids: [12], temperature_c: [99], input_voltage_v: [12.0], hardware_error_status: [0], comm_ok: [true], present_current_ma: [0.0]}"
# -> "SAFETY STOP id=12: temperature over limit"
```

## 설계 메모

- **read 효율**: Present Current(126,2)/Velocity(128,4)/Position(132,4)이 연속 주소이므로
  한 번의 GroupSyncRead(start=126, len=10)로 세 값을 동시에 읽는다. 느린 진단 레지스터
  (temperature/voltage/hardware error)는 사이클당 1모터 round-robin으로 읽어 버스 부하를 분산한다.
- **command init**: `on_activate`에서 현재 위치를 1회 read 해 command 인터페이스를 현재 위치로
  초기화(활성화 시 점프 방지).
- **realtime**: 진단 발행은 `realtime_tools::RealtimePublisher`의 `trylock()`만 사용(read 루프 비차단).
  GroupSyncRead/Write 객체는 `on_activate`에서 1회 생성(주기적 할당 회피).
- **sync vs async**: `ros2_control` 태그가 `is_async="true"`라 read/write는 별도 스레드에서 호출된다.
  100Hz 루프 read+write latency는 실HW에서 측정 후 아래에 기록.

### 100Hz 루프 latency (실HW 측정)

> TODO(실HW): `ros2 topic hz /joint_states`와 read+write 1주기 소요시간을 측정해 기록.
> margin이 부족하면 추가 튜닝(진단 decimation 상향, 또는 broadcaster 분리 - 접근 C) 검토.

## 한계 (YAGNI)

- position 명령 인터페이스만 패리티(velocity/current command 미구현). gripper는 mode5
  current-based position으로 soft grasp.
- X-series control table 상수 고정(다중 모델 일반화는 접근 C에서 `.model` 로더로 별도 진행).
- 기존 `DynamixelHardware` 경로는 수정/제거하지 않는다.
```

