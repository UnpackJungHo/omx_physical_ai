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

### HW 인터페이스 파라미터 (URDF `<hardware>` 내 `<param>`)

| 파라미터 | 기본값 | 의미 |
|----------|--------|------|
| `port_name` | `/dev/ttyACM1` | 시리얼 포트 |
| `baud_rate` | 1000000 | 통신 속도 |
| `diagnostics_publish_decimation` | 50 | read 사이클 N 회마다 진단 1회 발행 |
| `diagnostics_read_decimation` | 10 | read 사이클 N 회마다 1모터 진단 레지스터(temp/voltage/error) read - 매 사이클 blocking TxRx 방지 |
| `read_error_max_cycles` | 50 | 버스 전면 read 실패가 N 사이클 연속이면 `read()` 가 ERROR 반환(resource manager 에스컬레이션) |
| `disable_torque_at_deactivate` | true | on_deactivate 시 torque off 여부 |

**2단계 실패 처리 설계:** 개별 모터 드롭아웃은 `comm_ok=false` 로 진단->watchdog(컨트롤러 deactivate)에 위임하고,
버스 전면 실패가 `read_error_max_cycles` 연속이면 `read()` 가 ERROR 를 반환해 controller_manager 가
HW 를 비활성화한다. 로보티즈 `error_timeout_ms`(시간 기반 단일 에스컬레이션)와 달리, "개별 모터 vs 버스 전체"를
분리해 처리한다.

## 운영 서비스 (실행 중 모터 진단/복구)

커스텀 HW 가 활성일 때 `omx_dxl_hw_diag` 노드가 두 서비스를 advertise 한다. 실 로봇에서 과부하로
멈춘 모터를 stack 재시작 없이 복구하거나 토크를 끄는 용도.

| 서비스 | 타입 | 동작 |
|--------|------|------|
| `/omx_dxl_hw_diag/set_torque` | `std_srvs/srv/SetBool` | `data: true` 전 모터 torque on, `false` off |
| `/omx_dxl_hw_diag/reboot` | `omx_interfaces/srv/RebootDxl` | `id` 모터 reboot (`id: 0` = 전체). 과부하/HW error latch 해제 |

```bash
ros2 service call /omx_dxl_hw_diag/reboot omx_interfaces/srv/RebootDxl "{id: 13}"
ros2 service call /omx_dxl_hw_diag/set_torque std_srvs/srv/SetBool "{data: false}"
```

**동시성 설계:** 시리얼 버스는 단일 회선이라 동시 접근 시 패킷이 손상된다. 모든 `bus_` 접근을
`std::mutex bus_mtx_` 로 직렬화하고, 서비스는 **RT 경로(read/write)와 분리된 전용 executor 스레드**에서
spin 한다. 락은 트랜잭션(~1-2ms) 동안만 잡는다. - 로보티즈 `DynamixelHardware` 가 `read()` 루프 안에서
`rclcpp::spin_some()` 으로 서비스 콜백을 처리하는 것(RT 경로에 비결정적 콜백 혼입)과 대비되는 지점.

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

