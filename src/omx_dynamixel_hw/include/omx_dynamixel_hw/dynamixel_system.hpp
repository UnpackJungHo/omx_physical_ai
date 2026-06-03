// include/omx_dynamixel_hw/dynamixel_system.hpp
#pragma once
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "omx_interfaces/msg/dynamixel_diagnostics.hpp"
#include "omx_interfaces/srv/reboot_dxl.hpp"
#include "omx_dynamixel_hw/dxl_bus.hpp"

namespace omx_dynamixel_hw {

// joint <-> dxl(gpio) 1:1 대응 설정. gpio 파라미터에서 채운다.
// 고정
struct JointConfig {
  std::string name;          // URDF joint 이름 (prefix 포함)
  uint8_t id{0};             // DYNAMIXEL ID
  int operating_mode{3};     // 3 = position control, 5 = current-based position
  int position_p{0}, position_i{0}, position_d{0}; // URDF 에서 주입
  int profile_velocity{0}, profile_acceleration{0};
  bool has_position_limit{false};                    // joint limit
  int min_position_limit{0}, max_position_limit{0};  // ..
  bool has_current{false};   // mode 5 (current-based position)
  int goal_current{0}, current_limit{0}; // gripper의 힘 제어(과도하게 안 쥐게)
  bool expose_effort{false}; // gripper present current -> effort state
  uint16_t model{0};         // ping 으로 확인한 DYNAMIXEL model number (진단 발행/모델별 임계용)
  double current_ma_per_unit{2.69};  // model 별 current 단위 (on_configure 의 ping model 로 설정)
};

// 모터별 진단 스냅샷 (round-robin 으로 갱신)
// 매번 갱신
struct MotorHealth {
  double present_current_ma{0.0};
  int temperature_c{0};
  double input_voltage_v{0.0};
  uint8_t hardware_error_status{0};
  bool comm_ok{true};
  bool diag_valid{false}; // temp/voltage/hwerr 가 round-robin 으로 1회 이상
                          // 채워졌는지 -> watchdog 에서 모터 진단 여부를 확인 후 에러 여부 판정
};

class OmxDynamixelSystem : public hardware_interface::SystemInterface {
public:
  // RCLCPP_SHARED_PTR_DEFINITIONS: ros2 매크로 - 이 클래스의 SharedPtr 타입과
  // make_shared 헬퍼를 자동 생성, controller_manager 가 플러그인으로 이 클래스를 shard_ptr로 들기 때문에 필요
  RCLCPP_SHARED_PTR_DEFINITIONS(OmxDynamixelSystem)

  ~OmxDynamixelSystem() override;

  // 로드 시 1회 URDF 파라미터 파싱 -> joints_ 채움
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
      override;
  // 설정, 포트 open, ping -> model 확인, 진단 노드/서비스 스레드 시작
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  // 활성화, 토크 ON, P/I/D 등 servo config write, setup_groups, 현재 위치를 읽어 점프 방지
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  // 비활성화 토크 off(옵션)
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  // 매 사이클(100Hz), sync_read_present_states → decode → state 노출
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
    // 매 사이클(100Hz), command → rad_to_tick → sync_write_goal_position 
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool apply_servo_config(const JointConfig & jc);  // write 실패 시 false
  void publish_diagnostics(const rclcpp::Time & time);
  void start_service_thread();
  void stop_service_thread();
  void on_set_torque(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                     std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  void on_reboot(const std::shared_ptr<omx_interfaces::srv::RebootDxl::Request> req,
                 std::shared_ptr<omx_interfaces::srv::RebootDxl::Response> res);

  std::vector<JointConfig> joints_;  // 모터별 설정
  std::vector<uint8_t> ids_;         // ID 목록(sync read/write 용)
  std::vector<MotorHealth> health_;  // 모터별 건강
  std::unique_ptr<DxlBus> bus_;      // 통신 래퍼
  std::mutex bus_mtx_;   // 모든 bus_ 접근 직렬화 (시리얼 단일회선 - 동시접근 시 패킷 손상)
  // 시리얼은 물리적으로 1회선, 그런데 이 시스템에서 두 개의 스레드가 버스를 쓰려함
  // RT 스레드: controller_manager 가 부르는 read()/write() (100Hz)
  // 서비스 스레드: on_reboot/on_set_torque (사용자 요청 시)

  // hardware_parameters (URDF) 에서 읽는 설정
  std::string port_{"/dev/ttyACM1"};
  int baud_{1000000};
  int diag_publish_decimation_{50};  // read 사이클 N 회마다 1회 진단 발행
  int diag_read_decimation_{10};     // read 사이클 N 회마다 1모터 진단 read (버스 부하 절감)
  int read_error_max_cycles_{50};    // 버스 전면 실패가 N 사이클 연속이면 read() ERROR 에스컬레이션
  bool disable_torque_at_deactivate_{true};

  // 별도 노드: 진단 publish + 운영 서비스(reboot/torque). RT 경로(read/write)와 분리된
  // 전용 executor 스레드에서 spin 한다 (로보티즈처럼 read() 안에서 spin 하지 않음).
  std::shared_ptr<rclcpp::Node> diag_node_;
  std::shared_ptr<realtime_tools::RealtimePublisher<omx_interfaces::msg::DynamixelDiagnostics>>
    diag_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_torque_srv_;
  rclcpp::Service<omx_interfaces::srv::RebootDxl>::SharedPtr reboot_srv_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> svc_exec_;
  std::thread svc_thread_;
  std::atomic<bool> svc_running_{false};

  size_t diag_rr_index_{0};   // round-robin 으로 한 사이클에 한 모터 진단 read
  int read_cycle_{0};
  int read_fail_streak_{0};   // sync read 가 전 모터 실패한 연속 사이클 수
};

}  // namespace omx_dynamixel_hw
