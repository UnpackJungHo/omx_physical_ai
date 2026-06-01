// include/omx_dynamixel_hw/dynamixel_system.hpp
#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "omx_interfaces/msg/dynamixel_diagnostics.hpp"
#include "omx_dynamixel_hw/dxl_bus.hpp"

namespace omx_dynamixel_hw {

// joint <-> dxl(gpio) 1:1 대응 설정. gpio 파라미터에서 채운다.
struct JointConfig {
  std::string name;          // URDF joint 이름 (prefix 포함)
  uint8_t id{0};             // DYNAMIXEL ID
  int operating_mode{3};
  int position_p{0}, position_i{0}, position_d{0};
  int profile_velocity{0}, profile_acceleration{0};
  bool has_position_limit{false};
  int min_position_limit{0}, max_position_limit{0};
  bool has_current{false};   // mode 5 (current-based position)
  int goal_current{0}, current_limit{0};
  bool expose_effort{false}; // gripper present current -> effort state
};

// 모터별 진단 스냅샷 (round-robin 으로 갱신).
struct MotorHealth {
  double present_current_ma{0.0};
  int temperature_c{0};
  double input_voltage_v{0.0};
  uint8_t hardware_error_status{0};
  bool comm_ok{true};
};

class OmxDynamixelSystem : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OmxDynamixelSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool apply_servo_config(const JointConfig & jc);  // write 실패 시 false
  void publish_diagnostics(const rclcpp::Time & time);

  std::vector<JointConfig> joints_;
  std::vector<uint8_t> ids_;
  std::vector<MotorHealth> health_;
  std::unique_ptr<DxlBus> bus_;

  // hardware_parameters (URDF) 에서 읽는 설정
  std::string port_{"/dev/ttyACM1"};
  int baud_{1000000};
  int diag_publish_decimation_{50};  // read 사이클 N 회마다 1회 진단 발행
  int diag_read_decimation_{10};     // read 사이클 N 회마다 1모터 진단 read (버스 부하 절감)
  int read_error_max_cycles_{50};    // 버스 전면 실패가 N 사이클 연속이면 read() ERROR 에스컬레이션
  bool disable_torque_at_deactivate_{true};

  // 진단 publisher (별도 노드, spin 불필요 - publish-only)
  std::shared_ptr<rclcpp::Node> diag_node_;
  std::shared_ptr<realtime_tools::RealtimePublisher<omx_interfaces::msg::DynamixelDiagnostics>>
    diag_pub_;

  size_t diag_rr_index_{0};   // round-robin 으로 한 사이클에 한 모터 진단 read
  int read_cycle_{0};
  int read_fail_streak_{0};   // sync read 가 전 모터 실패한 연속 사이클 수
};

}  // namespace omx_dynamixel_hw
