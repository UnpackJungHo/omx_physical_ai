// src/dxl_watchdog_node.cpp
// 진단 토픽 구독 -> 임계 초과 시 컨트롤러 deactivate 로 안전정지.
// HW(read 루프)와 책임을 분리해 "실 로봇 시스템 이슈 분석/개선"을 별도 프로세스로 증명.
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "omx_interfaces/msg/dynamixel_diagnostics.hpp"
#include "omx_dynamixel_hw/watchdog_logic.hpp"

using omx_dynamixel_hw::JointHealth;
using omx_dynamixel_hw::should_safety_stop;
using omx_dynamixel_hw::WatchdogThresholds;

class DxlWatchdog : public rclcpp::Node {
public:
  DxlWatchdog() : Node("dxl_watchdog") {
    th_.max_temperature_c = declare_parameter<int>("max_temperature_c", 70);
    th_.min_input_voltage_v = declare_parameter<double>("min_input_voltage_v", 9.5);
    th_.max_comm_fail_streak = declare_parameter<int>("max_comm_fail_streak", 5);
    const auto topic = declare_parameter<std::string>(
      "diagnostics_topic", "/omx_dxl_hw_diag/dynamixel_diagnostics");
    // 상대경로 기본값: 네임스페이스 push 시 함께 따라가도록 (project_relative_names_namespace)
    switch_service_ = declare_parameter<std::string>(
      "switch_controller_service", "controller_manager/switch_controller");
    deactivate_controllers_ = declare_parameter<std::vector<std::string>>(
      "deactivate_controllers", std::vector<std::string>{"arm_controller", "gripper_controller"});

    sub_ = create_subscription<omx_interfaces::msg::DynamixelDiagnostics>(
      topic, rclcpp::QoS(10),
      std::bind(&DxlWatchdog::on_diag, this, std::placeholders::_1));
    cli_ = create_client<controller_manager_msgs::srv::SwitchController>(switch_service_);

    RCLCPP_INFO(get_logger(),
                "dxl_watchdog up. topic=%s temp>%d voltage<%.1f comm_streak>%d -> deactivate via %s",
                topic.c_str(), th_.max_temperature_c, th_.min_input_voltage_v,
                th_.max_comm_fail_streak, switch_service_.c_str());
  }

private:
  void on_diag(const omx_interfaces::msg::DynamixelDiagnostics::SharedPtr m) {
    if (stopped_) return;  // 이미 안전정지 발동했으면 반복 호출 방지
    for (size_t i = 0; i < m->ids.size(); ++i) {
      const uint8_t id = m->ids[i];
      // 연속 통신 실패 카운터를 노드에서 추적 (단발 드롭으로 트립하지 않도록)
      if (i < m->comm_ok.size() && !m->comm_ok[i]) {
        comm_fail_streak_[id] += 1;
      } else {
        comm_fail_streak_[id] = 0;
      }
      JointHealth h;
      h.temperature_c = (i < m->temperature_c.size()) ? m->temperature_c[i] : 0;
      h.input_voltage_v = (i < m->input_voltage_v.size()) ? m->input_voltage_v[i] : 999.0;
      h.hardware_error_status =
        (i < m->hardware_error_status.size()) ? m->hardware_error_status[i] : 0;
      h.comm_fail_streak = comm_fail_streak_[id];

      const auto d = should_safety_stop(h, th_);
      if (d.triggered) {
        trigger_stop(id, d.reason);
        return;
      }
    }
  }

  void trigger_stop(uint8_t id, const std::string & reason) {
    RCLCPP_ERROR(get_logger(), "SAFETY STOP id=%u: %s", id, reason.c_str());
    if (!cli_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "switch_controller 서비스 미준비 - 안전정지 호출 실패: %s",
                   switch_service_.c_str());
      return;
    }
    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->deactivate_controllers = deactivate_controllers_;
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    cli_->async_send_request(req);
    stopped_ = true;
  }

  WatchdogThresholds th_{};
  std::string switch_service_;
  std::vector<std::string> deactivate_controllers_;
  std::map<uint8_t, int> comm_fail_streak_;
  bool stopped_{false};
  rclcpp::Subscription<omx_interfaces::msg::DynamixelDiagnostics>::SharedPtr sub_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr cli_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DxlWatchdog>());
  rclcpp::shutdown();
  return 0;
}
