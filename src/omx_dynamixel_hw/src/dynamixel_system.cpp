// src/dynamixel_system.cpp
#include "omx_dynamixel_hw/dynamixel_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "omx_dynamixel_hw/control_table.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace {
constexpr char kSetTorqueService[] = "~/set_torque";
constexpr char kRebootService[] = "~/reboot";
}  // namespace

namespace omx_dynamixel_hw {

namespace {
// gpio 파라미터(map<string,string>) 헬퍼. 키가 없으면 default 반환.
int param_int(const std::unordered_map<std::string, std::string> & p,
              const std::string & key, int def) {
  auto it = p.find(key);
  return it != p.end() ? std::stoi(it->second) : def;
}
bool has_key(const std::unordered_map<std::string, std::string> & p, const std::string & key) {
  return p.find(key) != p.end();
}
std::string hw_param(const std::unordered_map<std::string, std::string> & p,
                     const std::string & key, const std::string & def) {
  auto it = p.find(key);
  return it != p.end() ? it->second : def;
}
}  // namespace

hardware_interface::CallbackReturn OmxDynamixelSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // hardware_parameters (URDF <param>) - 튜닝/설정값
  port_ = hw_param(info_.hardware_parameters, "port_name", port_);
  baud_ = std::stoi(hw_param(info_.hardware_parameters, "baud_rate", std::to_string(baud_)));
  diag_publish_decimation_ = std::stoi(
    hw_param(info_.hardware_parameters, "diagnostics_publish_decimation",
             std::to_string(diag_publish_decimation_)));
  if (diag_publish_decimation_ < 1) diag_publish_decimation_ = 1;
  diag_read_decimation_ = std::stoi(
    hw_param(info_.hardware_parameters, "diagnostics_read_decimation",
             std::to_string(diag_read_decimation_)));
  if (diag_read_decimation_ < 1) diag_read_decimation_ = 1;
  read_error_max_cycles_ = std::stoi(
    hw_param(info_.hardware_parameters, "read_error_max_cycles",
             std::to_string(read_error_max_cycles_)));
  if (read_error_max_cycles_ < 1) read_error_max_cycles_ = 1;
  disable_torque_at_deactivate_ =
    hw_param(info_.hardware_parameters, "disable_torque_at_deactivate", "true") == "true";

  // joint <-> gpio 1:1 매핑 (이 로봇은 선언 순서로 정렬: joint1..gripper <-> dxl11..dxl16)
  if (info_.gpios.size() != info_.joints.size()) {
    RCLCPP_ERROR(
      get_logger(), "joints(%zu) 와 gpios(%zu) 개수가 다릅니다. 커스텀 HW 는 1:1 매핑을 요구합니다.",
      info_.joints.size(), info_.gpios.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joints_.clear();
  ids_.clear();
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & j = info_.joints[i];
    const auto & g = info_.gpios[i].parameters;
    JointConfig jc;
    jc.name = j.name;
    jc.id = static_cast<uint8_t>(param_int(g, "ID", 0));
    jc.operating_mode = param_int(g, "Operating Mode", 3);
    jc.position_p = param_int(g, "Position P Gain", 0);
    jc.position_i = param_int(g, "Position I Gain", 0);
    jc.position_d = param_int(g, "Position D Gain", 0);
    jc.profile_velocity = param_int(g, "Profile Velocity", 0);
    jc.profile_acceleration = param_int(g, "Profile Acceleration", 0);
    jc.has_position_limit = has_key(g, "Min Position Limit") && has_key(g, "Max Position Limit");
    jc.min_position_limit = param_int(g, "Min Position Limit", 0);
    jc.max_position_limit = param_int(g, "Max Position Limit", 0);
    jc.has_current = has_key(g, "Goal Current");
    jc.goal_current = param_int(g, "Goal Current", 0);
    jc.current_limit = param_int(g, "Current Limit", 0);
    // gripper 는 effort state interface 를 선언했는지로 판별
    jc.expose_effort = std::any_of(
      j.state_interfaces.begin(), j.state_interfaces.end(),
      [](const hardware_interface::InterfaceInfo & si) {
        return si.name == hardware_interface::HW_IF_EFFORT;
      });

    if (jc.id == 0) {
      RCLCPP_ERROR(get_logger(), "joint '%s' 에 대응하는 gpio 의 ID 파라미터가 없습니다.",
                   jc.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "map joint '%s' <-> dxl ID %u (mode %d%s)", jc.name.c_str(), jc.id,
                jc.operating_mode, jc.expose_effort ? ", effort" : "");
    joints_.push_back(jc);
    ids_.push_back(jc.id);
  }
  health_.assign(joints_.size(), MotorHealth{});

  RCLCPP_INFO(get_logger(), "OmxDynamixelSystem on_init: port=%s baud=%d joints=%zu", port_.c_str(),
              baud_, joints_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmxDynamixelSystem::on_configure(
  const rclcpp_lifecycle::State &) {
  stop_service_thread();  // 재구성 시 기존 스레드가 옛 bus_ 를 만지지 않도록 먼저 정리
  bus_ = std::make_unique<DxlBus>(port_, baud_);
  if (!bus_->open()) {
    RCLCPP_ERROR(get_logger(), "포트 open 실패: %s @ %d", port_.c_str(), baud_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  // 6 모터 ping 확인 (실패 시 명시적 ERROR)
  for (const auto & jc : joints_) {
    uint16_t model = 0;
    std::string err;
    if (!bus_->ping(jc.id, model, err)) {
      RCLCPP_ERROR(get_logger(), "ping 실패 ID %u (%s): %s", jc.id, jc.name.c_str(), err.c_str());
      bus_->close();
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "ping OK ID %u model=%u", jc.id, model);
  }

  // 별도 노드: 진단 publish + 운영 서비스
  diag_node_ = std::make_shared<rclcpp::Node>("omx_dxl_hw_diag");
  auto pub = diag_node_->create_publisher<omx_interfaces::msg::DynamixelDiagnostics>(
    "~/dynamixel_diagnostics", rclcpp::QoS(10));
  diag_pub_ = std::make_shared<
    realtime_tools::RealtimePublisher<omx_interfaces::msg::DynamixelDiagnostics>>(pub);

  set_torque_srv_ = diag_node_->create_service<std_srvs::srv::SetBool>(
    kSetTorqueService,
    std::bind(&OmxDynamixelSystem::on_set_torque, this, std::placeholders::_1,
              std::placeholders::_2));
  reboot_srv_ = diag_node_->create_service<omx_interfaces::srv::RebootDxl>(
    kRebootService,
    std::bind(&OmxDynamixelSystem::on_reboot, this, std::placeholders::_1,
              std::placeholders::_2));

  start_service_thread();
  RCLCPP_INFO(get_logger(), "운영 서비스 준비: %s/set_torque, %s/reboot",
              diag_node_->get_name(), diag_node_->get_name());
  return hardware_interface::CallbackReturn::SUCCESS;
}

void OmxDynamixelSystem::start_service_thread() {
  if (svc_running_.exchange(true)) return;
  svc_exec_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  svc_exec_->add_node(diag_node_);
  svc_thread_ = std::thread([this]() { svc_exec_->spin(); });
}

void OmxDynamixelSystem::stop_service_thread() {
  if (!svc_running_.exchange(false)) return;
  if (svc_exec_) svc_exec_->cancel();
  if (svc_thread_.joinable()) svc_thread_.join();
  svc_exec_.reset();
}

OmxDynamixelSystem::~OmxDynamixelSystem() { stop_service_thread(); }

void OmxDynamixelSystem::on_set_torque(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
  std::lock_guard<std::mutex> lock(bus_mtx_);  // RT read/write 와 버스 직렬화
  if (!bus_) { res->success = false; res->message = "bus not open"; return; }
  bool all_ok = true;
  for (const auto & jc : joints_) {
    all_ok &= bus_->write1(jc.id, reg::TORQUE_ENABLE, req->data ? 1 : 0);
  }
  res->success = all_ok;
  res->message = std::string("torque ") + (req->data ? "enable" : "disable") +
                 (all_ok ? " OK" : " 일부 실패");
  RCLCPP_WARN(get_logger(), "[service] %s", res->message.c_str());
}

void OmxDynamixelSystem::on_reboot(
  const std::shared_ptr<omx_interfaces::srv::RebootDxl::Request> req,
  std::shared_ptr<omx_interfaces::srv::RebootDxl::Response> res) {
  std::lock_guard<std::mutex> lock(bus_mtx_);
  if (!bus_) { res->success = false; res->message = "bus not open"; return; }
  bool all_ok = true;
  std::string targets;
  for (const auto & jc : joints_) {
    if (req->id != 0 && jc.id != req->id) continue;
    const bool ok = bus_->reboot(jc.id);
    all_ok &= ok;
    targets += " " + std::to_string(jc.id) + (ok ? "(ok)" : "(fail)");
  }
  if (targets.empty()) { res->success = false; res->message = "no matching id"; return; }
  res->success = all_ok;
  res->message = "reboot:" + targets;
  RCLCPP_WARN(get_logger(), "[service] %s", res->message.c_str());
}

bool OmxDynamixelSystem::apply_servo_config(const JointConfig & jc) {
  bool ok = true;
  // operating mode / current limit 는 안전 직결 - 실패하면 false (on_activate 가 ERROR 처리).
  // operating mode 변경은 torque off 상태에서만 가능.
  ok &= bus_->write1(jc.id, reg::OPERATING_MODE, static_cast<uint8_t>(jc.operating_mode));
  // PID/profile 은 gain 손실 가능성을 경고로 노출(치명적이진 않으나 침묵 금지).
  if (!bus_->write2(jc.id, reg::POSITION_P_GAIN, static_cast<uint16_t>(jc.position_p)) ||
      !bus_->write2(jc.id, reg::POSITION_I_GAIN, static_cast<uint16_t>(jc.position_i)) ||
      !bus_->write2(jc.id, reg::POSITION_D_GAIN, static_cast<uint16_t>(jc.position_d))) {
    RCLCPP_WARN(get_logger(), "PID gain write 일부 실패 ID %u - 모터 기본 gain 으로 동작할 수 있음", jc.id);
  }
  if (jc.profile_velocity > 0)
    bus_->write4(jc.id, reg::PROFILE_VELOCITY, static_cast<uint32_t>(jc.profile_velocity));
  if (jc.profile_acceleration > 0)
    bus_->write4(jc.id, reg::PROFILE_ACCELERATION, static_cast<uint32_t>(jc.profile_acceleration));
  if (jc.has_current) {
    if (jc.current_limit > 0)
      ok &= bus_->write2(jc.id, reg::CURRENT_LIMIT, static_cast<uint16_t>(jc.current_limit));
    ok &= bus_->write2(jc.id, reg::GOAL_CURRENT, static_cast<uint16_t>(jc.goal_current));
  }
  return ok;
}

hardware_interface::CallbackReturn OmxDynamixelSystem::on_activate(
  const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(bus_mtx_);  // 서비스 스레드와 버스 직렬화
  // 1) torque disable -> config write -> torque enable
  for (const auto & jc : joints_) {
    if (!bus_->write1(jc.id, reg::TORQUE_ENABLE, 0)) {
      RCLCPP_ERROR(get_logger(), "torque disable 실패 ID %u", jc.id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!apply_servo_config(jc)) {
      RCLCPP_ERROR(get_logger(),
                   "servo config write 실패 ID %u (operating mode/current limit) - 활성화 중단", jc.id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!bus_->write1(jc.id, reg::TORQUE_ENABLE, 1)) {
      RCLCPP_ERROR(get_logger(), "torque enable 실패 ID %u", jc.id);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (!bus_->setup_groups(ids_)) {
    RCLCPP_ERROR(get_logger(), "GroupSyncRead/Write 파라미터 등록 실패");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 2) 현재 위치 1회 read -> command 를 현재 위치로 초기화(활성화 시 점프 방지)
  std::map<uint8_t, PresentState> states;
  if (bus_->sync_read_present_states(ids_, states)) {
    for (const auto & jc : joints_) {
      auto it = states.find(jc.id);
      if (it == states.end()) continue;
      const double rad = tick_to_rad(static_cast<int>(it->second.position));
      set_state(jc.name + "/" + hardware_interface::HW_IF_POSITION, rad);
      set_command(jc.name + "/" + hardware_interface::HW_IF_POSITION, rad);
    }
  } else {
    RCLCPP_WARN(get_logger(), "on_activate: 초기 위치 read 실패 - command 초기화 생략");
  }

  RCLCPP_INFO(get_logger(), "OmxDynamixelSystem activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmxDynamixelSystem::on_deactivate(
  const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(bus_mtx_);
  if (disable_torque_at_deactivate_) {
    for (const auto & jc : joints_) {
      bus_->write1(jc.id, reg::TORQUE_ENABLE, 0);
    }
    RCLCPP_INFO(get_logger(), "OmxDynamixelSystem deactivated (torque off)");
  } else {
    RCLCPP_INFO(get_logger(), "OmxDynamixelSystem deactivated (torque 유지)");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OmxDynamixelSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/) {
  ++read_cycle_;
  std::lock_guard<std::mutex> lock(bus_mtx_);  // 서비스 스레드와 버스 직렬화 (RT 경로, ~1-2ms)

  // 1) position/velocity/current 일괄 GroupSyncRead
  std::map<uint8_t, PresentState> states;
  bus_->sync_read_present_states(ids_, states);
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto & jc = joints_[i];
    auto it = states.find(jc.id);
    if (it == states.end()) {
      health_[i].comm_ok = false;  // 개별 모터 드롭아웃 -> watchdog 로 위임
      continue;
    }
    health_[i].comm_ok = true;
    const double rad = tick_to_rad(static_cast<int>(it->second.position));
    const double radps =
      velocity_unit_to_rad_per_s(decode_present_velocity(it->second.velocity));
    set_state(jc.name + "/" + hardware_interface::HW_IF_POSITION, rad);
    set_state(jc.name + "/" + hardware_interface::HW_IF_VELOCITY, radps);
    if (jc.expose_effort) {
      const double ma = current_unit_to_ma(decode_present_current(it->second.current));
      health_[i].present_current_ma = ma;
      set_state(jc.name + "/" + hardware_interface::HW_IF_EFFORT, ma);
    }
  }

  // 결함1 수정: 2단계 실패 처리.
  //  - 개별 모터 드롭아웃: 위에서 comm_ok=false -> 진단/watchdog 가 컨트롤러 deactivate
  //  - 버스 전면 실패(한 모터도 못 읽음)가 연속 N 사이클: resource manager 에 ERROR 에스컬레이션
  if (states.empty()) {
    if (++read_fail_streak_ >= read_error_max_cycles_) {
      RCLCPP_ERROR(get_logger(),
                   "버스 read 가 %d 사이클 연속 전면 실패 - HW ERROR 에스컬레이션", read_fail_streak_);
      return hardware_interface::return_type::ERROR;
    }
  } else {
    read_fail_streak_ = 0;
  }

  // 결함3 수정: 진단(느린 개별 레지스터)은 매 사이클이 아니라 decimation 주기로만 read.
  if (!joints_.empty() && (read_cycle_ % diag_read_decimation_) == 0) {
    const size_t idx = diag_rr_index_ % joints_.size();
    const uint8_t id = joints_[idx].id;
    uint8_t temp = 0, hwerr = 0;
    uint16_t volt = 0;
    bool dok = true;
    dok &= bus_->read1(id, reg::PRESENT_TEMPERATURE, temp);
    dok &= bus_->read2(id, reg::PRESENT_INPUT_VOLTAGE, volt);
    dok &= bus_->read1(id, reg::HARDWARE_ERROR_STATUS, hwerr);
    if (dok) {
      health_[idx].temperature_c = static_cast<int>(temp);
      health_[idx].input_voltage_v = voltage_unit_to_v(static_cast<int>(volt));
      health_[idx].hardware_error_status = hwerr;
    }
    diag_rr_index_ = (diag_rr_index_ + 1) % joints_.size();
  }

  // decimation 주기로 진단 발행
  if ((read_cycle_ % diag_publish_decimation_) == 0) {
    publish_diagnostics(time);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OmxDynamixelSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  std::map<uint8_t, uint32_t> goals;
  for (const auto & jc : joints_) {
    const double rad = get_command(jc.name + "/" + hardware_interface::HW_IF_POSITION);
    if (std::isnan(rad)) continue;  // 아직 명령 미설정
    int tick = rad_to_tick(rad);
    if (jc.has_position_limit) {
      tick = std::clamp(tick, jc.min_position_limit, jc.max_position_limit);
    }
    goals[jc.id] = static_cast<uint32_t>(tick);
  }
  std::lock_guard<std::mutex> lock(bus_mtx_);  // 서비스 스레드와 버스 직렬화
  if (!goals.empty() && !bus_->sync_write_goal_position(goals)) {
    // 죽은 버스의 에스컬레이션은 read() 가 담당. 여기선 침묵하지 않게 throttled 경고만.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "GroupSyncWrite goal position 실패");
  }
  return hardware_interface::return_type::OK;
}

void OmxDynamixelSystem::publish_diagnostics(const rclcpp::Time & time) {
  if (!diag_pub_ || !diag_pub_->trylock()) return;
  auto & m = diag_pub_->msg_;
  m.header.stamp = time;
  const size_t n = joints_.size();
  m.ids.resize(n);
  m.present_current_ma.resize(n);
  m.temperature_c.resize(n);
  m.input_voltage_v.resize(n);
  m.hardware_error_status.resize(n);
  m.comm_ok.resize(n);
  for (size_t i = 0; i < n; ++i) {
    m.ids[i] = joints_[i].id;
    m.present_current_ma[i] = health_[i].present_current_ma;
    m.temperature_c[i] = health_[i].temperature_c;
    m.input_voltage_v[i] = health_[i].input_voltage_v;
    m.hardware_error_status[i] = health_[i].hardware_error_status;
    m.comm_ok[i] = health_[i].comm_ok;
  }
  diag_pub_->unlockAndPublish();
}

}  // namespace omx_dynamixel_hw

PLUGINLIB_EXPORT_CLASS(omx_dynamixel_hw::OmxDynamixelSystem, hardware_interface::SystemInterface)
