// omx_motion_server — motion_server.cpp
//
// Action servers:
//   /omx/move_to_named   (omx_interfaces/action/MoveToNamed)
//   /omx/move_to_pose    (omx_interfaces/action/MoveToPose)
//   /omx/move_to_joints  (omx_interfaces/action/MoveToJoints)
//   /omx/gripper_command (omx_interfaces/action/GripperCommand)
//
// 네 액션 모두 MoveGroupInterface 로 제어한다.
//   arm     그룹: MoveToNamed, MoveToPose, MoveToJoints
//   gripper 그룹: GripperCommand  ← GripperActionController 직접 호출 대신
//                                    MoveIt trajectory 로 속도 제한 적용
//
// 동시성 정책:
//   arm_busy_ (atomic<bool>) — MoveToNamed / MoveToPose / MoveToJoints 가 실행 중이면 true.
//     handle_*_goal 에서 CAS 로 선점하고 execute_* 소멸 시 RAII 로 해제한다.
//     결과적으로 arm_group_ 의 setNamedTarget / setPoseTarget /
//     setMaxVelocityScalingFactor 호출이 직렬화된다.
//   gripper_busy_ — GripperCommand 에 동일하게 적용.
//   arm_thread_ / gripper_thread_ — detach 대신 명시적 join 으로 수명 관리.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <filesystem>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <builtin_interfaces/msg/duration.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <omx_interfaces/action/move_to_named.hpp>
#include <omx_interfaces/action/move_to_pose.hpp>
#include <omx_interfaces/action/move_to_joints.hpp>
#include <omx_interfaces/action/gripper_command.hpp>
#include <omx_interfaces/srv/clear_plan.hpp>
#include <omx_interfaces/srv/execute_plan.hpp>
#include <omx_interfaces/srv/plan_to_joints.hpp>

// ── 워크스페이스 한계 (workspace_guard 와 일치) ───────────────────────────
static constexpr double WS_Z_MIN      = 0.01;
static constexpr double WS_Z_MAX      = 0.48;
static constexpr double WS_XY_RADIUS  = 0.42;

// ── 그리퍼 조인트 위치 범위 (SRDF: close=0, open=1) ──────────────────────
static constexpr double GRIPPER_CLOSE_POS = 0.0;
static constexpr double GRIPPER_OPEN_POS  = 1.0;

// ── 기본 velocity / acceleration scaling ─────────────────────────────────
static constexpr double DEFAULT_VEL_SCALE = 0.3;
static constexpr double DEFAULT_ACC_SCALE = 0.1;
static constexpr double PREVIEW_SYNC_TIME_SEC = 1.0;
static constexpr int PREVIEW_DOMAIN_ID = 90;
static constexpr double PREVIEW_READY_TIMEOUT_SEC = 45.0;
static constexpr double PREVIEW_ACCEPT_TIMEOUT_SEC = 20.0;
static constexpr const char * PREVIEW_RUNTIME_DIR = "/tmp/omx_preview_runtime";

// ── quaternion 정규화 허용 오차 ───────────────────────────────────────────
static constexpr double QUAT_NORM_TOL = 0.01;

// ── MoveItErrorCode → 사람이 읽을 수 있는 이름 ───────────────────────────
// execute() 실패 원인을 action result message 에 담을 때 사용.
static std::string moveit_error_name(const moveit::core::MoveItErrorCode & code)
{
  using C = moveit_msgs::msg::MoveItErrorCodes;
  switch (code.val) {
    case C::SUCCESS: return "SUCCESS";
    case C::FAILURE: return "FAILURE";
    case C::PLANNING_FAILED: return "PLANNING_FAILED";
    case C::INVALID_MOTION_PLAN: return "INVALID_MOTION_PLAN";
    case C::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case C::CONTROL_FAILED: return "CONTROL_FAILED";
    case C::UNABLE_TO_AQUIRE_SENSOR_DATA: return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case C::TIMED_OUT: return "TIMED_OUT";
    case C::PREEMPTED: return "PREEMPTED";
    case C::START_STATE_IN_COLLISION: return "START_STATE_IN_COLLISION";
    case C::START_STATE_VIOLATES_PATH_CONSTRAINTS:
      return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case C::GOAL_IN_COLLISION: return "GOAL_IN_COLLISION";
    case C::GOAL_VIOLATES_PATH_CONSTRAINTS: return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case C::GOAL_CONSTRAINTS_VIOLATED: return "GOAL_CONSTRAINTS_VIOLATED";
    case C::INVALID_GROUP_NAME: return "INVALID_GROUP_NAME";
    case C::INVALID_GOAL_CONSTRAINTS: return "INVALID_GOAL_CONSTRAINTS";
    case C::INVALID_ROBOT_STATE: return "INVALID_ROBOT_STATE";
    case C::INVALID_LINK_NAME: return "INVALID_LINK_NAME";
    case C::INVALID_OBJECT_NAME: return "INVALID_OBJECT_NAME";
    case C::FRAME_TRANSFORM_FAILURE: return "FRAME_TRANSFORM_FAILURE";
    case C::COLLISION_CHECKING_UNAVAILABLE: return "COLLISION_CHECKING_UNAVAILABLE";
    case C::ROBOT_STATE_STALE: return "ROBOT_STATE_STALE";
    case C::SENSOR_INFO_STALE: return "SENSOR_INFO_STALE";
    case C::NO_IK_SOLUTION: return "NO_IK_SOLUTION";
    default: return "UNKNOWN";
  }
}

static std::string execute_failure_message(
  const std::string & prefix,
  const moveit::core::MoveItErrorCode & code)
{
  return prefix + " (code=" + moveit_error_name(code)
       + ", val=" + std::to_string(code.val) + ")";
}

// ── named pose → SRDF group_state 매핑 ───────────────────────────────────
// SRDF(omx_f.srdf) 에 등록된 arm 그룹 named state 목록:
//   "init" : 모든 조인트 0
//   "home" : joint2=-1.57, joint3=1.57, joint4=1.57
static const std::unordered_map<std::string, std::string> NAMED_POSE_MAP = {
  {"home",      "home"},
  {"init",      "init"},
  {"ready",     "home"},   // home 자세를 ready 로 사용
  {"stow",      "init"},   // 모든 조인트 0 → 안전 접힘
  {"pre_grasp", "home"},   // 그리핑 전 준비 → home 근사값
};


class MotionServer : public rclcpp::Node
{
public:
  using MoveToNamed  = omx_interfaces::action::MoveToNamed;
  using MoveToPose   = omx_interfaces::action::MoveToPose;
  using MoveToJoints = omx_interfaces::action::MoveToJoints;
  using GripperCmd   = omx_interfaces::action::GripperCommand;
  using PlanToJoints = omx_interfaces::srv::PlanToJoints;
  using ExecutePlan  = omx_interfaces::srv::ExecutePlan;
  using ClearPlan    = omx_interfaces::srv::ClearPlan;

  explicit MotionServer(const rclcpp::NodeOptions & options)
  : Node("motion_server", options), node_options_(options)
  {
    declare_parameter<std::string>("arm_tip_link", "end_effector_link");
    // Workspace 안전 제한 (world frame 기준).
    // EE 가 x,y 방향으로 멀리 뻗으면 dxl12 (shoulder) 의 중력 모멘트가 급증해
    // Overload Hardware Error 로 모터가 차단된다. 그 영역을 사전에 거부한다.
    declare_parameter<double>("workspace.x_abs_max", 0.26);
    declare_parameter<double>("workspace.y_abs_max", 0.26);
    declare_parameter<double>("workspace.z_min",     0.0);
    declare_parameter<double>("workspace.z_max",     0.45);
    declare_parameter<std::string>("workspace.frame", "world");
    // 액션 서버들은 Reentrant callback group 에 배치한다.
    // arm_busy_ / gripper_busy_ 로 실제 직렬 실행을 보장하므로
    // MoveGroupInterface 의 공유 상태 경쟁을 방지한다.
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    move_to_named_server_ = rclcpp_action::create_server<MoveToNamed>(
      this, "/omx/move_to_named",
      std::bind(&MotionServer::handle_named_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MotionServer::handle_named_cancel,   this, std::placeholders::_1),
      std::bind(&MotionServer::handle_named_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), cb_group_);

    move_to_pose_server_ = rclcpp_action::create_server<MoveToPose>(
      this, "/omx/move_to_pose",
      std::bind(&MotionServer::handle_pose_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MotionServer::handle_pose_cancel,   this, std::placeholders::_1),
      std::bind(&MotionServer::handle_pose_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), cb_group_);

    move_to_joints_server_ = rclcpp_action::create_server<MoveToJoints>(
      this, "/omx/move_to_joints",
      std::bind(&MotionServer::handle_joints_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MotionServer::handle_joints_cancel,   this, std::placeholders::_1),
      std::bind(&MotionServer::handle_joints_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), cb_group_);

    gripper_server_ = rclcpp_action::create_server<GripperCmd>(
      this, "/omx/gripper_command",
      std::bind(&MotionServer::handle_gripper_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MotionServer::handle_gripper_cancel,   this, std::placeholders::_1),
      std::bind(&MotionServer::handle_gripper_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), cb_group_);

    plan_to_joints_service_ = create_service<PlanToJoints>(
      "/omx/plan_to_joints",
      std::bind(
        &MotionServer::handle_plan_to_joints,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default,
      cb_group_);

    execute_plan_service_ = create_service<ExecutePlan>(
      "/omx/execute_plan",
      std::bind(
        &MotionServer::handle_execute_plan,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default,
      cb_group_);

    clear_plan_service_ = create_service<ClearPlan>(
      "/omx/clear_plan",
      std::bind(
        &MotionServer::handle_clear_plan,
        this,
        std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default,
      cb_group_);

    RCLCPP_INFO(get_logger(), "MotionServer: action servers and plan services created, waiting for MoveIt...");
  }

  ~MotionServer()
  {
    // 노드 종료 시 실행 중인 스레드를 안전하게 join — detach 없음
    if (arm_thread_.joinable())     arm_thread_.join();
    if (gripper_thread_.joinable()) gripper_thread_.join();
  }

  // MoveGroupInterface 는 executor 가 spin 을 시작한 뒤 초기화해야 한다.
  void init_moveit()
  {
    if (!moveit_node_) {
      create_moveit_helper_node();
    }

    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      moveit_node_, "arm");
    arm_group_->setMaxVelocityScalingFactor(DEFAULT_VEL_SCALE);
    arm_group_->setMaxAccelerationScalingFactor(DEFAULT_ACC_SCALE);
    arm_group_->setPlanningTime(5.0);
    // Goal tolerance: hardware encoder resolution(약 0.088°) + TRAC-IK Distance 해상도 고려.
    // 1 mm / 1 mrad 수준으로 명시화해 default(보통 1cm) 의 느슨한 도착 판정을 차단.
    arm_group_->setGoalPositionTolerance(0.001);     // 1 mm
    arm_group_->setGoalOrientationTolerance(0.01);   // ~0.57°, position_only_ik 라 큰 영향 없음
    arm_group_->setGoalJointTolerance(0.001);        // ~0.057°, encoder 1 step 보다 작음

    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      moveit_node_, "gripper");
    gripper_group_->setMaxVelocityScalingFactor(DEFAULT_VEL_SCALE);
    gripper_group_->setMaxAccelerationScalingFactor(DEFAULT_ACC_SCALE);
    gripper_group_->setPlanningTime(3.0);
    gripper_group_->setGoalJointTolerance(0.001);

    arm_tip_link_ = get_parameter("arm_tip_link").as_string();
    if (arm_tip_link_.empty()) {
      arm_tip_link_ = "end_effector_link";
    }

    ws_x_abs_max_ = get_parameter("workspace.x_abs_max").as_double();
    ws_y_abs_max_ = get_parameter("workspace.y_abs_max").as_double();
    ws_z_min_     = get_parameter("workspace.z_min").as_double();
    ws_z_max_     = get_parameter("workspace.z_max").as_double();
    ws_frame_     = get_parameter("workspace.frame").as_string();
    RCLCPP_INFO(
      get_logger(),
      "Workspace limit (frame='%s'): |x|<=%.3f, |y|<=%.3f, %.3f<=z<=%.3f",
      ws_frame_.c_str(), ws_x_abs_max_, ws_y_abs_max_, ws_z_min_, ws_z_max_);

    const std::string arm_planning_frame = arm_group_->getPlanningFrame();
    const std::string arm_pose_frame = arm_group_->getPoseReferenceFrame();
    const std::string arm_eef_link = arm_group_->getEndEffectorLink();
    const std::string gripper_planning_frame = gripper_group_->getPlanningFrame();
    const std::string gripper_pose_frame = gripper_group_->getPoseReferenceFrame();
    const std::string gripper_eef_link = gripper_group_->getEndEffectorLink();

    RCLCPP_INFO(
      get_logger(),
      "MoveIt arm group ready: planning_frame='%s' pose_reference_frame='%s' end_effector_link='%s'",
      arm_planning_frame.c_str(),
      arm_pose_frame.c_str(),
      arm_eef_link.empty() ? "<empty>" : arm_eef_link.c_str());
    RCLCPP_INFO(
      get_logger(),
      "MoveIt gripper group ready: planning_frame='%s' pose_reference_frame='%s' end_effector_link='%s'",
      gripper_planning_frame.c_str(),
      gripper_pose_frame.c_str(),
      gripper_eef_link.empty() ? "<empty>" : gripper_eef_link.c_str());
    RCLCPP_INFO(
      get_logger(),
      "MotionServer arm tip link override: '%s'",
      arm_tip_link_.c_str());
  }

  void create_moveit_helper_node()
  {
    if (moveit_node_) {
      return;
    }

    auto helper_options = node_options_;
    helper_options.use_global_arguments(false);
    helper_options.arguments({});
    helper_options.automatically_declare_parameters_from_overrides(true);

    const auto & overrides =
      this->get_node_parameters_interface()->get_parameter_overrides();
    for (const auto & [name, value] : overrides) {
      helper_options.append_parameter_override(rclcpp::Parameter(name, value));
    }

    moveit_node_ = std::make_shared<rclcpp::Node>(
      "motion_server_moveit", helper_options);

    RCLCPP_INFO(
      get_logger(),
      "Created dedicated MoveIt helper node: '%s'",
      moveit_node_->get_fully_qualified_name());
  }

  rclcpp::Node::SharedPtr get_moveit_helper_node() const
  {
    return moveit_node_;
  }

  // MoveGroupInterface 연결 완료 여부 — main 의 readiness 폴링에 사용
  bool is_moveit_ready() const
  {
    return arm_group_     && !arm_group_->getPlanningFrame().empty()
        && gripper_group_ && !gripper_group_->getPlanningFrame().empty();
  }

  // execute 직전/직후 조인트 상태 스냅샷.
  // plan 의 start state 와 controller current state 가 drift 하면 MoveIt 은
  // CONTROL_FAILED 로 goal 을 abort 한다. 이 값을 로그에 남겨두면 원인 분석이
  // 쉬워진다.
  void log_current_arm_state(const std::string & tag) const
  {
    if (!arm_group_) {
      return;
    }
    auto state = arm_group_->getCurrentState(0.5);  // 0.5s wait
    if (!state) {
      RCLCPP_WARN(get_logger(),
        "%s: getCurrentState() returned null — TF/joint_state 누락 가능",
        tag.c_str());
      return;
    }
    std::vector<double> joints;
    state->copyJointGroupPositions("arm", joints);
    std::ostringstream oss;
    oss << tag << ": joints=[";
    for (std::size_t i = 0; i < joints.size(); ++i) {
      if (i > 0) oss << ", ";
      oss << std::fixed << std::setprecision(4) << joints[i];
    }
    oss << "]";

    try {
      const auto pose = arm_tip_link_.empty()
        ? arm_group_->getCurrentPose()
        : arm_group_->getCurrentPose(arm_tip_link_);
      const auto & p = pose.pose.position;
      const auto & q = pose.pose.orientation;
      oss << " pose=(" << std::fixed << std::setprecision(4)
          << p.x << ", " << p.y << ", " << p.z << ")"
          << " q=(" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << ")";
    } catch (const std::exception & e) {
      oss << " pose=<err: " << e.what() << ">";
    }
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }

private:
  // ── 멤버 ───────────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::NodeOptions node_options_;
  rclcpp::Node::SharedPtr moveit_node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  rclcpp_action::Server<MoveToNamed>::SharedPtr  move_to_named_server_;
  rclcpp_action::Server<MoveToPose>::SharedPtr   move_to_pose_server_;
  rclcpp_action::Server<MoveToJoints>::SharedPtr move_to_joints_server_;
  rclcpp_action::Server<GripperCmd>::SharedPtr   gripper_server_;
  rclcpp::Service<PlanToJoints>::SharedPtr plan_to_joints_service_;
  rclcpp::Service<ExecutePlan>::SharedPtr execute_plan_service_;
  rclcpp::Service<ClearPlan>::SharedPtr clear_plan_service_;

  // ── 동시성 제어 ────────────────────────────────────────────────────────
  // busy 플래그는 handle_*_goal 에서 CAS 로 설정되고 execute_* 의
  // BusyGuard 소멸자에서 해제된다 (모든 return 경로 포함).
  std::atomic<bool> arm_busy_{false};
  std::atomic<bool> gripper_busy_{false};
  std::thread arm_thread_;      // detach 대신 handle_*_accepted 에서 join 후 교체
  std::thread gripper_thread_;
  std::string arm_tip_link_;
  // workspace 제한 (world frame 기준)
  double ws_x_abs_max_{0.26};
  double ws_y_abs_max_{0.26};
  double ws_z_min_{0.0};
  double ws_z_max_{0.45};
  std::string ws_frame_{"world"};
  std::mutex cached_plan_mutex_;
  moveit::planning_interface::MoveGroupInterface::Plan cached_arm_plan_;
  std::string cached_plan_id_;
  bool cached_plan_valid_{false};


  // ── RAII 헬퍼 ─────────────────────────────────────────────────────────

  // atomic 플래그를 스코프 종료 시 false 로 해제한다.
  struct BusyGuard {
    std::atomic<bool> & flag;
    ~BusyGuard() { flag.store(false, std::memory_order_release); }
  };

  // velocity scaling 을 스코프 종료 시 기본값으로 복원한다.
  // grp 가 nullptr 이면 복원을 건너뛴다.
  struct VelScaleGuard {
    moveit::planning_interface::MoveGroupInterface * grp;
    ~VelScaleGuard() {
      if (grp) grp->setMaxVelocityScalingFactor(DEFAULT_VEL_SCALE);
    }
  };

  static double duration_to_sec(const builtin_interfaces::msg::Duration & duration)
  {
    return static_cast<double>(duration.sec)
      + (static_cast<double>(duration.nanosec) * 1e-9);
  }

  static builtin_interfaces::msg::Duration sec_to_duration(double seconds)
  {
    builtin_interfaces::msg::Duration duration;
    if (seconds < 0.0) {
      seconds = 0.0;
    }

    duration.sec = static_cast<int32_t>(std::floor(seconds));
    duration.nanosec = static_cast<uint32_t>(
      std::llround((seconds - static_cast<double>(duration.sec)) * 1e9));

    if (duration.nanosec >= 1000000000u) {
      duration.sec += 1;
      duration.nanosec -= 1000000000u;
    }
    return duration;
  }

  trajectory_msgs::msg::JointTrajectory build_preview_trajectory(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan) const
  {
    auto preview = plan.trajectory.joint_trajectory;
    if (preview.joint_names.empty()) {
      preview.joint_names = arm_group_->getActiveJoints();
    }

    auto current_state = arm_group_->getCurrentState(1.0);
    if (!current_state) {
      return preview;
    }

    std::vector<double> current_positions;
    current_state->copyJointGroupPositions("arm", current_positions);
    if (!current_positions.empty() && current_positions.size() == preview.joint_names.size()) {
      trajectory_msgs::msg::JointTrajectoryPoint start_point;
      start_point.positions = current_positions;
      start_point.velocities.assign(current_positions.size(), 0.0);
      start_point.accelerations.assign(current_positions.size(), 0.0);
      start_point.time_from_start = sec_to_duration(0.0);

      for (auto & point : preview.points) {
        point.time_from_start = sec_to_duration(
          duration_to_sec(point.time_from_start) + PREVIEW_SYNC_TIME_SEC);
      }
      preview.points.insert(preview.points.begin(), start_point);
    }

    return preview;
  }

  std::string write_preview_trajectory_file(
    const trajectory_msgs::msg::JointTrajectory & trajectory) const
  {
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const std::string path = "/tmp/omx_preview_trajectory_" + std::to_string(now_ns) + ".json";

    std::ofstream out(path, std::ios::trunc);
    if (!out.is_open()) {
      return "";
    }

    out << "{\n";
    out << "  \"joint_names\": [";
    for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i) {
      if (i > 0) out << ", ";
      out << '"' << trajectory.joint_names[i] << '"';
    }
    out << "],\n";
    out << "  \"points\": [\n";
    for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto & point = trajectory.points[i];
      out << "    {\n";

      auto dump_vector = [&out](const char * key, const std::vector<double> & values) {
        out << "      \"" << key << "\": [";
        for (std::size_t j = 0; j < values.size(); ++j) {
          if (j > 0) out << ", ";
          out << values[j];
        }
        out << "]";
      };

      dump_vector("positions", point.positions);
      out << ",\n";
      dump_vector("velocities", point.velocities);
      out << ",\n";
      dump_vector("accelerations", point.accelerations);
      out << ",\n";
      out << "      \"time_from_start_sec\": " << duration_to_sec(point.time_from_start) << '\n';
      out << "    }";
      if (i + 1 < trajectory.points.size()) {
        out << ',';
      }
      out << '\n';
    }
    out << "  ]\n";
    out << "}\n";
    return path;
  }

  long long next_preview_request_id() const
  {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  }

  static std::filesystem::path preview_runtime_path(const std::string & filename)
  {
    return std::filesystem::path(PREVIEW_RUNTIME_DIR) / filename;
  }

  static std::string read_file_trimmed(const std::filesystem::path & path)
  {
    std::ifstream in(path);
    if (!in.is_open()) {
      return "";
    }
    std::ostringstream buffer;
    buffer << in.rdbuf();
    std::string text = buffer.str();
    while (!text.empty() && (text.back() == '\n' || text.back() == '\r' || text.back() == ' ')) {
      text.pop_back();
    }
    return text;
  }

  static bool process_exists(pid_t pid)
  {
    if (pid <= 0) {
      return false;
    }
    return ::kill(pid, 0) == 0;
  }

  bool preview_instance_alive() const
  {
    const std::string pid_text = read_file_trimmed(preview_runtime_path("player.pid"));
    if (pid_text.empty()) {
      return false;
    }
    try {
      const pid_t pid = static_cast<pid_t>(std::stol(pid_text));
      return process_exists(pid);
    } catch (const std::exception &) {
      return false;
    }
  }

  bool wait_for_preview_ready() const
  {
    const auto ready_path = preview_runtime_path("ready.flag");
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(PREVIEW_READY_TIMEOUT_SEC);
    while (std::chrono::steady_clock::now() < deadline) {
      if (std::filesystem::exists(ready_path)) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return false;
  }

  bool wait_for_preview_accept(long long request_id) const
  {
    const auto accepted_path = preview_runtime_path("accepted.txt");
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(PREVIEW_ACCEPT_TIMEOUT_SEC);
    while (std::chrono::steady_clock::now() < deadline) {
      const std::string text = read_file_trimmed(accepted_path);
      if (!text.empty()) {
        try {
          if (std::stoll(text) == request_id) {
            return true;
          }
        } catch (const std::exception &) {
          // ignore malformed ack and keep waiting
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return false;
  }

  bool launch_preview_gazebo(std::string * log_path) const
  {
    std::error_code ec;
    std::filesystem::create_directories(PREVIEW_RUNTIME_DIR, ec);

    const std::string resolved_log_path = preview_runtime_path("gazebo.log").string();
    if (log_path) {
      *log_path = resolved_log_path;
    }

    std::ostringstream command;
    command
      << "/bin/sh -c \"ros2 launch omx_bringup omx_preview_gazebo.launch.py "
      << "preview_domain_id:=" << PREVIEW_DOMAIN_ID << ' '
      << "runtime_dir:=" << PREVIEW_RUNTIME_DIR
      << " >" << resolved_log_path << " 2>&1 &\"";

    return std::system(command.str().c_str()) == 0;
  }

  static double plan_duration_sec(
    const moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    const auto & points = plan.trajectory.joint_trajectory.points;
    if (points.empty()) {
      return 0.0;
    }
    return duration_to_sec(points.back().time_from_start);
  }

  static std::string make_plan_id()
  {
    const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    return "moveit-plan-" + std::to_string(now_ms);
  }

  void handle_plan_to_joints(
    const std::shared_ptr<PlanToJoints::Request> request,
    std::shared_ptr<PlanToJoints::Response> response)
  {
    response->success = false;
    response->duration = 0.0;
    response->point_count = 0;

    bool expected = false;
    if (!arm_busy_.compare_exchange_strong(
          expected, true, std::memory_order_acquire, std::memory_order_relaxed)) {
      response->message = "Arm is busy";
      return;
    }
    BusyGuard busy_guard{arm_busy_};
    VelScaleGuard vel_guard{arm_group_.get()};

    if (!arm_group_) {
      response->message = "MoveGroupInterface not initialized";
      return;
    }
    if (request->positions.empty()) {
      response->message = "positions is empty";
      return;
    }
    if (!request->joint_names.empty() &&
        request->joint_names.size() != request->positions.size()) {
      response->message = "joint_names and positions size mismatch";
      return;
    }

    if (request->velocity_scale > 0.0f) {
      const double v_scale = std::clamp(static_cast<double>(request->velocity_scale), 0.01, 1.0);
      arm_group_->setMaxVelocityScalingFactor(v_scale);
    }

    std::vector<std::string> names = request->joint_names;
    if (names.empty()) {
      names = arm_group_->getActiveJoints();
      if (names.size() != request->positions.size()) {
        response->message = "positions size does not match active joints";
        return;
      }
    }

    std::map<std::string, double> target_map;
    for (std::size_t i = 0; i < names.size(); ++i) {
      target_map.emplace(names[i], request->positions[i]);
    }

    arm_group_->setStartStateToCurrentState();
    if (!arm_group_->setJointValueTarget(target_map)) {
      response->message = "Joint target out of limits or unknown joint name";
      response->invalid_joints = names;
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_code = arm_group_->plan(plan);
    if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
      response->message = execute_failure_message("Planning failed", plan_code);
      return;
    }

    const std::string plan_id = make_plan_id();
    {
      std::lock_guard<std::mutex> lock(cached_plan_mutex_);
      cached_arm_plan_ = plan;
      cached_plan_id_ = plan_id;
      cached_plan_valid_ = true;
    }

    response->success = true;
    response->message = "MoveIt trajectory planned";
    response->plan_id = plan_id;
    response->duration = plan_duration_sec(plan);
    response->point_count = static_cast<uint32_t>(plan.trajectory.joint_trajectory.points.size());
    response->trajectory_joint_names = plan.trajectory.joint_trajectory.joint_names;
    response->trajectory_times.reserve(plan.trajectory.joint_trajectory.points.size());
    response->trajectory_positions.reserve(
      plan.trajectory.joint_trajectory.points.size()
      * plan.trajectory.joint_trajectory.joint_names.size());
    for (const auto & point : plan.trajectory.joint_trajectory.points) {
      response->trajectory_times.push_back(duration_to_sec(point.time_from_start));
      for (double position : point.positions) {
        response->trajectory_positions.push_back(position);
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "PlanToJoints: cached %s, points=%u, duration=%.3fs",
      plan_id.c_str(),
      response->point_count,
      response->duration);
  }

  void handle_execute_plan(
    const std::shared_ptr<ExecutePlan::Request> request,
    std::shared_ptr<ExecutePlan::Response> response)
  {
    response->success = false;

    bool expected = false;
    if (!arm_busy_.compare_exchange_strong(
          expected, true, std::memory_order_acquire, std::memory_order_relaxed)) {
      response->message = "Arm is busy";
      return;
    }
    BusyGuard busy_guard{arm_busy_};

    if (!arm_group_) {
      response->message = "MoveGroupInterface not initialized";
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    std::string plan_id;
    {
      std::lock_guard<std::mutex> lock(cached_plan_mutex_);
      if (!cached_plan_valid_) {
        response->message = "No cached plan. Run Plan first.";
        return;
      }
      if (!request->plan_id.empty() && request->plan_id != cached_plan_id_) {
        response->message = "Cached plan id mismatch. Run Plan again.";
        response->plan_id = cached_plan_id_;
        return;
      }
      plan = cached_arm_plan_;
      plan_id = cached_plan_id_;
    }

    log_current_arm_state("ExecutePlan before execute");
    const auto exec_code = arm_group_->execute(plan);
    const bool executed = (exec_code == moveit::core::MoveItErrorCode::SUCCESS);

    response->success = executed;
    response->plan_id = plan_id;
    response->message = executed
      ? "Executed cached MoveIt plan"
      : execute_failure_message("ExecutePlan failed", exec_code);

    if (executed) {
      std::lock_guard<std::mutex> lock(cached_plan_mutex_);
      cached_plan_valid_ = false;
      cached_plan_id_.clear();
    }
  }

  void handle_clear_plan(
    const std::shared_ptr<ClearPlan::Request>,
    std::shared_ptr<ClearPlan::Response> response)
  {
    std::lock_guard<std::mutex> lock(cached_plan_mutex_);
    cached_plan_valid_ = false;
    cached_plan_id_.clear();
    response->success = true;
    response->message = "Cached plan cleared";
  }


  // ══════════════════════════════════════════════════════════════════════
  // MoveToNamed
  // ══════════════════════════════════════════════════════════════════════

  rclcpp_action::GoalResponse handle_named_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToNamed::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "MoveToNamed: requested '%s'", goal->name.c_str());

    if (NAMED_POSE_MAP.find(goal->name) == NAMED_POSE_MAP.end()) {
      RCLCPP_WARN(get_logger(), "MoveToNamed: unknown name '%s'", goal->name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    // arm 동시 실행 차단: false → true CAS 실패 시 이미 실행 중
    bool expected = false;
    if (!arm_busy_.compare_exchange_strong(
          expected, true, std::memory_order_acquire, std::memory_order_relaxed)) {
      RCLCPP_WARN(get_logger(), "MoveToNamed: arm is busy, rejecting goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_named_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToNamed>>)
  {
    RCLCPP_INFO(get_logger(), "MoveToNamed: cancel requested");
    // stop() 은 execute() 와 동시 호출 가능하도록 MoveGroupInterface 내부에서 처리된다.
    if (arm_group_) arm_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_named_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToNamed>> goal_handle)
  {
    // arm_busy_ 로 직렬화되어 있으므로 join 은 거의 즉시 반환된다.
    if (arm_thread_.joinable()) arm_thread_.join();
    arm_thread_ = std::thread([this, goal_handle]() { execute_named(goal_handle); });
  }

  void execute_named(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToNamed>> goal_handle)
  {
    BusyGuard busy_guard{arm_busy_};  // 모든 return 경로에서 arm_busy_ 해제

    auto result   = std::make_shared<MoveToNamed::Result>();
    auto feedback = std::make_shared<MoveToNamed::Feedback>();

    const std::string & requested = goal_handle->get_goal()->name;
    const std::string & srdf_name = NAMED_POSE_MAP.at(requested);

    if (!arm_group_) {
      result->success = false;
      result->message = "MoveGroupInterface not initialized";
      goal_handle->abort(result);
      return;
    }

    feedback->status = "planning";
    goal_handle->publish_feedback(feedback);

    // 현재 조인트 상태로 시작점 동기화 (stale 플랜 방지)
    arm_group_->setStartStateToCurrentState();
    arm_group_->setNamedTarget(srdf_name);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!planned) {
      result->success = false;
      result->message = "Planning failed for '" + requested + "'";
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      return;
    }

    feedback->status = "executing";
    goal_handle->publish_feedback(feedback);

    const auto exec_code = arm_group_->execute(plan);
    bool executed = (exec_code == moveit::core::MoveItErrorCode::SUCCESS);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Cancelled";
      goal_handle->canceled(result);
      return;
    }

    result->success = executed;
    result->message = executed
      ? "Moved to '" + requested + "'"
      : execute_failure_message("MoveToNamed execution failed", exec_code);
    if (executed) {
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
    } else {
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    }
  }


  // ══════════════════════════════════════════════════════════════════════
  // MoveToJoints
  //   - joint_names[] 이 비어 있으면 arm 그룹의 active joints 를 기본 사용
  //   - setJointValueTarget(std::map<std::string,double>) 로 joint-space 목표 지정
  //   - MoveIt 이 joint_limits.yaml 기반으로 범위 검증 → 실패 시 IK/goal invalid
  //   - plan / execute 는 MoveToNamed 와 동일하게 직렬 실행
  // ══════════════════════════════════════════════════════════════════════

  rclcpp_action::GoalResponse handle_joints_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToJoints::Goal> goal)
  {
    if (goal->positions.empty()) {
      RCLCPP_WARN(get_logger(), "MoveToJoints: positions is empty, rejecting");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!goal->joint_names.empty() &&
        goal->joint_names.size() != goal->positions.size()) {
      RCLCPP_WARN(get_logger(),
        "MoveToJoints: joint_names(%zu) and positions(%zu) size mismatch",
        goal->joint_names.size(), goal->positions.size());
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_logger(),
      "MoveToJoints: %zu joints, velocity_scale=%.3f",
      goal->positions.size(), goal->velocity_scale);

    bool expected = false;
    if (!arm_busy_.compare_exchange_strong(
          expected, true, std::memory_order_acquire, std::memory_order_relaxed)) {
      RCLCPP_WARN(get_logger(), "MoveToJoints: arm is busy, rejecting goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_joints_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToJoints>>)
  {
    RCLCPP_INFO(get_logger(), "MoveToJoints: cancel requested");
    if (arm_group_) arm_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_joints_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToJoints>> goal_handle)
  {
    if (arm_thread_.joinable()) arm_thread_.join();
    arm_thread_ = std::thread([this, goal_handle]() { execute_joints(goal_handle); });
  }

  void execute_joints(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToJoints>> goal_handle)
  {
    BusyGuard     busy_guard{arm_busy_};
    VelScaleGuard vel_guard{arm_group_.get()};

    auto result   = std::make_shared<MoveToJoints::Result>();
    auto feedback = std::make_shared<MoveToJoints::Feedback>();
    const auto & goal = goal_handle->get_goal();

    if (!arm_group_) {
      result->success = false;
      result->message = "MoveGroupInterface not initialized";
      goal_handle->abort(result);
      vel_guard.grp = nullptr;
      return;
    }

    // velocity_scale 적용 (0 또는 음수는 기본값 유지)
    if (goal->velocity_scale > 0.0f) {
      double v_scale = std::clamp(static_cast<double>(goal->velocity_scale), 0.01, 1.0);
      arm_group_->setMaxVelocityScalingFactor(v_scale);
    }

    // joint_names 가 비어있으면 arm 그룹의 active joints 사용
    std::vector<std::string> names = goal->joint_names;
    if (names.empty()) {
      names = arm_group_->getActiveJoints();
      if (names.size() != goal->positions.size()) {
        result->success = false;
        result->message = "joint_names empty but positions size("
          + std::to_string(goal->positions.size())
          + ") != active joints("
          + std::to_string(names.size()) + ")";
        goal_handle->abort(result);
        RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
        return;
      }
    }

    std::map<std::string, double> target_map;
    for (std::size_t i = 0; i < names.size(); ++i) {
      target_map.emplace(names[i], goal->positions[i]);
    }

    feedback->progress = 0.0f;
    feedback->status   = "planning";
    goal_handle->publish_feedback(feedback);

    arm_group_->setStartStateToCurrentState();

    if (!arm_group_->setJointValueTarget(target_map)) {
      result->success = false;
      result->message = "Joint target out of limits or unknown joint name";
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "MoveToJoints: %s", result->message.c_str());
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!planned) {
      result->success = false;
      result->message = "Planning failed";
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "MoveToJoints: planning failed");
      return;
    }

    feedback->progress = 0.5f;
    feedback->status   = "executing";
    goal_handle->publish_feedback(feedback);

    const auto exec_code = arm_group_->execute(plan);
    bool executed = (exec_code == moveit::core::MoveItErrorCode::SUCCESS);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Cancelled";
      goal_handle->canceled(result);
      return;
    }

    result->success = executed;
    result->message = executed
      ? "Moved to joint target"
      : execute_failure_message("MoveToJoints execution failed", exec_code);

    if (executed) {
      feedback->progress = 1.0f;
      feedback->status   = "done";
      goal_handle->publish_feedback(feedback);
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "MoveToJoints: succeeded");
    } else {
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    }
  }


  // ══════════════════════════════════════════════════════════════════════
  // MoveToPose
  // ══════════════════════════════════════════════════════════════════════

  rclcpp_action::GoalResponse handle_pose_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const MoveToPose::Goal> goal)
  {
    const auto & ps  = goal->target_pose;
    const auto & pos = ps.pose.position;
    const auto & ori = ps.pose.orientation;

    RCLCPP_INFO(get_logger(),
      "MoveToPose: target (%.3f, %.3f, %.3f) frame='%s' plan_only=%s preview_in_sim=%s",
      pos.x, pos.y, pos.z, ps.header.frame_id.c_str(),
      goal->plan_only ? "true" : "false",
      goal->preview_in_sim ? "true" : "false");

    if (goal->preview_in_sim && !goal->plan_only) {
      RCLCPP_WARN(
        get_logger(),
        "MoveToPose: preview_in_sim requires plan_only=true, rejecting goal");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // frame_id 누락 경고 (비어있어도 MoveIt 기본 planning frame 으로 처리하므로 reject 않음)
    if (ps.header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(),
        "MoveToPose: frame_id is empty — MoveIt will use the default planning frame");
    }

    // quaternion 정규화 검사: 비정규 값은 IK/planning 오류를 유발한다
    double qn = std::sqrt(
      ori.x*ori.x + ori.y*ori.y + ori.z*ori.z + ori.w*ori.w);
    if (std::abs(qn - 1.0) > QUAT_NORM_TOL) {
      RCLCPP_WARN(get_logger(),
        "MoveToPose: quaternion not normalized (norm=%.6f), rejecting", qn);
      return rclcpp_action::GoalResponse::REJECT;
    }

    // 워크스페이스 경계 사전 검사
    double xy_dist = std::sqrt(pos.x * pos.x + pos.y * pos.y);
    if (pos.z < WS_Z_MIN || pos.z > WS_Z_MAX || xy_dist > WS_XY_RADIUS) {
      RCLCPP_WARN(get_logger(),
        "MoveToPose: pose out of workspace (z=%.3f, xy_dist=%.3f)", pos.z, xy_dist);
      return rclcpp_action::GoalResponse::REJECT;
    }

    // arm 동시 실행 차단
    bool expected = false;
    if (!arm_busy_.compare_exchange_strong(
          expected, true, std::memory_order_acquire, std::memory_order_relaxed)) {
      RCLCPP_WARN(get_logger(), "MoveToPose: arm is busy, rejecting goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_pose_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPose>>)
  {
    RCLCPP_INFO(get_logger(), "MoveToPose: cancel requested");
    if (arm_group_) arm_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_pose_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPose>> goal_handle)
  {
    if (arm_thread_.joinable()) arm_thread_.join();
    arm_thread_ = std::thread([this, goal_handle]() { execute_pose(goal_handle); });
  }

  void execute_pose(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPose>> goal_handle)
  {
    BusyGuard   busy_guard{arm_busy_};                  // arm_busy_ 해제
    VelScaleGuard vel_guard{arm_group_.get()};          // velocity scaling 기본값 복원

    auto result   = std::make_shared<MoveToPose::Result>();
    auto feedback = std::make_shared<MoveToPose::Feedback>();
    const auto & goal = goal_handle->get_goal();

    if (!arm_group_) {
      result->success = false;
      result->message = "MoveGroupInterface not initialized";
      goal_handle->abort(result);
      vel_guard.grp = nullptr;  // 복원 불필요
      return;
    }

    // velocity_scale 적용 (0.01~1.0): VelScaleGuard 가 종료 시 DEFAULT_VEL_SCALE 로 복원
    // arm_busy_ 로 직렬화되어 있어 다른 arm goal 과 겹치지 않는다.
    double v_scale = std::clamp(static_cast<double>(goal->velocity_scale), 0.01, 1.0);
    arm_group_->setMaxVelocityScalingFactor(v_scale);

    const std::string target_link = arm_tip_link_;
    const std::string planning_frame = arm_group_->getPlanningFrame();
    const std::string pose_frame = arm_group_->getPoseReferenceFrame();
    const auto & target_pose = goal->target_pose.pose;

    RCLCPP_INFO(
      get_logger(),
      "MoveToPose: planning_frame='%s' pose_reference_frame='%s' target_link='%s'",
      planning_frame.c_str(),
      pose_frame.c_str(),
      target_link.empty() ? "<empty>" : target_link.c_str());
    RCLCPP_INFO(
      get_logger(),
      "MoveToPose: target pose p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
      target_pose.position.x,
      target_pose.position.y,
      target_pose.position.z,
      target_pose.orientation.x,
      target_pose.orientation.y,
      target_pose.orientation.z,
      target_pose.orientation.w);

    // ── Workspace 안전 제한 ─────────────────────────────────────────────
    // EE 가 멀리 뻗으면 shoulder(dxl12) 가 Overload 로 차단되어 로봇이 정지한다.
    // pose frame 이 ws_frame_ 과 일치할 때만 검증 (다른 frame 은 TF 변환 비용 회피).
    {
      const std::string & req_frame = goal->target_pose.header.frame_id;
      if (req_frame.empty() || req_frame == ws_frame_ || req_frame == planning_frame) {
        const double ax = std::abs(target_pose.position.x);
        const double ay = std::abs(target_pose.position.y);
        const double z  = target_pose.position.z;
        const bool out_of_range =
          (ax > ws_x_abs_max_) ||
          (ay > ws_y_abs_max_) ||
          (z  < ws_z_min_)     ||
          (z  > ws_z_max_);
        if (out_of_range) {
          char msg[256];
          std::snprintf(
            msg, sizeof(msg),
            "Target out of workspace: p=(%.3f,%.3f,%.3f) limit |x|<=%.3f, |y|<=%.3f, %.3f<=z<=%.3f",
            target_pose.position.x, target_pose.position.y, target_pose.position.z,
            ws_x_abs_max_, ws_y_abs_max_, ws_z_min_, ws_z_max_);
          result->success = false;
          result->message = msg;
          goal_handle->abort(result);
          RCLCPP_WARN(get_logger(), "MoveToPose: %s", msg);
          return;
        }
      } else {
        RCLCPP_WARN(
          get_logger(),
          "MoveToPose: target frame_id='%s' differs from workspace frame='%s' / planning_frame='%s'; "
          "skipping workspace check",
          req_frame.c_str(), ws_frame_.c_str(), planning_frame.c_str());
      }
    }

    const auto current_pose = target_link.empty()
      ? arm_group_->getCurrentPose()
      : arm_group_->getCurrentPose(target_link);
    const auto & current = current_pose.pose;
    RCLCPP_INFO(
      get_logger(),
      "MoveToPose: current pose p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
      current.position.x,
      current.position.y,
      current.position.z,
      current.orientation.x,
      current.orientation.y,
      current.orientation.z,
      current.orientation.w);

    feedback->progress = 0.0f;
    feedback->status   = "planning";
    goal_handle->publish_feedback(feedback);

    // 현재 조인트 상태로 시작점 동기화
    arm_group_->setStartStateToCurrentState();

    // 5-DOF OMX 에서는 setPoseTarget + RRTConnect 조합이 불안정하다.
    // RRTConnect 의 goal sampler 가 매 sample 마다 KDL IK 를 호출하는데
    // under-actuated arm 이라 6-DOF IK 가 수렴 실패 → GOAL_STATE_INVALID.
    // setJointValueTarget(pose, link) 는 IK 를 한 번만 호출해 joint 값을 구하고
    // joint-space goal 로 설정한다. RRTConnect 는 joint RRT 로 안정적으로 풀린다.
    // IK 가 완전해를 찾지 못하면 setApproximateJointValueTarget 로 fallback 한다.
    bool ik_ok = target_link.empty()
      ? arm_group_->setJointValueTarget(goal->target_pose)
      : arm_group_->setJointValueTarget(goal->target_pose, target_link);

    if (!ik_ok) {
      RCLCPP_WARN(
        get_logger(),
        "MoveToPose: exact IK failed, trying approximate IK for target_link='%s'",
        target_link.empty() ? "<empty>" : target_link.c_str());
      ik_ok = target_link.empty()
        ? arm_group_->setApproximateJointValueTarget(goal->target_pose)
        : arm_group_->setApproximateJointValueTarget(goal->target_pose, target_link);
    }

    if (!ik_ok) {
      result->success = false;
      result->message = "IK failed for target pose (out of reach or invalid orientation)";
      goal_handle->abort(result);
      RCLCPP_WARN(
        get_logger(),
        "MoveToPose: IK (exact + approximate) failed for p=(%.3f,%.3f,%.3f)",
        target_pose.position.x, target_pose.position.y, target_pose.position.z);
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!planned) {
      result->success = false;
      result->message = "Planning failed";
      goal_handle->abort(result);
      RCLCPP_WARN(
        get_logger(),
        "MoveToPose: planning failed for target_link='%s' in planning_frame='%s'",
        target_link.empty() ? "<empty>" : target_link.c_str(),
        planning_frame.c_str());
      return;  // guard 들이 velocity 복원 + busy 해제
    }

    feedback->progress = 0.5f;
    feedback->status   = "planned";
    goal_handle->publish_feedback(feedback);

    if (goal->preview_in_sim) {
      auto preview_trajectory = build_preview_trajectory(plan);
      const std::string trajectory_file = write_preview_trajectory_file(preview_trajectory);
      if (trajectory_file.empty()) {
        result->success = false;
        result->message = "Failed to write preview trajectory file";
        goal_handle->abort(result);
        RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
        return;
      }

      if (!preview_instance_alive()) {
        std::string preview_log_path;
        const bool launched = launch_preview_gazebo(&preview_log_path);
        if (!launched) {
          result->success = false;
          result->message = "Failed to launch Gazebo preview";
          goal_handle->abort(result);
          RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
          return;
        }
        if (!wait_for_preview_ready()) {
          result->success = false;
          result->message =
            "Gazebo preview did not become ready (log=" + preview_log_path + ")";
          goal_handle->abort(result);
          RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
          return;
        }
      }

      const long long request_id = next_preview_request_id();
      const auto request_path = preview_runtime_path("request.json");
      {
        std::ofstream request_out(request_path, std::ios::trunc);
        if (!request_out.is_open()) {
          result->success = false;
          result->message = "Failed to write preview request file";
          goal_handle->abort(result);
          RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
          return;
        }
        request_out
          << "{\n"
          << "  \"request_id\": " << request_id << ",\n"
          << "  \"trajectory_file\": \"" << trajectory_file << "\"\n"
          << "}\n";
      }

      if (!wait_for_preview_accept(request_id)) {
        result->success = false;
        result->message = "Preview instance did not accept request in time";
        goal_handle->abort(result);
        RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
        return;
      }

      std::string preview_log_path;
      preview_log_path = preview_runtime_path("gazebo.log").string();

      result->success = true;
      result->message =
        "Plan valid; Gazebo preview executing (ROS_DOMAIN_ID=" +
        std::to_string(PREVIEW_DOMAIN_ID) +
        ", request_id=" + std::to_string(request_id) +
        ", trajectory=" + trajectory_file +
        ", log=" + preview_log_path + ")";
      feedback->progress = 1.0f;
      feedback->status = "preview_executing";
      goal_handle->publish_feedback(feedback);
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
      return;
    }

    if (goal->plan_only) {
      result->success = true;
      result->message = "Plan valid";
      feedback->progress = 1.0f;
      feedback->status = "validated";
      goal_handle->publish_feedback(feedback);
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "MoveToPose: plan-only validation succeeded");
      return;
    }

    feedback->status = "executing";
    goal_handle->publish_feedback(feedback);

    // 디버깅 도움: execute 직전 current joint / pose 상태를 남겨 plan 의 start state 와
    // controller current state 사이의 drift 여부를 추적할 수 있게 한다.
    log_current_arm_state("MoveToPose pre-execute");

    const auto exec_code = arm_group_->execute(plan);
    bool executed = (exec_code == moveit::core::MoveItErrorCode::SUCCESS);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Cancelled";
      goal_handle->canceled(result);
      return;
    }

    result->success = executed;
    result->message = executed
      ? "Pose reached"
      : execute_failure_message("MoveToPose execution failed", exec_code);

    if (!executed) {
      log_current_arm_state("MoveToPose post-execute (failed)");
    }

    if (executed) {
      feedback->progress = 1.0f;
      feedback->status   = "done";
      goal_handle->publish_feedback(feedback);
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "MoveToPose: succeeded");
    } else {
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    }
  }


  // ══════════════════════════════════════════════════════════════════════
  // GripperCommand  →  MoveGroupInterface("gripper") 로 trajectory 실행
  //
  // GripperActionController 를 직접 호출하면 velocity profile 없이
  // 순간 이동하여 모터에 무리가 간다.
  // MoveIt gripper 그룹을 사용하면 joint_limits.yaml 의 velocity/acceleration
  // scaling 이 적용되어 부드럽게 움직인다.
  // ══════════════════════════════════════════════════════════════════════

  rclcpp_action::GoalResponse handle_gripper_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GripperCmd::Goal> goal)
  {
    RCLCPP_INFO(get_logger(),
      "GripperCommand: position=%.3f, max_effort=%.1f",
      goal->position, goal->max_effort);

    bool expected = false;
    if (!gripper_busy_.compare_exchange_strong(
          expected, true, std::memory_order_acquire, std::memory_order_relaxed)) {
      RCLCPP_WARN(get_logger(), "GripperCommand: gripper is busy, rejecting goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_gripper_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCmd>>)
  {
    if (gripper_group_) gripper_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_gripper_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCmd>> goal_handle)
  {
    if (gripper_thread_.joinable()) gripper_thread_.join();
    gripper_thread_ = std::thread([this, goal_handle]() { execute_gripper(goal_handle); });
  }

  void execute_gripper(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCmd>> goal_handle)
  {
    BusyGuard busy_guard{gripper_busy_};  // 모든 return 경로에서 gripper_busy_ 해제

    auto result   = std::make_shared<GripperCmd::Result>();
    auto feedback = std::make_shared<GripperCmd::Feedback>();
    const auto & goal = goal_handle->get_goal();

    if (!gripper_group_) {
      result->success = false;
      result->message = "MoveGroupInterface(gripper) not initialized";
      goal_handle->abort(result);
      return;
    }

    // max_effort 은 현재 미구현 — 컨트롤러 레벨 전류 제한에 의존한다.
    if (goal->max_effort > 0.0f) {
      RCLCPP_WARN(get_logger(),
        "GripperCommand: max_effort=%.1f requested but not enforced"
        " — hardware current limit applies",
        goal->max_effort);
    }

    double pos = std::clamp(
      static_cast<double>(goal->position), GRIPPER_CLOSE_POS, GRIPPER_OPEN_POS);

    feedback->position = static_cast<float>(pos);
    goal_handle->publish_feedback(feedback);

    // 반드시 현재 joint state 를 start 로 동기화한다.
    //   MoveGroupInterface 는 내부적으로 "마지막 setJointValueTarget 의 goal"을
    //   다음 plan 의 start state 로 유지하는 경향이 있어, 연속 호출(예: open→close
    //   →open)에서 두 번째 open 이 stale start state 로 PLANNING_FAILED 가 된다.
    gripper_group_->setStartStateToCurrentState();
    gripper_group_->setJointValueTarget("gripper_joint_1", pos);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_code = gripper_group_->plan(plan);
    bool planned = (plan_code == moveit::core::MoveItErrorCode::SUCCESS);

    if (!planned) {
      result->success = false;
      result->message = execute_failure_message(
        "Gripper planning failed", plan_code);
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(),
        "GripperCommand: %s (pos=%.3f)",
        result->message.c_str(), pos);
      return;
    }

    const auto exec_code = gripper_group_->execute(plan);
    bool executed = (exec_code == moveit::core::MoveItErrorCode::SUCCESS);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Cancelled";
      goal_handle->canceled(result);
      return;
    }

    result->success  = executed;
    result->position = static_cast<float>(pos);
    result->message  = executed
      ? "Gripper command succeeded"
      : execute_failure_message("Gripper execution failed", exec_code);

    if (executed) {
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "GripperCommand: pos=%.3f succeeded", pos);
    } else {
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "GripperCommand: %s", result->message.c_str());
    }
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<MotionServer>(options);
  node->create_moveit_helper_node();

  // MultiThreadedExecutor: MoveGroupInterface 와 action server callback 이
  // 서로 블로킹하지 않도록 별도 스레드에서 spin 한다.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(node->get_moveit_helper_node());

  // executor.spin() 을 먼저 돌린다.
  //   - 액션 서버는 생성자에서 이미 등록됐으므로, 이 시점부터 `ros2 action info`
  //     로 즉시 발견된다.
  //   - MoveGroupInterface 초기화가 지연/실패해도 프로세스는 종료하지 않는다.
  //     이전 구조는 15s 타임아웃 시 return 1 → 액션 서버까지 사라져 DDS 그래프에
  //     zombie 노드가 누적됐고, `ros2 action info` 에서 0 server 로 보였다.
  std::thread spin_thread([&executor]() { executor.spin(); });

  // MoveIt 초기화는 백그라운드에서 재시도. 연결되면 루프 종료.
  //   - MoveGroupInterface 생성은 move_group 과 service handshake 가 필요해
  //     순간적으로 실패할 수 있다 (move_group 준비 중일 때).
  //   - 실패해도 node 는 계속 살아있고 액션 goal 은 arm_group_ 체크로 abort 된다.
  std::thread init_thread([node]() {
    constexpr auto kPollInterval = std::chrono::milliseconds(200);
    constexpr auto kPollAttempts = 50;                 // 약 10초
    constexpr auto kRetryInterval = std::chrono::seconds(2);
    while (rclcpp::ok()) {
      try {
        node->init_moveit();
      } catch (const std::exception & e) {
        RCLCPP_WARN(node->get_logger(),
          "init_moveit threw: %s — retrying in 2s", e.what());
        std::this_thread::sleep_for(kRetryInterval);
        continue;
      }
      // init_moveit 가 예외 없이 반환해도 planning_frame 이 채워지는 데
      // 잠깐 시간이 걸릴 수 있으므로 짧게 폴링한다.
      for (int i = 0; i < kPollAttempts && rclcpp::ok(); ++i) {
        if (node->is_moveit_ready()) {
          RCLCPP_INFO(node->get_logger(),
            "MoveGroupInterface ready — MotionServer running");
          return;
        }
        std::this_thread::sleep_for(kPollInterval);
      }
      RCLCPP_WARN(node->get_logger(),
        "MoveGroupInterface not ready after 10s — rebuilding in 2s");
      std::this_thread::sleep_for(kRetryInterval);
    }
  });

  spin_thread.join();
  if (init_thread.joinable()) init_thread.join();
  rclcpp::shutdown();
  return 0;
}
