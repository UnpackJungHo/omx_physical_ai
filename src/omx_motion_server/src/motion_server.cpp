// omx_motion_server — motion_server.cpp
//
// Action servers:
//   /omx/move_to_named   (omx_interfaces/action/MoveToNamed)
//   /omx/move_to_pose    (omx_interfaces/action/MoveToPose)
//   /omx/gripper_command (omx_interfaces/action/GripperCommand)
//
// 세 액션 모두 MoveGroupInterface 로 제어한다.
//   arm     그룹: MoveToNamed, MoveToPose
//   gripper 그룹: GripperCommand  ← GripperActionController 직접 호출 대신
//                                    MoveIt trajectory 로 속도 제한 적용
//
// 동시성 정책:
//   arm_busy_ (atomic<bool>) — MoveToNamed / MoveToPose 가 실행 중이면 true.
//     handle_*_goal 에서 CAS 로 선점하고 execute_* 소멸 시 RAII 로 해제한다.
//     결과적으로 arm_group_ 의 setNamedTarget / setPoseTarget /
//     setMaxVelocityScalingFactor 호출이 직렬화된다.
//   gripper_busy_ — GripperCommand 에 동일하게 적용.
//   arm_thread_ / gripper_thread_ — detach 대신 명시적 join 으로 수명 관리.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <omx_interfaces/action/move_to_named.hpp>
#include <omx_interfaces/action/move_to_pose.hpp>
#include <omx_interfaces/action/gripper_command.hpp>

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

// ── quaternion 정규화 허용 오차 ───────────────────────────────────────────
static constexpr double QUAT_NORM_TOL = 0.01;

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
  using MoveToNamed = omx_interfaces::action::MoveToNamed;
  using MoveToPose  = omx_interfaces::action::MoveToPose;
  using GripperCmd  = omx_interfaces::action::GripperCommand;

  explicit MotionServer(const rclcpp::NodeOptions & options)
  : Node("motion_server", options)
  {
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

    gripper_server_ = rclcpp_action::create_server<GripperCmd>(
      this, "/omx/gripper_command",
      std::bind(&MotionServer::handle_gripper_goal,     this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MotionServer::handle_gripper_cancel,   this, std::placeholders::_1),
      std::bind(&MotionServer::handle_gripper_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), cb_group_);

    RCLCPP_INFO(get_logger(), "MotionServer: action servers created, waiting for MoveIt...");
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
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm");
    arm_group_->setMaxVelocityScalingFactor(DEFAULT_VEL_SCALE);
    arm_group_->setMaxAccelerationScalingFactor(DEFAULT_ACC_SCALE);
    arm_group_->setPlanningTime(5.0);

    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "gripper");
    gripper_group_->setMaxVelocityScalingFactor(DEFAULT_VEL_SCALE);
    gripper_group_->setMaxAccelerationScalingFactor(DEFAULT_ACC_SCALE);
    gripper_group_->setPlanningTime(3.0);
  }

  // MoveGroupInterface 연결 완료 여부 — main 의 readiness 폴링에 사용
  bool is_moveit_ready() const
  {
    return arm_group_     && !arm_group_->getPlanningFrame().empty()
        && gripper_group_ && !gripper_group_->getPlanningFrame().empty();
  }

private:
  // ── 멤버 ───────────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  rclcpp_action::Server<MoveToNamed>::SharedPtr move_to_named_server_;
  rclcpp_action::Server<MoveToPose>::SharedPtr  move_to_pose_server_;
  rclcpp_action::Server<GripperCmd>::SharedPtr  gripper_server_;

  // ── 동시성 제어 ────────────────────────────────────────────────────────
  // busy 플래그는 handle_*_goal 에서 CAS 로 설정되고 execute_* 의
  // BusyGuard 소멸자에서 해제된다 (모든 return 경로 포함).
  std::atomic<bool> arm_busy_{false};
  std::atomic<bool> gripper_busy_{false};
  std::thread arm_thread_;      // detach 대신 handle_*_accepted 에서 join 후 교체
  std::thread gripper_thread_;


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

    bool executed = (arm_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Cancelled";
      goal_handle->canceled(result);
      return;
    }

    result->success = executed;
    result->message = executed ? "Moved to '" + requested + "'" : "Execution failed";
    if (executed) {
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
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
      "MoveToPose: target (%.3f, %.3f, %.3f) frame='%s'",
      pos.x, pos.y, pos.z, ps.header.frame_id.c_str());

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

    feedback->progress = 0.0f;
    feedback->status   = "planning";
    goal_handle->publish_feedback(feedback);

    // 현재 조인트 상태로 시작점 동기화
    arm_group_->setStartStateToCurrentState();
    arm_group_->setPoseTarget(goal->target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!planned) {
      result->success = false;
      result->message = "Planning failed";
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "MoveToPose: planning failed");
      return;  // guard 들이 velocity 복원 + busy 해제
    }

    feedback->progress = 0.5f;
    feedback->status   = "executing";
    goal_handle->publish_feedback(feedback);

    bool executed = (arm_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Cancelled";
      goal_handle->canceled(result);
      return;
    }

    result->success = executed;
    result->message = executed ? "Pose reached" : "Execution failed";
    if (executed) {
      feedback->progress = 1.0f;
      feedback->status   = "done";
      goal_handle->publish_feedback(feedback);
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "MoveToPose: succeeded");
    } else {
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "MoveToPose: execution failed");
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

    // gripper_joint_1 에 목표 위치 → MoveIt 이 velocity profile 포함 trajectory 생성
    gripper_group_->setJointValueTarget("gripper_joint_1", pos);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = (gripper_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!planned) {
      result->success = false;
      result->message = "Gripper planning failed";
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "GripperCommand: planning failed (pos=%.3f)", pos);
      return;
    }

    bool executed = (gripper_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (goal_handle->is_canceling()) {
      result->success = false;
      result->message = "Cancelled";
      goal_handle->canceled(result);
      return;
    }

    result->success  = executed;
    result->position = static_cast<float>(pos);
    result->message  = executed ? "Gripper command succeeded" : "Gripper execution failed";

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

  // MultiThreadedExecutor: MoveGroupInterface 와 action server callback 이
  // 서로 블로킹하지 않도록 별도 스레드에서 spin 한다.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // executor.spin() 이 먼저 돌아야 MoveGroupInterface 가 연결될 수 있다.
  std::thread spin_thread([&executor]() { executor.spin(); });

  // MoveGroupInterface 객체 생성 (비동기 연결 시작)
  node->init_moveit();

  // 고정 sleep 대신 readiness 폴링으로 경쟁 조건 완화
  // getPlanningFrame() 이 비어있으면 아직 move_group 과 연결되지 않은 상태다.
  constexpr auto kTimeout  = std::chrono::seconds(15);
  constexpr auto kInterval = std::chrono::milliseconds(200);
  const auto deadline = std::chrono::steady_clock::now() + kTimeout;

  while (!node->is_moveit_ready()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_FATAL(node->get_logger(),
        "MoveGroupInterface not ready after 15s — is move_group running?");
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 1;
    }
    std::this_thread::sleep_for(kInterval);
  }

  RCLCPP_INFO(node->get_logger(),
    "MoveGroupInterface ready — MotionServer running");

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
