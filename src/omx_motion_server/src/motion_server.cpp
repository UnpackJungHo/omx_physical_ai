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

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <omx_interfaces/action/move_to_named.hpp>
#include <omx_interfaces/action/move_to_pose.hpp>
#include <omx_interfaces/action/gripper_command.hpp>

// ── 워크스페이스 한계 (workspace_guard 와 일치) ───────────────────────────
static constexpr double WS_Z_MIN = 0.01;   // floor collision object top
static constexpr double WS_Z_MAX = 0.48;   // ceiling collision object bottom
static constexpr double WS_XY_RADIUS = 0.42;  // 팔 최대 도달 거리

// ── 그리퍼 조인트 위치 범위 (SRDF: close=0, open=1) ──────────────────────
static constexpr double GRIPPER_CLOSE_POS = 0.0;
static constexpr double GRIPPER_OPEN_POS  = 1.0;

// ── named pose → SRDF group_state 매핑 ───────────────────────────────────
// SRDF(omx_f.srdf)에 등록된 arm 그룹 named state 목록:
//   "init" : 모든 조인트 0
//   "home" : joint2=-1.57, joint3=1.57, joint4=1.57
// 그 외 이름은 매핑 테이블로 SRDF 이름으로 변환한다.
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
    // MoveGroupInterface 호출은 블로킹이므로, 실제로는 한 번에 하나씩 실행된다.
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    move_to_named_server_ = rclcpp_action::create_server<MoveToNamed>(
      this, "/omx/move_to_named",
      std::bind(&MotionServer::handle_named_goal,    this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MotionServer::handle_named_cancel,  this, std::placeholders::_1),
      std::bind(&MotionServer::handle_named_accepted, this, std::placeholders::_1),
      rcl_action_server_get_default_options(), cb_group_);

    move_to_pose_server_ = rclcpp_action::create_server<MoveToPose>(
      this, "/omx/move_to_pose",
      std::bind(&MotionServer::handle_pose_goal,    this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MotionServer::handle_pose_cancel,  this, std::placeholders::_1),
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

  // MoveGroupInterface 는 node 가 spin 된 이후에 초기화해야 한다.
  void init_moveit()
  {
    arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm");
    arm_group_->setMaxVelocityScalingFactor(0.3);
    arm_group_->setMaxAccelerationScalingFactor(0.1);
    arm_group_->setPlanningTime(5.0);

    // 그리퍼도 MoveGroupInterface 로 제어: joint_limits.yaml 의 velocity scaling 적용
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "gripper");
    gripper_group_->setMaxVelocityScalingFactor(0.3);
    gripper_group_->setMaxAccelerationScalingFactor(0.1);
    gripper_group_->setPlanningTime(3.0);

    RCLCPP_INFO(get_logger(), "MoveGroupInterface ready. Planning frame: %s",
      arm_group_->getPlanningFrame().c_str());
  }

private:
  // ── 멤버 ───────────────────────────────────────────────────────────────
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  rclcpp_action::Server<MoveToNamed>::SharedPtr move_to_named_server_;
  rclcpp_action::Server<MoveToPose>::SharedPtr  move_to_pose_server_;
  rclcpp_action::Server<GripperCmd>::SharedPtr  gripper_server_;


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
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_named_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToNamed>>)
  {
    RCLCPP_INFO(get_logger(), "MoveToNamed: cancel requested");
    if (arm_group_) arm_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_named_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToNamed>> goal_handle)
  {
    std::thread([this, goal_handle]() { execute_named(goal_handle); }).detach();
  }

  void execute_named(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToNamed>> goal_handle)
  {
    auto result = std::make_shared<MoveToNamed::Result>();
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
    const auto & pos = goal->target_pose.pose.position;
    RCLCPP_INFO(get_logger(),
      "MoveToPose: target (%.3f, %.3f, %.3f)", pos.x, pos.y, pos.z);

    // 워크스페이스 경계 사전 검사
    double xy_dist = std::sqrt(pos.x * pos.x + pos.y * pos.y);
    if (pos.z < WS_Z_MIN || pos.z > WS_Z_MAX || xy_dist > WS_XY_RADIUS) {
      RCLCPP_WARN(get_logger(),
        "MoveToPose: pose out of workspace (z=%.3f, xy_dist=%.3f)", pos.z, xy_dist);
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
    std::thread([this, goal_handle]() { execute_pose(goal_handle); }).detach();
  }

  void execute_pose(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPose>> goal_handle)
  {
    auto result   = std::make_shared<MoveToPose::Result>();
    auto feedback = std::make_shared<MoveToPose::Feedback>();

    const auto & goal = goal_handle->get_goal();

    if (!arm_group_) {
      result->success = false;
      result->message = "MoveGroupInterface not initialized";
      goal_handle->abort(result);
      return;
    }

    // velocity_scale 적용 (0.0~1.0, 기본값 0.5)
    double v_scale = std::clamp(static_cast<double>(goal->velocity_scale), 0.01, 1.0);
    arm_group_->setMaxVelocityScalingFactor(v_scale);

    feedback->progress = 0.0f;
    feedback->status   = "planning";
    goal_handle->publish_feedback(feedback);

    arm_group_->setPoseTarget(goal->target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!planned) {
      result->success = false;
      result->message = "Planning failed";
      goal_handle->abort(result);
      arm_group_->setMaxVelocityScalingFactor(0.3);  // 기본값 복원
      RCLCPP_WARN(get_logger(), "MoveToPose: planning failed");
      return;
    }

    feedback->progress = 0.5f;
    feedback->status   = "executing";
    goal_handle->publish_feedback(feedback);

    bool executed = (arm_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    arm_group_->setMaxVelocityScalingFactor(0.3);  // 기본값 복원

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
    std::thread([this, goal_handle]() { execute_gripper(goal_handle); }).detach();
  }

  void execute_gripper(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCmd>> goal_handle)
  {
    auto result   = std::make_shared<GripperCmd::Result>();
    auto feedback = std::make_shared<GripperCmd::Feedback>();

    const auto & goal = goal_handle->get_goal();

    if (!gripper_group_) {
      result->success = false;
      result->message = "MoveGroupInterface(gripper) not initialized";
      goal_handle->abort(result);
      return;
    }

    // position 클램핑 (0.0=닫힘, 1.0=열림)
    double pos = std::clamp(static_cast<double>(goal->position),
      GRIPPER_CLOSE_POS, GRIPPER_OPEN_POS);

    feedback->position = static_cast<float>(pos);
    goal_handle->publish_feedback(feedback);

    // gripper_joint_1 에 목표 위치 설정 → MoveIt 이 velocity profile 포함 trajectory 생성
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

  // MoveGroupInterface 는 executor 가 spin 을 시작한 뒤 초기화해야 한다.
  std::thread spin_thread([&executor]() { executor.spin(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  node->init_moveit();

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
