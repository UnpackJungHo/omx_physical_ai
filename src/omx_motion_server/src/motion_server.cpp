#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <omx_interfaces/action/move_to_named.hpp>
#include <omx_interfaces/action/move_to_pose.hpp>
#include <omx_interfaces/action/move_to_joints.hpp>
#include <omx_interfaces/action/gripper_command.hpp>
#include <omx_interfaces/srv/compute_align_yaw.hpp>

#include <rclcpp/logging.hpp> 
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "omx_motion_server/motion_geometry.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>


// ── 그리퍼 조인트 위치 범위 (SRDF: close=0, open=1) ──────────────────────
static constexpr double GRIPPER_CLOSE_POS = 0.0;
static constexpr double GRIPPER_OPEN_POS = 1.0;

// MoveToNamed용 (srdf 에 정의된 이름, 변경될 이름)
static const std::unordered_map<std::string, std::string> NAMED_POSE_MAP = {
    {"home", "home"},
    {"init", "init"},
};

// MoveItErrorCode → 사람이 읽을 수 있는 이름으로 변환
// execute() 실패 원인을 action result message 에 담을 때 사용
static std::string moveit_error_name(const moveit::core::MoveItErrorCode &code)
{
  using C = moveit_msgs::msg::MoveItErrorCodes;
  switch (code.val)
  {
  case C::SUCCESS:
    return "SUCCESS";
  case C::FAILURE:
    return "FAILURE";
  case C::PLANNING_FAILED:
    return "PLANNING_FAILED";
  case C::INVALID_MOTION_PLAN:
    return "INVALID_MOTION_PLAN";
  case C::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
    return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
  case C::CONTROL_FAILED:
    return "CONTROL_FAILED";
  case C::UNABLE_TO_AQUIRE_SENSOR_DATA:
    return "UNABLE_TO_AQUIRE_SENSOR_DATA";
  case C::TIMED_OUT:
    return "TIMED_OUT";
  case C::PREEMPTED:
    return "PREEMPTED";
  case C::START_STATE_IN_COLLISION:
    return "START_STATE_IN_COLLISION";
  case C::START_STATE_VIOLATES_PATH_CONSTRAINTS:
    return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
  case C::GOAL_IN_COLLISION:
    return "GOAL_IN_COLLISION";
  case C::GOAL_VIOLATES_PATH_CONSTRAINTS:
    return "GOAL_VIOLATES_PATH_CONSTRAINTS";
  case C::GOAL_CONSTRAINTS_VIOLATED:
    return "GOAL_CONSTRAINTS_VIOLATED";
  case C::INVALID_GROUP_NAME:
    return "INVALID_GROUP_NAME";
  case C::INVALID_GOAL_CONSTRAINTS:
    return "INVALID_GOAL_CONSTRAINTS";
  case C::INVALID_ROBOT_STATE:
    return "INVALID_ROBOT_STATE";
  case C::INVALID_LINK_NAME:
    return "INVALID_LINK_NAME";
  case C::INVALID_OBJECT_NAME:
    return "INVALID_OBJECT_NAME";
  case C::FRAME_TRANSFORM_FAILURE:
    return "FRAME_TRANSFORM_FAILURE";
  case C::COLLISION_CHECKING_UNAVAILABLE:
    return "COLLISION_CHECKING_UNAVAILABLE";
  case C::ROBOT_STATE_STALE:
    return "ROBOT_STATE_STALE";
  case C::SENSOR_INFO_STALE:
    return "SENSOR_INFO_STALE";
  case C::NO_IK_SOLUTION:
    return "NO_IK_SOLUTION";
  default:
    return "UNKNOWN";
  }
}

// 실행 실패 메시지 출력 메서드
static std::string execute_failure_message(const std::string &prefix, const moveit::core::MoveItErrorCode &code)
{
  return prefix + " (code=" + moveit_error_name(code) + ", val=" + std::to_string(code.val) + ")";
}

// motion_geometry 순수 함수를 짧은 이름으로 쓰기 위한 전역 using (클래스 밖이어야 함).
using omx_motion_server::jaw_axis_yaw_from_quaternion;
using omx_motion_server::position_in_workspace_box;
using omx_motion_server::wrap_to_pm45;

class MotionServer : public rclcpp::Node {
public:
  using MoveToNamed = omx_interfaces::action::MoveToNamed;
  using MoveToPose = omx_interfaces::action::MoveToPose;
  using MoveToJoints = omx_interfaces::action::MoveToJoints;
  using GripperCmd = omx_interfaces::action::GripperCommand;
  using ComputeAlignYaw = omx_interfaces::srv::ComputeAlignYaw;

  // 생성자: 파라미터 적재 + callback group 생성 + omx/move_to_named 액션 서버를 등록
  explicit MotionServer(const rclcpp::NodeOptions &options)
      : Node("motion_server", options), node_options_(options)
  {
    declare_and_load_params();
        
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    move_to_named_server_ = rclcpp_action::create_server<MoveToNamed>(
        this, "omx/move_to_named",
        std::bind(&MotionServer::handle_named_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&MotionServer::handle_arm_cancel<MoveToNamed>, this,
                  "MoveToNamed", std::placeholders::_1),
        std::bind(&MotionServer::handle_named_accepted,
                  this, // goal이 ACCEPT_AND_EXECUTE로 수락되면 라이브러리가 이
                        // 함수를 부름
                  std::placeholders::_1),
        rcl_action_server_get_default_options(), cb_group_);

    move_to_pose_server_ = rclcpp_action::create_server<MoveToPose>(
        this, "omx/move_to_pose",
        std::bind(&MotionServer::handle_pose_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&MotionServer::handle_arm_cancel<MoveToPose>, this,
                  "MoveToPose", std::placeholders::_1),
        std::bind(&MotionServer::handle_pose_accepted, this,
                  std::placeholders::_1),
        rcl_action_server_get_default_options(), cb_group_);
    
    move_to_joints_server_ = rclcpp_action::create_server<MoveToJoints>(
        this, "omx/move_to_joints",
        std::bind(&MotionServer::handle_joints_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MotionServer::handle_arm_cancel<MoveToJoints>, this, "MoveToJoints", std::placeholders::_1),
        std::bind(&MotionServer::handle_joints_accepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(), cb_group_);

    gripper_server_ = rclcpp_action::create_server<GripperCmd>(
        this, "omx/gripper_command",
        std::bind(&MotionServer::handle_gripper_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MotionServer::handle_gripper_cancel, this, std::placeholders::_1),
        std::bind(&MotionServer::handle_gripper_accepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(), cb_group_);

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Joint states 구독 (best_effort - 센서 스트림)
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states",               // 토픽 이름
         rclcpp::SensorDataQoS(),     // QoS
        [this](sensor_msgs::msg::JointState::SharedPtr msg) { // 메시지 도착 시 실행될 콜백(람다)
          std::lock_guard<std::mutex> lk(joint_state_mutex_);
          latest_joint_state_ = msg;
        });

    compute_align_yaw_server_ = create_service<ComputeAlignYaw>(
          "omx/compute_align_yaw",
          std::bind(&MotionServer::handle_compute_align, this,
                    std::placeholders::_1, std::placeholders::_2),
          rclcpp::ServicesQoS(), cb_group_);

    RCLCPP_INFO(get_logger(), "MotionServer: action servers created, waiting for MoveIt...");
  }

  // 소멸자: 실행 중이던 worker 스레드를 join 해 안전하게 정리 (detach 금지)
  ~MotionServer()
  {
    // 노드 종료 시 실행 중인 스레드를 안전하게 join - detach 없음
    if (arm_thread_.joinable())
      arm_thread_.join();
    if (gripper_thread_.joinable())
      gripper_thread_.join();
  }

  // ────────────────────────────── MoveIt 설정 및 생성 관련 메서드──────────────────────────────────────────────────

  // arm/gripper MoveGroupInterface 를 생성하고 속도·tolerance·planning time 을 적용 (executor spin 이후 호출).
  void init_moveit() {
    if (!moveit_node_)
    {
      create_moveit_helper_node();
    }

    const std::string move_group_namespace = resolved_move_group_namespace();
    const rclcpp::Duration wait_for_move_group =
        rclcpp::Duration::from_seconds(10.0);
    moveit::planning_interface::MoveGroupInterface::Options arm_options(
        "arm",
        moveit::planning_interface::MoveGroupInterface::ROBOT_DESCRIPTION,
        move_group_namespace);
    arm_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            moveit_node_, arm_options, std::shared_ptr<tf2_ros::Buffer>(),
            wait_for_move_group);
    
    arm_group_->setMaxVelocityScalingFactor(default_vel_scale_);
    arm_group_->setMaxAccelerationScalingFactor(default_acc_scale_);
    arm_group_->setPlanningTime(arm_planning_time_);
    arm_group_->setGoalPositionTolerance(arm_goal_position_tol_);
    arm_group_->setGoalOrientationTolerance(arm_goal_orientation_tol_);
    arm_group_->setGoalJointTolerance(arm_goal_joint_tol_);

    moveit::planning_interface::MoveGroupInterface::Options gripper_options(
        "gripper",
        moveit::planning_interface::MoveGroupInterface::ROBOT_DESCRIPTION,
        move_group_namespace);
    
    gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        moveit_node_, gripper_options, std::shared_ptr<tf2_ros::Buffer>(), wait_for_move_group);
    gripper_group_->setMaxVelocityScalingFactor(default_vel_scale_);
    gripper_group_->setMaxAccelerationScalingFactor(default_acc_scale_);
    gripper_group_->setPlanningTime(gripper_planning_time_);
    gripper_group_->setGoalJointTolerance(gripper_goal_joint_tol_);

    arm_tip_link_ = get_parameter("arm_tip_link").as_string();
    if (arm_tip_link_.empty())
    {
      arm_tip_link_ = "end_effector_link";
    }

    RCLCPP_INFO(
        get_logger(),
        "Workspace limit (frame='%s'): |x|<=%.3f, |y|<=%.3f, %.3f<=z<=%.3f",
        ws_frame_.c_str(), ws_x_abs_max_, ws_y_abs_max_, ws_z_min_, ws_z_max_);

    // getPlanningFrame: 모션 플래닝이 수행되는 기준 프레임(보통 로봇 모델의 고정 루트, 예: world / base_link)
    const std::string arm_planning_frame = arm_group_->getPlanningFrame();
    // getPoseReferenceFrame: setPoseTarget/setJointValueTarget(pose)로 넘긴 목표 pose를 어느 frame 기준으로 해석할지. 기본은 planning frame이지만 setPoseReferenceFrame()으로 바꿀 수 있음
    const std::string arm_pose_frame = arm_group_->getPoseReferenceFrame();
    //pose 목표가 어느 링크를 그 위치로 보낼지, 즉 IK가 맞추는 tip 링크. SRDF group의 end effector에서 옴.
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

  // MoveGroupInterface 가 붙을 전용 helper 노드 생성(전역 인자 차단 + 필요한 파라미터만 주입).
  void create_moveit_helper_node() {
    if (moveit_node_)
    {
      return;
    }
    auto helper_options = node_options_;
    helper_options.use_global_arguments(false); // 전역 설정 인자를 모두 차단
    helper_options.arguments({});
    helper_options.automatically_declare_parameters_from_overrides(true);

    //launch의 --params-file로 받은 파라미터 오버라이드 전체를 꺼내(get_parameter_overrides()), helper 노드 옵션에 하나씩 복사해 넣는 과정
    const auto &overrides = this->get_node_parameters_interface()->get_parameter_overrides();
    for (const auto &[name, value] : overrides)
    {
      helper_options.append_parameter_override(rclcpp::Parameter(name, value));
    }

    moveit_node_ = std::make_shared<rclcpp::Node>(
        "motion_server_moveit", this->get_namespace(), helper_options);
    
    RCLCPP_INFO(
      get_logger(),
      "Created dedicated MoveIt helper node: '%s'",
      moveit_node_->get_fully_qualified_name());
  }

  // main 이 executor 에 추가할 수 있도록 helper 노드 핸들을 반환
  rclcpp::Node::SharedPtr get_moveit_helper_node() const
  {
    return moveit_node_;
  }

  // moveit_ready 파라미터를 갱신해 MoveIt 준비 상태를 외부에 알림
  void publish_moveit_ready(bool ready)
  {
    set_parameter(rclcpp::Parameter("moveit_ready", ready));
  }

   // MoveGroupInterface 연결 완료 여부 - main 의 readiness 폴링에 사용
  bool is_moveit_ready() const
  {
    return arm_group_ && !arm_group_->getPlanningFrame().empty() && gripper_group_ && !gripper_group_->getPlanningFrame().empty();
  }

private:
  rclcpp::NodeOptions node_options_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Node::SharedPtr moveit_node_;

  rclcpp_action::Server<MoveToNamed>::SharedPtr move_to_named_server_;
  rclcpp_action::Server<MoveToPose>::SharedPtr move_to_pose_server_;
  rclcpp_action::Server<MoveToJoints>::SharedPtr move_to_joints_server_;
  rclcpp_action::Server<GripperCmd>::SharedPtr gripper_server_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;

  std::thread arm_thread_;
  std::thread gripper_thread_;
  std::atomic<bool> arm_busy_{false};
  std::atomic<bool> gripper_busy_{false};

  std::string arm_tip_link_;

  // workspace 제한 (world frame 기준)
  double ws_x_abs_max_{0.28};
  double ws_y_abs_max_{0.28};
  double ws_z_min_{0.0};
  double ws_z_max_{0.45};
  std::string ws_frame_{"world"};

  // ── MoveIt 모션 튜닝 (yaml 외부화, 생성자에서 적재) ───────────────────
  double default_vel_scale_{0.3};
  double default_acc_scale_{0.1};
  double quat_norm_tol_{0.01};
  double arm_planning_time_{5.0};
  double arm_goal_position_tol_{0.001};
  double arm_goal_orientation_tol_{0.01};
  double arm_goal_joint_tol_{0.001};
  double gripper_planning_time_{3.0};
  double gripper_goal_joint_tol_{0.001};

  // ── compute_align_yaw 멤버 ────────────────────────────────────────────
  rclcpp::Service<ComputeAlignYaw>::SharedPtr compute_align_yaw_server_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::string align_world_frame_{"world"};
  double joint5_yaw_sign_{1.0};

  // ─────────────────────────────── RAII 헬퍼 ───────────────────────────────────────

  // atomic 플래그를 스코프 종료 시 false 로 해제
  struct BusyGuard
  {
    std::atomic<bool> &flag;
    ~BusyGuard() { flag.store(false, std::memory_order_release); }
  };

  // velocity scaling 을 스코프 종료 시 기본값(restore_scale)으로 복원
  // grp 가 nullptr 이면 복원을 건너뜀. restore_scale 은 default_vel_scale_
  // 파라미터를 호출부에서 주입(중첩 struct 라 바깥 멤버 직접 접근 불가)
  struct VelScaleGuard
  {
    moveit::planning_interface::MoveGroupInterface *grp;
    double restore_scale;
    ~VelScaleGuard()
    {
      if (grp)
        grp->setMaxVelocityScalingFactor(restore_scale);
    }
  };

  // ────────────────────────────── [omx/move_to_named 액션 관련 메서드들]──────────────────────────────────────

  // goal 콜백: 이름 유효성 검사와 arm 점유(CAS) 후 수락/거부를 결정
  rclcpp_action::GoalResponse handle_named_goal(const rclcpp_action::GoalUUID &,
                                                std::shared_ptr<const MoveToNamed::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "MovedToNamed : requested '%s'", goal->name.c_str());
    
    if (NAMED_POSE_MAP.find(goal->name) == NAMED_POSE_MAP.end())
    {
      RCLCPP_WARN(get_logger(), "MoveToNamed: unknown name '%s'", goal->name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!try_acquire_arm("MoveToNamed"))
      return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // handle_named_accepted 실행
  }

  // accepted 콜백: 실제 실행(execute_named)을 worker 스레드로 띄움
  void handle_named_accepted( const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToNamed>> goal_handle) {
    launch_arm_execution(goal_handle, [this](auto gh) { execute_named(gh); });
  }

  // worker 스레드 본체: setNamedTarget -> plan -> execute 로 named pose 이동을 수행
  void execute_named(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToNamed>> goal_handle)
  {
    BusyGuard busy_guard{arm_busy_}; // 모든 return 경로에서 arm_busy_ 해제 -> execute_named 함수가 실행되는 순간에 가드를 추가하여 어느 return; 상황에서도 소멸자가 호출되어 arm_busy_가 해제되도록 하는 코드
    auto result = std::make_shared<MoveToNamed::Result>();
    auto feedback = std::make_shared<MoveToNamed::Feedback>();

    const std::string &requested = goal_handle->get_goal()->name;
    const std::string &srdf_name = NAMED_POSE_MAP.at(requested);

    if (!arm_group_) {
      result->success = false;
      result->message = "MoveGroupInterface not initialized";
      goal_handle->abort(result);
      return;
    }

    feedback->status = "planning";
    goal_handle->publish_feedback(feedback);

    // 현재 조인트 상태로 시작점 동기화
    arm_group_->setStartStateToCurrentState();
    arm_group_->setNamedTarget(srdf_name);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned =
        (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!planned) {
      result->success = false;
      result->message = "Planning failed for '" + requested + "'";
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      return;
    }

    feedback->status = "executing";
    goal_handle->publish_feedback(feedback);

    finish_arm_motion<MoveToNamed>(
        goal_handle, plan, result,
        "Moved to '" + requested + "'",
        "MoveToNamed execution failed",
        [this, result]()
        { RCLCPP_INFO(get_logger(), "%s", result->message.c_str()); });

  }
  
  // ────────────────────────────────────────────────────────────────────────────────────────────────────────

  // ────────────────────────────── [omx/move_to_pose 액션 관련 메서드들]──────────────────────────────────────

  // goal 콜백: quaternion 정규화 + workspace 박스 검사 후 arm 점유(CAS) 로 수락/거부를 결정
  rclcpp_action::GoalResponse handle_pose_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const MoveToPose::Goal> goal)
  {
    const auto &ps = goal->target_pose;
    const auto &pos = ps.pose.position;
    const auto &ori = ps.pose.orientation;

    RCLCPP_INFO(get_logger(),
                "MoveToPose: target (%.3f, %.3f, %.3f) frame='%s' plan_only=%s",
                pos.x, pos.y, pos.z, ps.header.frame_id.c_str(),
                goal->plan_only ? "true" : "false");

    // frame_id 누락 경고 (비어있어도 MoveIt 기본 planning frame 으로 처리하므로 reject 않음)
    if (ps.header.frame_id.empty())
    {
      RCLCPP_WARN(get_logger(),
                  "MoveToPose: frame_id is empty — MoveIt will use the default planning frame");
    }

    // quaternion 정규화 검사: 비정규 값은 IK/planning 오류를 유발한다
    double qn = std::sqrt(
        ori.x * ori.x + ori.y * ori.y + ori.z * ori.z + ori.w * ori.w);
    if (std::abs(qn - 1.0) > quat_norm_tol_)
    {
      RCLCPP_WARN(get_logger(),
                  "MoveToPose: quaternion not normalized (norm=%.6f), rejecting", qn);
      return rclcpp_action::GoalResponse::REJECT;
    }

    // 워크스페이스 박스 경계 사전 거부 (execute_pose 와 동일한 단일 소스 함수/값)
    // pose frame 이 비어있거나 ws_frame_(기본 world) 일 때만 검사한다. 다른 frame 은
    // 이 단계에서 TF 변환 없이 좌표를 신뢰할 수 없어 execute_pose 단계로 미룬다
    const std::string &req_frame = ps.header.frame_id;
    if ((req_frame.empty() || req_frame == ws_frame_) &&
        !position_in_workspace_box(pos.x, pos.y, pos.z,
                                   ws_x_abs_max_, ws_y_abs_max_, ws_z_min_, ws_z_max_))
    {
      RCLCPP_WARN(get_logger(),
                  "MoveToPose: target out of workspace box: p=(%.3f,%.3f,%.3f) "
                  "limit |x|<=%.3f, |y|<=%.3f, %.3f<=z<=%.3f",
                  pos.x, pos.y, pos.z,
                  ws_x_abs_max_, ws_y_abs_max_, ws_z_min_, ws_z_max_);
      return rclcpp_action::GoalResponse::REJECT;
    }

    // arm 동시 실행 차단
    if (!try_acquire_arm("MoveToPose"))
      return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

    // accepted 콜백: 실제 실행(execute_pose)을 worker 스레드로 띄움
    void handle_pose_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPose>> goal_handle)
    {
      launch_arm_execution(goal_handle, [this](auto gh)
                          { execute_pose(gh); });
    }

    // worker 스레드 본체: workspace 재검증 -> IK(setJointValueTarget, 실패 시 근사) -> plan -> execute 로 pose 이동을 수행
    void execute_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPose>> goal_handle)
    {
      BusyGuard busy_guard{arm_busy_};           // arm_busy_ 해제
      VelScaleGuard vel_guard{arm_group_.get(), default_vel_scale_}; // velocity scaling 기본값 복원

      auto result = std::make_shared<MoveToPose::Result>();
      auto feedback = std::make_shared<MoveToPose::Feedback>();
      const auto &goal = goal_handle->get_goal();

      if (!arm_group_)
      {
        result->success = false;
        result->message = "MoveGroupInterface not initialized";
        goal_handle->abort(result);
        vel_guard.grp = nullptr; // 복원 불필요
        return;
      }

      // velocity_scale 적용 (0.01~1.0): VelScaleGuard 가 종료 시 default_vel_scale_ 로 복원
      // arm_busy_ 로 직렬화되어 있어 다른 arm goal 과 겹치지 않는다.
      double v_scale = std::clamp(static_cast<double>(goal->velocity_scale), 0.01, 1.0);
      arm_group_->setMaxVelocityScalingFactor(v_scale);

      const std::string target_link = arm_tip_link_;
      const std::string planning_frame = arm_group_->getPlanningFrame();
      const std::string pose_frame = arm_group_->getPoseReferenceFrame();
      const auto &target_pose = goal->target_pose.pose;

      RCLCPP_INFO(get_logger(),
                  "MoveToPose: planning_frame='%s' pose_reference_frame='%s' "
                  "target_link='%s'",
                  planning_frame.c_str(), pose_frame.c_str(),
                  target_link.empty() ? "<empty>" : target_link.c_str());

      RCLCPP_INFO(get_logger(),
                  "MoveToPose: target pose p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, "
                  "%.3f, %.3f)",
                  target_pose.position.x, target_pose.position.y,
                  target_pose.position.z, target_pose.orientation.x,
                  target_pose.orientation.y, target_pose.orientation.z,
                  target_pose.orientation.w);
      
      // ── Workspace 안전 제한 ─────────────────────────────────────────────
      // EE 가 멀리 뻗으면 shoulder(dxl12) 가 Overload 로 차단되어 로봇이 정지한다.
      // pose frame 이 ws_frame_ 과 일치할 때만 검증 (다른 frame 은 TF 변환 비용 회피).
      const std::string &req_frame = goal->target_pose.header.frame_id;
      
      if (req_frame.empty() || req_frame == ws_frame_ || req_frame == planning_frame)
      {
        if (!position_in_workspace_box(
                target_pose.position.x, target_pose.position.y, target_pose.position.z,
                ws_x_abs_max_, ws_y_abs_max_, ws_z_min_, ws_z_max_))
        {
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
      }
      else
      {
        RCLCPP_WARN(
            get_logger(),
            "MoveToPose: target frame_id='%s' differs from workspace frame='%s' / planning_frame='%s'; "
            "skipping workspace check",
            req_frame.c_str(), ws_frame_.c_str(), planning_frame.c_str());
      }
      
      const auto current_pose = target_link.empty()
                                    ? arm_group_->getCurrentPose()
                                    : arm_group_->getCurrentPose(target_link);
      const auto &current = current_pose.pose;
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
      feedback->status = "planning";
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

      if (!ik_ok)
      {
        RCLCPP_WARN(
            get_logger(),
            "MoveToPose: exact IK failed, trying approximate IK for target_link='%s'",
            target_link.empty() ? "<empty>" : target_link.c_str());
        ik_ok = target_link.empty()
                    ? arm_group_->setApproximateJointValueTarget(goal->target_pose)
                    : arm_group_->setApproximateJointValueTarget(goal->target_pose, target_link);
      }

      if (!ik_ok)
      {
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

      if (!planned)
      {
        result->success = false;
        result->message = "Planning failed";
        goal_handle->abort(result);
        RCLCPP_WARN(
            get_logger(),
            "MoveToPose: planning failed for target_link='%s' in planning_frame='%s'",
            target_link.empty() ? "<empty>" : target_link.c_str(),
            planning_frame.c_str());
        return; // guard 들이 velocity 복원 + busy 해제
      }

      feedback->progress = 0.5f;
      feedback->status = "planned";
      goal_handle->publish_feedback(feedback);

      if (goal->plan_only)
      {
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

      finish_arm_motion<MoveToPose>(
          goal_handle, plan, result,
          "Pose reached",
          "MoveToPose execution failed",
          [this, feedback, goal_handle]()
          {
            feedback->progress = 1.0f;
            feedback->status = "done";
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(get_logger(), "MoveToPose: succeeded");
          },
          [this]()
          { log_current_arm_state("MoveToPose post-execute (failed)"); });
    }

  // ────────────────────────────── [omx/move_to_joints 액션 관련 메서드들]──────────────────────────────────────

    // goal 콜백: positions/joint_names 크기 정합성 검사 후 arm 점유(CAS) 로 수락/거부를 결정
    rclcpp_action::GoalResponse handle_joints_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const MoveToJoints::Goal> goal)
  {
    if (goal->positions.empty())
    {
      RCLCPP_WARN(get_logger(), "MoveToJoints: positions is empty, rejecting");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!goal->joint_names.empty() &&
        goal->joint_names.size() != goal->positions.size())
    {
      RCLCPP_WARN(get_logger(),
                  "MoveToJoints: joint_names(%zu) and positions(%zu) size mismatch",
                  goal->joint_names.size(), goal->positions.size());
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_logger(),
                "MoveToJoints: %zu joints, velocity_scale=%.3f",
                goal->positions.size(), goal->velocity_scale);

    if (!try_acquire_arm("MoveToJoints"))
      return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // accepted 콜백: 실제 실행(execute_joints)을 worker 스레드로 띄움
  void handle_joints_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToJoints>> goal_handle)
  {
    launch_arm_execution(goal_handle, [this](auto gh)
                         { execute_joints(gh); });
  }

  // worker 스레드 본체: joint_names/positions 를 target map 으로 묶어 setJointValueTarget -> plan -> execute 로 joint 이동을 수행
  void execute_joints(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToJoints>> goal_handle)
  {
    BusyGuard busy_guard{arm_busy_};
    VelScaleGuard vel_guard{arm_group_.get(), default_vel_scale_};

    auto result = std::make_shared<MoveToJoints::Result>();
    auto feedback = std::make_shared<MoveToJoints::Feedback>();
    const auto &goal = goal_handle->get_goal();

    if (!arm_group_)
    {
      result->success = false;
      result->message = "MoveGroupInterface not initialized";
      goal_handle->abort(result);
      vel_guard.grp = nullptr;
      return;
    }

    // velocity_scale 적용 (0 또는 음수는 기본값 유지)
    if (goal->velocity_scale > 0.0f)
    {
      double v_scale = std::clamp(static_cast<double>(goal->velocity_scale), 0.01, 1.0);
      arm_group_->setMaxVelocityScalingFactor(v_scale);
    }

    // joint_names 가 비어있으면 arm 그룹의 active joints 사용
    std::vector<std::string> names = goal->joint_names;
    if (names.empty())
    {
      names = arm_group_->getActiveJoints();
      if (names.size() != goal->positions.size())
      {
        result->success = false;
        result->message = "joint_names empty but positions size(" + std::to_string(goal->positions.size()) + ") != active joints(" + std::to_string(names.size()) + ")";
        goal_handle->abort(result);
        RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
        return;
      }
    }

    std::map<std::string, double> target_map;
    for (std::size_t i = 0; i < names.size(); ++i)
    {
      target_map.emplace(names[i], goal->positions[i]);
    }

    feedback->progress = 0.0f;
    feedback->status = "planning";
    goal_handle->publish_feedback(feedback);

    arm_group_->setStartStateToCurrentState();

    if (!arm_group_->setJointValueTarget(target_map))
    {
      result->success = false;
      result->message = "Joint target out of limits or unknown joint name";
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "MoveToJoints: %s", result->message.c_str());
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!planned)
    {
      result->success = false;
      result->message = "Planning failed";
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "MoveToJoints: planning failed");
      return;
    }

    feedback->progress = 0.5f;
    feedback->status = "executing";
    goal_handle->publish_feedback(feedback);

    finish_arm_motion<MoveToJoints>(
        goal_handle, plan, result,
        "Moved to joint target",
        "MoveToJoints execution failed",
        [this, feedback, goal_handle]()
        {
          feedback->progress = 1.0f;
          feedback->status = "done";
          goal_handle->publish_feedback(feedback);
          RCLCPP_INFO(get_logger(), "MoveToJoints: succeeded");
        });
  }
  // ────────────────────────────────────────────────────────────────────────────────────────────────────────

  // ────────────────────────────── [omx/gripper_command 액션 관련 메서드들]──────────────────────────────────────

  // goal 콜백: gripper 점유(CAS) 로 동시 실행을 차단하고 수락/거부를 결정
  rclcpp_action::GoalResponse handle_gripper_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const GripperCmd::Goal> goal)
  {
    RCLCPP_INFO(get_logger(),
                "GripperCommand: position=%.3f, max_effort=%.1f",
                goal->position, goal->max_effort);

    bool expected = false;
    if (!gripper_busy_.compare_exchange_strong(
            expected, true, std::memory_order_acquire, std::memory_order_relaxed))
    {
      RCLCPP_WARN(get_logger(), "GripperCommand: gripper is busy, rejecting goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // cancel 콜백: 진행 중인 gripper MoveGroupInterface 실행을 stop() 으로 중단 요청
  rclcpp_action::CancelResponse handle_gripper_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCmd>>)
  {
    if (gripper_group_)
      gripper_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // accepted 콜백: 이전 gripper 스레드를 정리하고 실제 실행(execute_gripper)을 worker 스레드로 띄움
  void handle_gripper_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCmd>> goal_handle)
  {
    if (gripper_thread_.joinable())
      gripper_thread_.join();
    gripper_thread_ = std::thread([this, goal_handle]()
                                  { execute_gripper(goal_handle); });
  }

  // worker 스레드 본체: position 을 open/close 범위로 clamp 후 setJointValueTarget -> plan -> execute 로 그리퍼를 구동
  void execute_gripper(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCmd>> goal_handle)
  {
    BusyGuard busy_guard{gripper_busy_}; // 모든 return 경로에서 gripper_busy_ 해제

    auto result = std::make_shared<GripperCmd::Result>();
    auto feedback = std::make_shared<GripperCmd::Feedback>();
    const auto &goal = goal_handle->get_goal();

    if (!gripper_group_)
    {
      result->success = false;
      result->message = "MoveGroupInterface(gripper) not initialized";
      goal_handle->abort(result);
      return;
    }

    // max_effort 은 현재 미구현 — 컨트롤러 레벨 전류 제한에 의존한다.
    if (goal->max_effort > 0.0f)
    {
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

    if (!planned)
    {
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

    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "Cancelled";
      goal_handle->canceled(result);
      return;
    }

    result->success = executed;
    result->position = static_cast<float>(pos);
    result->message = executed
                          ? "Gripper command succeeded"
                          : execute_failure_message("Gripper execution failed", exec_code);

    if (executed)
    {
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "GripperCommand: pos=%.3f succeeded", pos);
    }
    else
    {
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "GripperCommand: %s", result->message.c_str());
    }
  }
  // ────────────────────────────────────────────────────────────────────────────────────────────────────────

  // ────────────────────────────── [omx/compute_align_yaw 서비스 관련 메서드들]──────────────────────────────────────

  // service 콜백: joint5 현재각 + TF jaw heading 으로 box_yaw 정렬에 필요한 joint5 목표각을 계산해 반환(모션 없음)
  void handle_compute_align(
      const std::shared_ptr<ComputeAlignYaw::Request> request,
      std::shared_ptr<ComputeAlignYaw::Response> response)
  {
    // 1. joint_states 스냅샷에서 joint5 현재각.
    sensor_msgs::msg::JointState::SharedPtr js;
    {
      std::lock_guard<std::mutex> lk(joint_state_mutex_);
      js = latest_joint_state_;
    }
    if (!js)
    {
      response->valid = false;
      response->message = "joint_states not received";
      return;
    }
    double j5_cur = 0.0;
    bool j5_found = false;
    for (std::size_t i = 0; i < js->name.size() && i < js->position.size(); ++i)
    {
      if (js->name[i] == "joint5")
      {
        j5_cur = js->position[i];
        j5_found = true;
        break;
      }
    }
    if (!j5_found)
    {
      response->valid = false;
      response->message = "joint5 not in joint_states";
      return;
    }

    // 2. TF 로 jaw heading 조회 (world <- arm_tip_link).
    geometry_msgs::msg::TransformStamped tf_stamped;
    try
    {
      tf_stamped = tf_buffer_->lookupTransform(
          align_world_frame_, arm_tip_link_,
          tf2::TimePointZero,
          tf2::durationFromSec(0.5));
    }
    catch (const tf2::TransformException & ex)
    {
      response->valid = false;
      response->message = std::string("TF lookup failed: ") + ex.what();
      return;
    }
    const auto & q = tf_stamped.transform.rotation;
    const double jaw_yaw = jaw_axis_yaw_from_quaternion(q.x, q.y, q.z, q.w);

    // 3. 90deg 대칭 적용한 최단 잔차 + joint5 절대 목표각.
    const double delta = wrap_to_pm45(request->box_yaw_rad - jaw_yaw);
    const double j5_target = j5_cur + joint5_yaw_sign_ * delta;

    response->valid = true;
    response->message = "";
    response->jaw_yaw_rad = jaw_yaw;
    response->joint5_current_rad = j5_cur;
    response->joint5_target_rad = j5_target;
    response->delta_rad = delta;

    RCLCPP_INFO(get_logger(),
        "compute_align_yaw: box_yaw=%.1f° jaw_yaw=%.1f° delta=%.1f° joint5 %.1f°→%.1f°",
        request->box_yaw_rad * 180.0 / M_PI,
        jaw_yaw * 180.0 / M_PI,
        delta * 180.0 / M_PI,
        j5_cur * 180.0 / M_PI,
        j5_target * 180.0 / M_PI);
  }

  // ────────────────────────────────────────────────────────────────────────────────────────────────────────

  // ────────────────────────────── [arm cancel/execute/finish template 구현체] ──────────────────────────────

  // cancel 콜백: 진행 중인 MoveGroupInterface 실행을 stop() 으로 중단 요청
  template <typename ActionT>
  rclcpp_action::CancelResponse handle_arm_cancel(const char *action_name, const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)
  {
    RCLCPP_INFO(get_logger(), "%s: cancel requested", action_name);
    if (arm_group_)
      arm_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // 이전 worker 스레드를 정리하고 새 스레드에서 exec 함수(execute_*)를 실행
  template <typename ActionT, typename ExecFn>
  void launch_arm_execution(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> &goal_handle, ExecFn exec_fn)
  {
    if (arm_thread_.joinable())
      arm_thread_.join();
    arm_thread_ = std::thread([ goal_handle, exec_fn ]()
                              { exec_fn(goal_handle); });
  }

  // plan 실행 후 공통 마무리: execute -> cancel 확인 -> succeed/abort (성공/실패 후처리는 콜백으로 주입).
  template <typename ActionT>
  void finish_arm_motion(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>
          &goal_handle,
      moveit::planning_interface::MoveGroupInterface::Plan &plan,
      const std::shared_ptr<typename ActionT::Result> &result,
      const std::string &success_msg, const std::string &fail_prefix,
      const std::function<void()> &on_success = {},
      const std::function<void()> &on_failure = {})
  {
    const auto exec_code = arm_group_->execute(plan);
    const bool executed = (exec_code == moveit::core::MoveItErrorCode::SUCCESS);
    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "Cancelled";
      goal_handle->canceled(result);
      return;
    }

    result->success = executed;
    if (executed)
    {
      result->message = success_msg;
      if (on_success)
        on_success();
      goal_handle->succeed(result);
    }
    else
    {
      result->message = execute_failure_message(fail_prefix, exec_code);
      if (on_failure)
        on_failure();
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    }
  }
  // ────────────────────────────────────────────────────────────────────────────────────────────────────────

  // ────────────────────────────── 헬퍼 함수들 ───────────────────────────────────────────────────────────────

  // action 실행 시 arm_busy 점유되어 있는지 판단 메서드 
  bool try_acquire_arm(const char *action_name)
  {
    bool expected = false;
    // arm_busy 가 false 이면, expected 와 동일 -> arm_busy를 true로 변경하고 true 반환 -> 따라서 최종 false 
    // arm_busy 가 true 이면, expected 와 다름 -> expected를 true로 변경하고 false 반환 -> 따라서 최종 true 
    if (!arm_busy_.compare_exchange_strong(
            expected, true, std::memory_order_acquire, std::memory_order_relaxed))
    {
      RCLCPP_WARN(get_logger(), "%s: arm is busy, rejecting goal", action_name);
      return false;
    }
    return true;
  }

  // 현재 네임스페이스를 MoveGroupInterface 가 기대하는 형식(루트면 빈 문자열)으로 변환
  std::string resolved_move_group_namespace() const
  {
    const std::string ns = this->get_namespace();
    return ns == "/" ? std::string() : ns;
  }

  // 파라미터 선언(중복 가드) + 값 조회를 한 번에 처리
  // automatically_declare_parameters_from_overrides(true) 로 yaml/-p 로 이미
  // 선언된 키는 declare 를 건너뛰어 ParameterAlreadyDeclaredException 피함
  template <typename T>
  T declare_and_get(const std::string &name, const T &default_value)
  {
    if (!has_parameter(name))
      declare_parameter<T>(name, default_value);
    return get_parameter(name).get_value<T>();
  }

  // 노드 파라미터 선언 + 멤버 적재를 한 곳에 모은다. 생성자에서 1회 호출.
  void declare_and_load_params()
  {
    declare_parameter<std::string>("arm_tip_link", "end_effector_link");
    declare_parameter<bool>("moveit_ready", false);

    // Workspace 사전 거부 한계 (world frame 기준).
    // EE 가 x,y 방향으로 멀리 뻗으면 dxl12 (shoulder) 의 중력 모멘트가 급증해
    // Overload Hardware Error 로 모터가 차단된다. 그 영역의 goal pose 를 사전에 거부한다.
    // 같은 workspace 값은 omx_bringup 의 workspace_guard 와 공유한다.
    // 단일 설정 소스: omx_bringup/config/omx_f/workspace.yaml.
    // goal 사전 거부(handle_pose_goal)와 execute 직전 검사(execute_pose)가 같은 값을 쓴다.
    ws_x_abs_max_ = declare_and_get<double>("workspace.x_abs_max", 0.3);
    ws_y_abs_max_ = declare_and_get<double>("workspace.y_abs_max", 0.3);
    ws_z_min_ = declare_and_get<double>("workspace.z_min", 0.0);
    ws_z_max_ = declare_and_get<double>("workspace.z_max", 0.45);
    ws_frame_ = declare_and_get<std::string>("workspace.frame", "world");

    // MoveIt 모션 튜닝: velocity/acceleration scaling, quaternion 허용 오차,
    // planning time, goal tolerance. init_moveit() 에서 MoveGroupInterface 에
    // 적용하고 VelScaleGuard 복원값으로도 쓴다.
    default_vel_scale_ = declare_and_get<double>("default_vel_scale", 0.3);
    default_acc_scale_ = declare_and_get<double>("default_acc_scale", 0.1);
    quat_norm_tol_ = declare_and_get<double>("quat_norm_tol", 0.01);
    arm_planning_time_ = declare_and_get<double>("arm.planning_time", 5.0);
    arm_goal_position_tol_ = declare_and_get<double>("arm.goal_position_tol", 0.001);
    arm_goal_orientation_tol_ = declare_and_get<double>("arm.goal_orientation_tol", 0.01);
    arm_goal_joint_tol_ = declare_and_get<double>("arm.goal_joint_tol", 0.001);
    gripper_planning_time_ = declare_and_get<double>("gripper.planning_time", 3.0);
    gripper_goal_joint_tol_ = declare_and_get<double>("gripper.goal_joint_tol", 0.001);

    // compute_align_yaw service 파라미터. service 는 생성자에서 즉시 살아나므로
    // (init_moveit 완료를 기다리지 않음) 여기서 적재해야 이른 호출에도 yaml 값을
    // 쓴다. init_moveit 에서 늦게 적재하면 그 사이 호출은 멤버 기본값(부호 1.0)을
    // 써서 정렬 방향이 뒤집힐 수 있다.
    align_world_frame_ = declare_and_get<std::string>("align_world_frame", "world");
    joint5_yaw_sign_ = declare_and_get<double>("joint5_yaw_sign", 1.0);
  }

    // execute 직전/직후 조인트 상태 스냅샷.
  // plan 의 start state 와 controller current state 가 drift 하면 MoveIt 은
  // CONTROL_FAILED 로 goal 을 abort 한다. 이 값을 로그에 남겨두면 원인 분석이 쉬워진다.
  void log_current_arm_state(const std::string &tag) const
  {
    if (!arm_group_)
    {
      return;
    }
    auto state = arm_group_->getCurrentState(0.5); // 0.5s wait
    if (!state)
    {
      RCLCPP_WARN(get_logger(),
                  "%s: getCurrentState() returned null - TF/joint_state 누락 가능",
                  tag.c_str());
      return;
    }
    std::vector<double> joints;
    state->copyJointGroupPositions("arm", joints);
    std::ostringstream oss;
    oss << tag << ": joints=[";
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
      if (i > 0)
        oss << ", ";
      oss << std::fixed << std::setprecision(4) << joints[i];
    }
    oss << "]";

    try
    {
      const auto pose = arm_tip_link_.empty()
                            ? arm_group_->getCurrentPose()
                            : arm_group_->getCurrentPose(arm_tip_link_);
      const auto &p = pose.pose.position;
      const auto &q = pose.pose.orientation;
      oss << " pose=(" << std::fixed << std::setprecision(4)
          << p.x << ", " << p.y << ", " << p.z << ")"
          << " q=(" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << ")";
    }
    catch (const std::exception &e)
    {
      oss << " pose=<err: " << e.what() << ">";
    }
    RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  }
  // ────────────────────────────────────────────────────────────────────────────────────────────────────────
};

// executor.spin() 콜백 처리 루프를 도는 spin 스레드 진입점.
void run_spin(rclcpp::executors::MultiThreadedExecutor *exec) { exec->spin(); }

// MoveGroupInterface 가 연결될 때까지 init_moveit 을 재시도하는 init 스레드 진입점.
void init_moveit_loop(std::shared_ptr<MotionServer> node) {
  constexpr auto kPollInterval = std::chrono::milliseconds(200);
  constexpr auto kPollAttempts = 50;                 // 약 10초
  constexpr auto kRetryInterval = std::chrono::seconds(2);
  while (rclcpp::ok()) {
    node->publish_moveit_ready(false);
    try {
      node->init_moveit();
    } catch (const std::exception &e) {
      RCLCPP_WARN(node->get_logger(), "init_moveit threw: %s — retrying in 2s",
                  e.what());
      std::this_thread::sleep_for(kRetryInterval);
      continue;
    }
    for (int i = 0; i < kPollAttempts && rclcpp::ok(); ++i) {
        if (node->is_moveit_ready()) {
          node->publish_moveit_ready(true);
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
}

// 노드/executor 구성 후 spin 과 MoveIt 초기화를 각각 스레드로 돌리고 종료까지 대기
int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<MotionServer>(options);
  node->create_moveit_helper_node();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(node->get_moveit_helper_node());

  std::thread spin_thread(run_spin, &executor);
  std::thread init_thread(init_moveit_loop, node);
  
  if(spin_thread.joinable())spin_thread.join();
  if(init_thread.joinable())init_thread.join();
  rclcpp::shutdown();
  return 0;
}