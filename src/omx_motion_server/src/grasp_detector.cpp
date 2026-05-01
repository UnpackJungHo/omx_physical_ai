// grasp_detector — Step 1: Dynamixel current 기반 grasp 판단
//
// 목적
//   IL 데이터 라벨링 / RL 보상 신호 / classical pick & place retry 로직의
//   전제 조건이 되는 "그리퍼가 박스를 잡았는가?" 신호를 제공한다.
//
// 판단 로직 (3-신호 AND, stable_window_ms 안정화)
//   1. |present_current| > current_thresh_ma
//   2. |goal_pos − present_pos| > position_error_thresh
//   3. |present_velocity| < velocity_thresh
//   세 조건이 stable_window_ms 동안 연속 만족하면 is_grasping = true.
//
// 입력
//   /joint_states                                (sensor_msgs/JointState)
//     - effort[gripper] 가 dynamixel_hardware_interface 의
//       Present Current (Dynamixel raw unit) 로 매핑되어 발행된다.
//   /gripper_traj_controller/controller_state    (control_msgs/JointTrajectoryControllerState)
//     - reference.positions[gripper] = 최신 goal position.
//
// 출력
//   /gripper/is_grasping                         (std_msgs/Bool, transient_local)
//   /gripper/grasp_force_estimate                (std_msgs/Float32, mA)
//   /gripper/check_grasp                         (omx_interfaces/srv/CheckGrasp)
//
// 동시성
//   기본 SingleThreadedExecutor 한 개에 모든 콜백을 묶는다.
//   (joint_state, controller_state, service 모두 짧은 콜백 — lock 한 개로 충분)

#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <omx_interfaces/srv/check_grasp.hpp>

namespace omx
{

class GraspDetector : public rclcpp::Node
{
public:
  GraspDetector()
  : rclcpp::Node("grasp_detector")
  {
    // ── 파라미터 ────────────────────────────────────────────────────
    gripper_joint_ = declare_parameter<std::string>("gripper_joint", "gripper_joint_1");
    current_unit_to_ma_ = declare_parameter<double>("current_unit_to_ma", 2.69);
    current_thresh_ma_ = declare_parameter<double>("current_thresh_ma", 70.0);
    position_error_thresh_ = declare_parameter<double>("position_error_thresh", 0.03);
    velocity_thresh_ = declare_parameter<double>("velocity_thresh", 0.05);
    stable_window_ms_ = declare_parameter<int>("stable_window_ms", 150);
    state_stale_ms_ = declare_parameter<int>("state_stale_ms", 200);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);

    const std::string controller_state_topic = declare_parameter<std::string>(
      "controller_state_topic",
      "/gripper_traj_controller/controller_state");

    RCLCPP_INFO(get_logger(),
      "grasp_detector params: joint=%s current_thresh=%.1fmA pos_err=%.3f vel=%.3f "
      "stable=%dms stale=%dms unit_to_ma=%.3f",
      gripper_joint_.c_str(), current_thresh_ma_, position_error_thresh_,
      velocity_thresh_, stable_window_ms_, state_stale_ms_, current_unit_to_ma_);

    // ── publishers ─────────────────────────────────────────────────
    // is_grasping 은 latched (transient_local) — 늦게 뜨는 구독자도 마지막 값을 본다.
    rclcpp::QoS latched_qos(1);
    latched_qos.reliable();
    latched_qos.transient_local();
    is_grasping_pub_ = create_publisher<std_msgs::msg::Bool>("/gripper/is_grasping", latched_qos);

    // 힘 추정값은 일반 sensor 스트림 — best effort, depth 10.
    rclcpp::QoS sensor_qos(10);
    sensor_qos.best_effort();
    force_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/gripper/grasp_force_estimate", sensor_qos);

    // ── subscriptions ──────────────────────────────────────────────
    rclcpp::QoS js_qos(rclcpp::KeepLast(50));
    js_qos.best_effort();
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", js_qos,
      std::bind(&GraspDetector::on_joint_state, this, std::placeholders::_1));

    rclcpp::QoS cs_qos(rclcpp::KeepLast(10));
    cs_qos.reliable();
    controller_state_sub_ =
      create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
      controller_state_topic, cs_qos,
      std::bind(&GraspDetector::on_controller_state, this, std::placeholders::_1));

    // ── service ────────────────────────────────────────────────────
    check_grasp_srv_ = create_service<omx_interfaces::srv::CheckGrasp>(
      "/gripper/check_grasp",
      std::bind(&GraspDetector::on_check_grasp, this,
        std::placeholders::_1, std::placeholders::_2));

    // ── 주기적 평가 타이머 ───────────────────────────────────────────
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&GraspDetector::on_timer, this));
  }

private:
  // ── 한 sample 의 평가 결과 ───────────────────────────────────────
  struct SampleEval
  {
    bool valid;          // 모든 입력이 살아있고 정상이면 true
    bool conditions_met; // 3-신호 AND
    std::string reason;  // 평가 사유
    double current_ma;
    double position_error;
    double velocity;
    rclcpp::Time stamp;
  };

  // ────────────────────────────────────────────────────────────────
  // /joint_states 콜백: gripper_joint_1 의 position / velocity / effort 추출
  // ────────────────────────────────────────────────────────────────
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto it = std::find(msg->name.begin(), msg->name.end(), gripper_joint_);
    if (it == msg->name.end()) {
      return;  // gripper joint 가 아직 안 나타남
    }
    const size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));

    // effort 가 비어있으면 — URDF 가 effort state interface 를 export 하지 않은 것.
    // 이 경우 grasp 판단 자체가 불가능하므로 경고를 throttle 로 띄우고 무시.
    if (msg->effort.size() <= idx) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "[grasp_detector] /joint_states.effort is empty — gripper effort state "
        "is not exposed by hardware interface. URDF 의 ros2_control state_interface 에 "
        "effort 가 추가되어 있는지 확인하라.");
      return;
    }
    if (msg->position.size() <= idx || msg->velocity.size() <= idx) {
      return;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    last_position_ = msg->position[idx];
    last_velocity_ = msg->velocity[idx];
    last_current_unit_ = msg->effort[idx];
    last_state_stamp_ = msg->header.stamp;
    has_state_ = true;
  }

  // ────────────────────────────────────────────────────────────────
  // /gripper_traj_controller/controller_state 콜백: 최신 goal position 추출
  // ────────────────────────────────────────────────────────────────
  void on_controller_state(
    const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
  {
    auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), gripper_joint_);
    if (it == msg->joint_names.end()) {
      return;
    }
    const size_t idx = static_cast<size_t>(std::distance(msg->joint_names.begin(), it));
    if (msg->reference.positions.size() <= idx) {
      return;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    last_goal_position_ = msg->reference.positions[idx];
    last_goal_stamp_ = msg->header.stamp;
    has_goal_ = true;
  }

  // ────────────────────────────────────────────────────────────────
  // 한 시점 평가 — 락 잡은 채로 호출되어야 한다.
  // ────────────────────────────────────────────────────────────────
  SampleEval evaluate_locked() const
  {
    SampleEval e{};
    e.valid = false;
    e.conditions_met = false;
    e.current_ma = 0.0;
    e.position_error = 0.0;
    e.velocity = 0.0;
    e.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);

    if (!has_state_) {
      e.reason = "no_state";
      return e;
    }

    // joint_states timestamp staleness 검사.
    const rclcpp::Time now = get_clock()->now();
    rclcpp::Time state_stamp(last_state_stamp_, now.get_clock_type());
    const auto age = now - state_stamp;
    if (age > rclcpp::Duration(std::chrono::milliseconds(state_stale_ms_))) {
      e.reason = "stale_state";
      e.stamp = state_stamp;
      return e;
    }

    e.current_ma = last_current_unit_ * current_unit_to_ma_;
    e.velocity = last_velocity_;
    e.stamp = state_stamp;

    if (!has_goal_) {
      // goal 이 없으면 position error 비교가 불가 — grasping 은 false 로 단정.
      e.reason = "no_command";
      e.valid = true;
      return e;
    }
    e.position_error = last_goal_position_ - last_position_;

    const double abs_current = std::abs(e.current_ma);
    const double abs_pos_err = std::abs(e.position_error);
    const double abs_vel = std::abs(e.velocity);

    const bool current_high = abs_current > current_thresh_ma_;
    const bool blocked_position = abs_pos_err > position_error_thresh_;
    const bool stationary = abs_vel < velocity_thresh_;

    e.valid = true;
    e.conditions_met = current_high && blocked_position && stationary;
    if (e.conditions_met) {
      e.reason = "grasping";
    } else if (!stationary) {
      e.reason = "moving";
    } else if (!current_high) {
      e.reason = "no_object";
    } else {
      // current 는 높은데 position_error 가 작음 — goal 에 도달했다는 뜻.
      e.reason = "no_object";
    }
    return e;
  }

  // ────────────────────────────────────────────────────────────────
  // 평가 + stable window — 윈도우 내 모든 sample 이 conditions_met 이면 latched true.
  // ────────────────────────────────────────────────────────────────
  bool update_history_and_decide(const SampleEval & e)
  {
    // history_ 에 (stamp, conditions_met) 푸시.
    const rclcpp::Time now = get_clock()->now();
    history_.push_back({now, e.conditions_met});
    const auto window = rclcpp::Duration(std::chrono::milliseconds(stable_window_ms_));
    while (!history_.empty() && (now - history_.front().t) > window) {
      history_.pop_front();
    }
    if (history_.empty()) {
      return false;
    }
    // 모든 sample 이 true 여야 한다.
    for (const auto & h : history_) {
      if (!h.met) {
        return false;
      }
    }
    // 그리고 윈도우 자체가 충분히 채워져 있어야 한다 (방금 시작한 노드가 한 sample 만 보고
    // 즉시 true 되는 걸 막는다).
    const auto span = history_.back().t - history_.front().t;
    if (span < window * 0.5) {
      return false;
    }
    return true;
  }

  // ────────────────────────────────────────────────────────────────
  // 주기 타이머: 평가 → latched publish.
  // ────────────────────────────────────────────────────────────────
  void on_timer()
  {
    SampleEval e;
    bool grasping;
    double current_ma;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      e = evaluate_locked();
      if (!e.valid) {
        // 입력이 아직 없거나 stale — history 를 비워서 false 로 락한다.
        history_.clear();
        grasping = false;
      } else {
        grasping = update_history_and_decide(e);
      }
      current_ma = e.current_ma;
    }

    // is_grasping latched publish — 값이 바뀌었거나 아직 한 번도 안 발행했을 때만.
    if (!last_published_ || *last_published_ != grasping) {
      std_msgs::msg::Bool b;
      b.data = grasping;
      is_grasping_pub_->publish(b);
      last_published_ = grasping;
      RCLCPP_INFO(get_logger(),
        "[grasp_detector] is_grasping=%s reason=%s current=%.1fmA pos_err=%.3f vel=%.3f",
        grasping ? "true" : "false", e.reason.c_str(),
        e.current_ma, e.position_error, e.velocity);
    }

    std_msgs::msg::Float32 f;
    f.data = static_cast<float>(current_ma);
    force_pub_->publish(f);
  }

  // ────────────────────────────────────────────────────────────────
  // /gripper/check_grasp 서비스 — 즉시 평가.
  // ────────────────────────────────────────────────────────────────
  void on_check_grasp(
    const std::shared_ptr<omx_interfaces::srv::CheckGrasp::Request> /*req*/,
    std::shared_ptr<omx_interfaces::srv::CheckGrasp::Response> res)
  {
    SampleEval e;
    bool grasping_stable;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      e = evaluate_locked();
      grasping_stable = e.valid && update_history_and_decide(e);
    }
    res->is_grasping = grasping_stable;
    res->current_ma = static_cast<float>(e.current_ma);
    res->position_error = static_cast<float>(e.position_error);
    res->velocity = static_cast<float>(e.velocity);
    res->reason = e.reason;
    res->stamp = e.stamp;
  }

  // ── 파라미터 ─────────────────────────────────────────────────────
  std::string gripper_joint_;
  double current_unit_to_ma_;
  double current_thresh_ma_;
  double position_error_thresh_;
  double velocity_thresh_;
  int stable_window_ms_;
  int state_stale_ms_;
  double publish_rate_hz_;

  // ── 상태 ────────────────────────────────────────────────────────
  std::mutex mtx_;
  bool has_state_{false};
  bool has_goal_{false};
  double last_position_{0.0};
  double last_velocity_{0.0};
  double last_current_unit_{0.0};
  double last_goal_position_{0.0};
  builtin_interfaces::msg::Time last_state_stamp_;
  builtin_interfaces::msg::Time last_goal_stamp_;

  struct HistEntry { rclcpp::Time t; bool met; };
  std::deque<HistEntry> history_;
  std::optional<bool> last_published_;

  // ── ROS 인터페이스 ───────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr
    controller_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_grasping_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr force_pub_;
  rclcpp::Service<omx_interfaces::srv::CheckGrasp>::SharedPtr check_grasp_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace omx

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<omx::GraspDetector>());
  rclcpp::shutdown();
  return 0;
}
