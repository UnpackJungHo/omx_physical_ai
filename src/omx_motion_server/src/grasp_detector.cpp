// grasp_detector — Dynamixel current 기반 grasp 판단 (force-only)
//
// 목적
//   IL 데이터 라벨링 / RL 보상 신호 / pick & place retry 로직의 전제 조건이
//   되는 "그리퍼가 박스를 잡았는가?" 신호를 제공한다.
//
// 판단 로직 (force-only, stable_window_ms 안정화)
//   force_estimate = effort × current_unit_to_ma_ (signed mA)
//   force_estimate < grasp_force_threshold_ma  (예: < -500 mA)
//   이 조건이 stable_window_ms 동안 연속 만족하면 is_grasping = true.
//
//   signed 음수 임계값을 쓰는 이유: 클로즈 방향으로 작용하는 dynamixel current
//   는 음수로 측정된다 (실측 -1150 mA 부근). 절대값이 아닌 signed 비교로
//   close 방향 부하만 잡고, open/리액션 토크의 양수 부하는 grasp 로 보지 않는다.
//
// 입력
//   /joint_states  (sensor_msgs/JointState)
//     effort[gripper] 가 dynamixel_hardware_interface 의 Present Current
//     (Dynamixel raw unit) 로 매핑되어 발행된다.
//
// 출력
//   /gripper/is_grasping            (std_msgs/Bool, transient_local)
//   /gripper/grasp_force_estimate   (std_msgs/Float32, mA)  ← raw stream, 외부
//                                    그래프 클라이언트가 직접 구독한다. 토픽
//                                    이름/타입/QoS/발행 주기/값 정의는 절대
//                                    바꾸지 말 것 (omx_web_ws 차트 의존).
//   /gripper/check_grasp            (omx_interfaces/srv/CheckGrasp)
//
// 동시성
//   기본 SingleThreadedExecutor 한 개에 모든 콜백을 묶는다.

#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
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
    grasp_force_threshold_ma_ =
      declare_parameter<double>("grasp_force_threshold_ma", -500.0);
    stable_window_ms_ = declare_parameter<int>("stable_window_ms", 150);
    state_stale_ms_ = declare_parameter<int>("state_stale_ms", 200);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 20.0);

    RCLCPP_INFO(get_logger(),
      "grasp_detector params: joint=%s force_threshold=%.1fmA "
      "stable=%dms stale=%dms unit_to_ma=%.3f",
      gripper_joint_.c_str(), grasp_force_threshold_ma_,
      stable_window_ms_, state_stale_ms_, current_unit_to_ma_);

    // ── publishers ─────────────────────────────────────────────────
    // is_grasping 은 latched (transient_local) — 늦게 뜨는 구독자도 마지막 값을 본다.
    rclcpp::QoS latched_qos(1);
    latched_qos.reliable();
    latched_qos.transient_local();
    is_grasping_pub_ = create_publisher<std_msgs::msg::Bool>("gripper/is_grasping", latched_qos);

    // 힘 추정값은 일반 sensor 스트림 — best effort, depth 10.
    // omx_web_ws 차트가 이 토픽을 직접 구독하므로 이름/타입/QoS/발행 주기를
    // 변경하면 안 된다.
    rclcpp::QoS sensor_qos(10);
    sensor_qos.best_effort();
    force_pub_ = create_publisher<std_msgs::msg::Float32>(
      "gripper/grasp_force_estimate", sensor_qos);

    // ── subscriptions ──────────────────────────────────────────────
    rclcpp::QoS js_qos(rclcpp::KeepLast(50));
    js_qos.best_effort();
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", js_qos,
      std::bind(&GraspDetector::on_joint_state, this, std::placeholders::_1));

    // ── service ────────────────────────────────────────────────────
    check_grasp_srv_ = create_service<omx_interfaces::srv::CheckGrasp>(
      "gripper/check_grasp",
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
    bool valid{false};            // 모든 입력이 살아있고 정상이면 true
    bool conditions_met{false};   // force_estimate < threshold
    std::string reason;           // 평가 사유
    double current_ma{0.0};
    // rclcpp::Time 의 기본 생성자가 explicit 이라 SampleEval e{} value-init
    // 시 implicit default ctor 사용이 막힌다. 명시적 기본값으로 회피.
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  // ────────────────────────────────────────────────────────────────
  // /joint_states 콜백: gripper_joint_1 의 effort 추출
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

    std::lock_guard<std::mutex> lk(mtx_);
    last_current_unit_ = msg->effort[idx];
    last_state_stamp_ = msg->header.stamp;
    has_state_ = true;
  }

  // ────────────────────────────────────────────────────────────────
  // 한 시점 평가 — 락 잡은 채로 호출되어야 한다.
  // ────────────────────────────────────────────────────────────────
  SampleEval evaluate_locked() const
  {
    SampleEval e;  // 멤버 기본값으로 초기화됨

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
    e.stamp = state_stamp;
    e.valid = true;
    // signed 비교 — close 방향 토크가 음수로 측정된다는 캘리브레이션 전제.
    e.conditions_met = e.current_ma < grasp_force_threshold_ma_;
    e.reason = e.conditions_met ? "grasping" : "no_object";
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
        "[grasp_detector] is_grasping=%s reason=%s current=%.1fmA",
        grasping ? "true" : "false", e.reason.c_str(), e.current_ma);
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
    // position_error / velocity 는 force-only 판단에서 사용하지 않는다.
    // srv 호환을 위해 0 으로 채운다.
    res->position_error = 0.0f;
    res->velocity = 0.0f;
    res->reason = e.reason;
    res->stamp = e.stamp;
  }

  // ── 파라미터 ─────────────────────────────────────────────────────
  std::string gripper_joint_;
  double current_unit_to_ma_;
  double grasp_force_threshold_ma_;
  int stable_window_ms_;
  int state_stale_ms_;
  double publish_rate_hz_;

  // ── 상태 ────────────────────────────────────────────────────────
  std::mutex mtx_;
  bool has_state_{false};
  double last_current_unit_{0.0};
  builtin_interfaces::msg::Time last_state_stamp_;

  struct HistEntry { rclcpp::Time t; bool met; };
  std::deque<HistEntry> history_;
  std::optional<bool> last_published_;

  // ── ROS 인터페이스 ───────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
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
