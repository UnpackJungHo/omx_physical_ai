#include "omx_perception_world_pose/box_cup_world_pose_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

// yaml-cpp is only needed for intrinsics loading
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <yaml-cpp/yaml.h>
#pragma GCC diagnostic pop

using namespace std::chrono_literals;

namespace omx_perception_world_pose
{

namespace
{

// ---------- quaternion helpers ----------

cv::Matx33d quat_to_rot(
  double qx, double qy, double qz, double qw)
{
  double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (n < 1e-9) {
    return cv::Matx33d::eye();
  }
  qx /= n; qy /= n; qz /= n; qw /= n;

  return cv::Matx33d(
    1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw),
    2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw),
    2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy));
}

geometry_msgs::msg::Quaternion yaw_to_quat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

geometry_msgs::msg::Quaternion identity_quat()
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 0.0;
  q.w = 1.0;
  return q;
}

// ---------- ray-plane intersection ----------

// [기능] 이미지 픽셀 keypoint 들을 "world 수평평면 z=plane_z 위의 3D 점" 으로 매핑한다.
// 절차: 픽셀 → (K,D 로 왜곡 보정) normalized camera coord → world frame 광선 → 수평 plane 과의 교점 
// PnP 와 달리 객체 모델 크기에 의존하지 않으므로 작은 객체에서
// depth ambiguity 가 (x,y) 로 전이되지 않는다. 대신 plane_z(상자 = 3cm, 종이컵 = 8cm) 와 TF 정확도가 새 병목.
// 실패 시(광선이 수평/카메라 뒤) std::nullopt.
std::optional<std::vector<cv::Point3d>> project_keypoints_to_plane(
  const std::vector<cv::Point2f> & image_points,        // YOLO 픽셀 keypoint 들 (u,v)
  const cv::Mat & K, const cv::Mat & D,                 // 카메라 내부행렬 + 왜곡계수
  const cv::Matx33d & R_world_cam, const cv::Vec3d & t_world_cam,  // TF: world←camera
  double plane_z)                                       // 교점 평면의 world z (박스 윗면/컵 입구)
{
  if (image_points.empty()) {return std::nullopt;}      // 입력 픽셀 없으면 처리 불가

  std::vector<cv::Point2f> undistorted;
  cv::undistortPoints(image_points, undistorted, K, D); // 픽셀 → normalized (x_n, y_n) = (X_c/Z_c, Y_c/Z_c)

  std::vector<cv::Point3d> world_points;
  world_points.reserve(image_points.size());            // 결과 벡터 capacity 미리 확보

  constexpr double kEpsDz = 1e-6;                       // 광선 수평 판정 epsilon (분모 0 회피)

  for (const auto & p : undistorted) {                  // 각 keypoint 별로 ray-plane 교점 계산
    cv::Vec3d d_cam(p.x, p.y, 1.0);                     // 카메라 frame 광선 방향: (x_n, y_n, 1)
    cv::Vec3d d_world = R_world_cam * d_cam;            // 광선을 world frame 으로 회전 (R · d_cam)

    if (std::abs(d_world[2]) < kEpsDz) {return std::nullopt;}  // d_world.z ≈ 0 → 광선이 수평 → 교점 무한대
    double lambda = (plane_z - t_world_cam[2]) / d_world[2];   // (z₀ - t_z) / d_z. 광선 매개변수 λ
    if (lambda <= 0.0) {return std::nullopt;}                  // λ≤0 → 교점이 카메라 뒤 (비물리적)

    cv::Vec3d hit = t_world_cam + lambda * d_world;     // 교점: p(λ) = t + λ·d (world 좌표)
    world_points.emplace_back(hit[0], hit[1], hit[2]);  // (x, y, z=plane_z) 누적
  }
  return world_points;                                  // 모든 keypoint 의 world 3D 좌표 반환
}

// [기능] 3D 점들의 (x, y) 산술 평균을 반환한다. z 성분은 무시.
// project_keypoints_to_plane 결과(같은 plane_z 위 4 모서리)에 적용하면 그 평면상
// 중심점이 된다. 박스/컵이 좌우 대칭으로 잡히기만 하면 개별 모서리의 픽셀 오차가
// 평균에서 상쇄되므로, 객체 모델 크기를 몰라도 중심 위치를 안정적으로 얻는다.
cv::Point2d centroid_xy(const std::vector<cv::Point3d> & pts)
{
  double sx = 0.0, sy = 0.0;                                  // x, y 누적합 (z 는 사용 안 함)
  for (const auto & p : pts) {sx += p.x; sy += p.y;}          // 모든 점의 x, y 더하기
  const double n = static_cast<double>(pts.size());           // 개수 (0 가정하지 않음 → 호출부에서 비어있지 않음 보장)
  return {sx / n, sy / n};                                    // 평균 (sx/n, sy/n) = 평면 중심 (x̄, ȳ)
}

// ---------- box yaw from 4 planar corners ----------

// [기능] 박스 윗면 4 모서리(world 평면 위)로부터 yaw 각도를 추정한다.
// 입력 corner 순서는 (TL, TR, BR, BL). 이웃 모서리 edge 의 object-frame 방향은
// i=0:+x, i=1:+y, i=2:-x, i=3:-y 로 고정이므로, 각 edge 의 world 방향각에서
// object 방향각을 빼면 yaw 한 개 추정치가 나온다. 4개 edge 추정치의 circular
// mean (sin/cos 평균 후 atan2) 으로 노이즈를 평탄화한다.
double box_yaw_from_corners(const std::vector<cv::Point3d> & p)
{
  if (p.size() < 4) {return 0.0;}                      // 4 모서리 미만이면 yaw 계산 불가 → 0 반환
  static constexpr double kObjEdgeAngles[4] = {        // object frame 의 edge 방향각 (i=0..3 순서대로)
    0.0, M_PI / 2.0, M_PI, -M_PI / 2.0,                // +x(0°), +y(90°), -x(180°), -y(-90°)
  };
  double s_sum = 0.0, c_sum = 0.0;                     // circular mean 용 sin/cos 누적합
  for (int i = 0; i < 4; ++i) {                        // 4 개 edge 각각에서 yaw 추정
    double dx = p[(i + 1) % 4].x - p[i].x;             // edge 의 world x 성분 (p[i] → p[i+1])
    double dy = p[(i + 1) % 4].y - p[i].y;             // edge 의 world y 성분
    double yaw = std::atan2(dy, dx) - kObjEdgeAngles[i];  // world 방향각 - object 방향각 = yaw 추정치
    s_sum += std::sin(yaw);                            // sin 누적 (각도 평균을 직접 하면 wrap 문제)
    c_sum += std::cos(yaw);                            // cos 누적
  }
  return std::atan2(s_sum, c_sum);                     // atan2(Σsin, Σcos) = circular mean yaw
}

// ---------- yaw normalization & circular median ----------

// [기능] 임의 yaw 값을 [0, π/2) 구간으로 정규화한다 (90° 주기 modulo).
// 정육면체 박스는 yaw 가 90° 회전 대칭이라 yaw 와 yaw±π/2 가 같은 자세이므로
// 대표 구간 하나로 줄여도 정보 손실이 없다. 0 이 "박스 모서리가 world x 축과
// 평행" 자세에 대응해 사람이 읽기 쉽고, 하류 skill_executor 가 다시
// [-45°,45°] 로 wrap 하므로 본 구간 선택이 grasp 정렬에 영향을 주지 않는다.
double wrap_yaw_zero_pi_over_2(double yaw)
{
  constexpr double kPeriod = M_PI / 2.0;     // 90° (박스 회전 대칭 주기)
  double y = std::fmod(yaw, kPeriod);        // yaw mod 90° → (-90°, 90°) 사이 값
  if (y < 0.0) {y += kPeriod;}               // 음수면 +90° 더해 [0°, 90°) 으로 끌어올림
  return y;                                  // 결과는 항상 [0, π/2) 구간
}

double median_sorted(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const size_t n = values.size();
  if (n == 0) {return 0.0;}
  if (n % 2 == 1) {return values[n / 2];}
  return 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

// [기능] 정규화된 yaw 표본들의 circular median 을 구한다. 단위벡터 평균(circular
// mean) 대신 sin/cos 각각의 median 을 따로 취한 뒤 atan2 로 합쳐, 한두 개의
// outlier (예: YOLO keypoint 가 한 frame 에서 크게 튄 경우) 에도 흔들리지
// 않는 robust 한 대표값을 얻는다. wrap-around (예: 0° vs 89.9°) 도 자연스럽게 처리.
double circular_median_yaw_normalized(const std::vector<double> & yaws)
{
  if (yaws.empty()) {return 0.0;}              // 표본 없으면 0 반환 (호출부에서 has_yaw=false 와 함께 사용)
  std::vector<double> sins, coss;
  sins.reserve(yaws.size());                   // capacity 미리 확보
  coss.reserve(yaws.size());
  for (double y : yaws) {                      // 각 yaw 를 단위원 위의 좌표 (cos y, sin y) 로 변환
    sins.push_back(std::sin(y));               // sin 성분 누적
    coss.push_back(std::cos(y));               // cos 성분 누적
  }
  double s = median_sorted(std::move(sins));   // sin 들의 median (각 축별로 outlier robust)
  double c = median_sorted(std::move(coss));   // cos 들의 median
  if (std::abs(s) < 1e-12 && std::abs(c) < 1e-12) {return 0.0;}  // 양쪽 ~0 이면 방향 미정 → 0 반환
  return std::atan2(s, c);                     // 다시 각도로 환원: atan2(median sin, median cos)
}

}  // namespace

// ============================================================
// BoxCupWorldPoseNode
// ============================================================

BoxCupWorldPoseNode::BoxCupWorldPoseNode(const rclcpp::NodeOptions & options)
: Node("box_cup_world_pose", options)
{
  declare_parameter("camera_intrinsics_path", "");
  declare_parameter("keypoints_service_name", "/perception/get_box_cup_keypoints");
  declare_parameter("world_service_name", "/perception/get_box_cup_world_poses");
  declare_parameter("target_frame", "world");
  declare_parameter("camera_frame", "default_cam");
  declare_parameter("box_top_z_m", 0.030);
  declare_parameter("box_output_z_m", 0.015);
  declare_parameter("cup_top_z_m", 0.080);
  declare_parameter("cup_output_z_m", 0.080);
  declare_parameter("world_offset_x_m", 0.000); // x 오차
  declare_parameter("world_offset_y_m", 0.015); // y 오차
  declare_parameter("min_keypoint_confidence", 0.10);
  declare_parameter("keypoints_timeout_sec", 12.0);
  declare_parameter("tf_lookup_timeout_sec", 0.5);
  declare_parameter("keypoint_order", std::vector<int64_t>{0, 1, 2, 3});
  // 키포인트 노드에 한 번 호출로 N개 sample 을 요청한다. 키포인트 노드 측에서
  // 새 image frame 신호 기반으로 N회 추론한다 (시간 sleep 없음).
  declare_parameter("num_samples", 3);
  declare_parameter("min_valid_samples", 2);
  declare_parameter("cluster_match_tolerance_m", 0.03);

  keypoints_service_name_ = get_parameter("keypoints_service_name").as_string();
  world_service_name_ = get_parameter("world_service_name").as_string();
  target_frame_ = get_parameter("target_frame").as_string();
  camera_frame_ = get_parameter("camera_frame").as_string();
  box_top_z_m_ = get_parameter("box_top_z_m").as_double();
  box_output_z_m_ = get_parameter("box_output_z_m").as_double();
  cup_top_z_m_ = get_parameter("cup_top_z_m").as_double();
  cup_output_z_m_ = get_parameter("cup_output_z_m").as_double();
  world_offset_x_m_ = get_parameter("world_offset_x_m").as_double();
  world_offset_y_m_ = get_parameter("world_offset_y_m").as_double();
  min_keypoint_confidence_ = get_parameter("min_keypoint_confidence").as_double();
  keypoints_timeout_sec_ = get_parameter("keypoints_timeout_sec").as_double();
  tf_lookup_timeout_sec_ = get_parameter("tf_lookup_timeout_sec").as_double();
  keypoint_order_ = get_parameter("keypoint_order").as_integer_array();
  num_samples_ = static_cast<int>(get_parameter("num_samples").as_int());
  min_valid_samples_ = static_cast<int>(get_parameter("min_valid_samples").as_int());
  cluster_match_tolerance_m_ = get_parameter("cluster_match_tolerance_m").as_double();
  if (num_samples_ < 1) {
    throw std::runtime_error("num_samples must be >= 1");
  }
  if (min_valid_samples_ < 1 || min_valid_samples_ > num_samples_) {
    throw std::runtime_error(
      "min_valid_samples must satisfy 1 <= min_valid_samples <= num_samples");
  }
  if (cluster_match_tolerance_m_ <= 0.0) {
    throw std::runtime_error("cluster_match_tolerance_m must be positive");
  }

  std::string intrinsics_path = get_parameter("camera_intrinsics_path").as_string();
  if (intrinsics_path.empty()) {
    throw std::runtime_error("camera_intrinsics_path parameter must not be empty");
  }
  intrinsics_ = load_intrinsics(intrinsics_path);

  // TF 변환 정보를 저장/조회하는 버퍼 생성, get_clock()은 ROS 시간 기준으로 TF를 조회하기 위해 사용
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  // tf_buffer를 구독독하는 리스너 생성
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  keypoints_client_ = create_client<omx_interfaces::srv::GetKeypointDetections>(
    keypoints_service_name_,
    rclcpp::ServicesQoS(),
    cb_group_);

  world_pose_srv_ = create_service<omx_interfaces::srv::GetBlockPoses>(
    world_service_name_,
    std::bind(
      &BoxCupWorldPoseNode::handle_get_block_poses, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(),
    cb_group_);

  RCLCPP_INFO(get_logger(),
    "box_cup_world_pose ready: %s -> %s (target_frame=%s, camera_frame=%s)",
    keypoints_service_name_.c_str(), world_service_name_.c_str(),
    target_frame_.c_str(), camera_frame_.c_str());
}

// ============================================================
// service handler
// ============================================================

void BoxCupWorldPoseNode::handle_get_block_poses(
  const std::shared_ptr<omx_interfaces::srv::GetBlockPoses::Request> request,   // 요청 메시지: color 필터 등 (srv/GetBlockPoses.srv)
  std::shared_ptr<omx_interfaces::srv::GetBlockPoses::Response> response)       // 응답 메시지: BlockPose[] 를 채워서 돌려줌
{
  // cluster: 동일 객체(같은 color + 인접 XY)에 대해 모인 sample 집합.
  // 첫 sample 의 (x, y) 를 reference 로 잡고, 이후 sample 은 같은 color +
  // tolerance 내 cluster 에 join. 매칭 실패 sample 은 새 cluster 를 만든다.
  struct Cluster
  {
    int class_id;                              // 객체 종류 (KeypointDetection::CLASS_BOX/CLASS_CUP)
    std::string color;                         // 박스 색("red"/"blue"/...) 또는 컵("cup")
    std::vector<double> xs;                    // N개 sample 의 world x 누적 (median 입력)
    std::vector<double> ys;                    // N개 sample 의 world y 누적
    std::vector<double> yaws_norm;             // 박스만 채움. [0, π/2) 로 정규화된 yaw 누적
    std::vector<float> det_confs;              // sample 별 YOLO detection confidence
    std::vector<float> yaw_confs;              // sample 별 keypoint(=yaw) confidence (컵은 0)
    bool has_yaw;                              // 박스=true, 컵=false (원기둥은 yaw 의미 없음)
    double ref_x;                              // cluster 기준점 x (첫 sample 값, 거리비교 anchor)
    double ref_y;                              // cluster 기준점 y
    builtin_interfaces::msg::Time latest_stamp;// 이 cluster 에 마지막으로 추가된 sample 의 시각
  };

  std::vector<Cluster> clusters;               // 이번 호출 동안 동적으로 자라는 cluster 모음
  builtin_interfaces::msg::Time latest_stamp;  // 응답 BlockPose.header.stamp 후보 (가장 최근 TF 시각)
  bool any_stamp_set = false;                  // latest_stamp 가 한 번이라도 채워졌는지 flag
  int successful_samples = 0;                  // keypoint+TF 모두 성공한 sample 수 (0 이면 빈 응답)

  // 키포인트 노드에 num_samples_ 개 sample 을 한 번에 요청. 노드가 새 image
  // frame 도착을 신호로 N회 추론하므로 client 는 시간 sleep 없이 결과만 받는다.
  // min_valid_samples_ 도 전달해 서버 측 부분 성공 임계값과 일치시킨다.
  auto kp_response = call_keypoint_service(
    num_samples_, min_valid_samples_, /*publish_debug=*/false);
  if (!kp_response.has_value()) {                                               // timeout / 서비스 unavailable
    RCLCPP_WARN(get_logger(), "GetKeypointDetections failed");
    return;                                                                     // 빈 응답
  }
  const auto & kp_resp = kp_response.value();                                   // 응답 본체 (header, success, message, samples[])
  if (!kp_resp.success) {                                                       // 노드가 응답은 했지만 내부적으로 실패
    RCLCPP_WARN(get_logger(),
      "keypoint node returned success=false: %s", kp_resp.message.c_str());
    return;
  }

  for (size_t sample_idx = 0; sample_idx < kp_resp.samples.size(); ++sample_idx) {  // sample 별(서로 다른 frame) 순회
    const auto & sample = kp_resp.samples[sample_idx];                          // KeypointSample{stamp, detections[]}

    auto tf_opt = lookup_transform(sample.stamp);                               // 이 sample frame 시각과 동기화된 world←camera TF 조회
    if (!tf_opt.has_value()) {                                                  // TF lookup 실패 (시각 안 맞거나 frame 없음)
      RCLCPP_WARN(get_logger(), "sample %zu: TF lookup failed", sample_idx);    // sample 폐기
      continue;
    }
    const auto & tf_stamped = tf_opt.value();                                   // TransformStamped: header + transform.{translation,rotation}
    latest_stamp = tf_stamped.header.stamp;                                     // 최근 TF 시각으로 갱신 (응답 stamp 후보)
    any_stamp_set = true;                                                       // 최소 1회 성공 표시
    ++successful_samples;                                                       // keypoint+TF 동시 성공 카운트

    for (const auto & det : sample.detections) {                                // 이 sample 의 모든 detection 순회 (다중 객체)
      auto sample_opt = process_detection(det, tf_stamped);                     // ray-plane intersection 으로 (x,y,[yaw]) 산출
      if (!sample_opt.has_value()) {continue;}                                  // conf 미달 / 광선 수평 / 카메라 뒤 등 → drop
      const auto & s = sample_opt.value();                                      // 유효 DetectionSample (color,x,y,yaw_world,has_yaw,confs)

      // 같은 color + XY 거리 < tolerance 인 가장 가까운 cluster 에 join.
      int best_idx = -1;                                                        // 매칭된 cluster 인덱스 (없으면 -1)
      double best_d2 = std::numeric_limits<double>::infinity();                 // 현재까지 최소 거리² (작을수록 가까움)
      const double tol2 =
        cluster_match_tolerance_m_ * cluster_match_tolerance_m_;                // sqrt 회피용으로 거리²끼리 비교
      for (size_t ci = 0; ci < clusters.size(); ++ci) {                         // 기존 cluster 전부 훑기
        if (clusters[ci].color != s.color) {continue;}                          // 색이 다르면 동일 객체 아님
        double dx = clusters[ci].ref_x - s.x;                                   // cluster 기준점과의 x 차이
        double dy = clusters[ci].ref_y - s.y;                                   // y 차이
        double d2 = dx * dx + dy * dy;                                          // 유클리드 거리² (제곱근 생략)
        if (d2 < tol2 && d2 < best_d2) {                                        // tolerance 안 + 지금까지 최단이면 후보 갱신
          best_d2 = d2;                                                         // 최단 거리² 기록
          best_idx = static_cast<int>(ci);                                      // best cluster 인덱스 기록
        }
      }

      if (best_idx < 0) {                                                       // 매칭 실패 → 새 cluster 생성
        Cluster c;
        c.class_id = s.class_id;                                                // 종류 기록
        c.color = s.color;                                                      // 색 기록 (이후 매칭 조건)
        c.has_yaw = s.has_yaw;                                                  // yaw 보유 여부 (BOX=true)
        c.ref_x = s.x;                                                          // 첫 sample 의 (x,y) 가 cluster reference 가 됨
        c.ref_y = s.y;
        c.latest_stamp = tf_stamped.header.stamp;                               // 이 sample 의 시각 기록
        c.xs.push_back(s.x);                                                    // 첫 sample 의 x 누적
        c.ys.push_back(s.y);                                                    // y 누적
        c.det_confs.push_back(s.detection_confidence);                          // detection conf 누적
        c.yaw_confs.push_back(s.yaw_confidence);                                // yaw conf 누적 (컵은 0)
        if (s.has_yaw) {
          c.yaws_norm.push_back(wrap_yaw_zero_pi_over_2(s.yaw_world));          // 박스만 yaw 누적, [0,π/2) 로 정규화 후
        }
        clusters.push_back(std::move(c));                                       // cluster 모음에 추가 (move 로 복사 회피)
      } else {                                                                  // 기존 cluster 에 합치는 경로
        auto & c = clusters[best_idx];                                          // best cluster 참조 (수정 위해 reference)
        c.xs.push_back(s.x);                                                    // 같은 객체의 추가 관측 누적
        c.ys.push_back(s.y);
        c.det_confs.push_back(s.detection_confidence);
        c.yaw_confs.push_back(s.yaw_confidence);
        if (s.has_yaw) {
          c.yaws_norm.push_back(wrap_yaw_zero_pi_over_2(s.yaw_world));          // 박스 yaw 추가 누적
        }
        c.latest_stamp = tf_stamped.header.stamp;                               // 가장 최근 관측 시각으로 갱신
      }
    }
  }

  if (successful_samples == 0) {                                                // 모든 sample 실패 → 빈 응답으로 조기 종료
    RCLCPP_WARN(get_logger(),
      "all %zu sample(s) failed; returning empty blocks", kp_resp.samples.size());
    return;                                                                     // response->blocks 비어있는 상태로 리턴
  }

  // confidence 내림차순으로 정렬해 응답 (기존 호출부가 confidence 우선 가정).
  std::vector<std::pair<size_t, float>> ranking;                                // (cluster 인덱스, conf_median) 페어 모음
  for (size_t ci = 0; ci < clusters.size(); ++ci) {                             // 모든 cluster 검토
    if (static_cast<int>(clusters[ci].xs.size()) < min_valid_samples_) {        // sample 수 부족 (예: 1/3) → drop
      RCLCPP_DEBUG(get_logger(),
        "cluster color=%s dropped: %zu < min_valid_samples=%d",
        clusters[ci].color.c_str(), clusters[ci].xs.size(), min_valid_samples_);
      continue;
    }
    float conf_median = static_cast<float>(median_sorted(                       // detection conf 의 median 계산
      std::vector<double>(clusters[ci].det_confs.begin(),                       // float→double 변환 위한 임시 vector
                          clusters[ci].det_confs.end())));
    ranking.emplace_back(ci, conf_median);                                      // 응답 정렬용 페어 추가
  }
  std::sort(ranking.begin(), ranking.end(),
    [](const auto & a, const auto & b) {return a.second > b.second;});          // conf 내림차순 정렬

  for (const auto & [ci, conf_median] : ranking) {                              // 신뢰도 높은 순으로 응답 생성
    const auto & c = clusters[ci];                                              // 대상 cluster 참조

    if (!request->color.empty() && c.color != request->color) {                 // 요청에 color 필터가 있고 불일치면
      continue;                                                                 // 이 cluster 는 응답에서 제외
    }

    omx_interfaces::msg::BlockPose block;                                       // 응답 한 항목 생성
    block.header.stamp = any_stamp_set ? latest_stamp : c.latest_stamp;         // 최신 TF 시각 우선, 없으면 cluster 시각
    block.header.frame_id = target_frame_;                                      // 좌표계 = "world" (YAML target_frame)
    block.pose.header = block.header;                                           // 내부 PoseStamped 헤더도 동일하게

    double x_med = median_sorted(c.xs);                                         // x 의 robust 대표값
    double y_med = median_sorted(c.ys);                                         // y 의 robust 대표값
    block.pose.pose.position.x = x_med;                                         // 최종 world x
    block.pose.pose.position.y = y_med;                                         // 최종 world y
    block.pose.pose.position.z = (c.class_id ==                                 // class_id 에 따라 출력 z 선택
      omx_interfaces::msg::KeypointDetection::CLASS_BOX)
      ? box_output_z_m_ : cup_output_z_m_;                                      // 박스: 중심 z, 컵: 입구 z

    if (c.has_yaw && !c.yaws_norm.empty()) {                                    // 박스 + yaw 표본 있음
      double yaw = circular_median_yaw_normalized(c.yaws_norm);                 // sin/cos 의 median → atan2 (원형 robust median)
      // 수치 오차로 (-pi/4, pi/4] 밖으로 밀릴 수 있으니 한 번 더 wrap.
      yaw = wrap_yaw_zero_pi_over_2(yaw);                                       // 90° 대칭 구간으로 재정규화
      block.pose.pose.orientation = yaw_to_quat(yaw);                           // z축 회전만 quaternion 으로 변환
      block.yaw_confidence = static_cast<float>(median_sorted(                  // yaw 신뢰도도 median 사용
        std::vector<double>(c.yaw_confs.begin(), c.yaw_confs.end())));
      block.color = c.color.empty() ? "unknown" : c.color;                      // 색 채움 (빈 문자열 방어)
    } else {                                                                    // 컵 또는 박스인데 yaw 표본 없음
      block.pose.pose.orientation = identity_quat();                            // (0,0,0,1) — yaw 없음
      block.yaw_confidence = 0.0f;                                              // yaw 신뢰도 없음
      block.color = c.color.empty() ? "unknown" : c.color;                      // 색 채움 (컵이면 "cup")
    }
    block.confidence = conf_median;                                             // detection conf 의 median 을 종합 신뢰도로
    block.grasp_pose = block.pose;                                              // 별도 grasp 보정 없이 pose 와 동일 (필요시 추후 분리)

    response->blocks.push_back(block);                                          // 최종 응답에 추가
  }

  RCLCPP_DEBUG(get_logger(),
    "fused %d/%zu samples into %zu cluster(s), returned %zu block(s)",         // 진단 로그: 성공률, cluster 수, 응답 항목 수
    successful_samples, kp_resp.samples.size(), clusters.size(), response->blocks.size());
}

// ============================================================
// keypoint service call (event-based, no sleep)
// ============================================================

std::optional<omx_interfaces::srv::GetKeypointDetections::Response>
BoxCupWorldPoseNode::call_keypoint_service(
  int num_samples, int min_valid_samples, bool publish_debug)
{
  // timeout 의 의미: "응답 못 받음 → 빈 결과로 client 에게 알림" 이지 fail 이 아님.
  // pick_place 측 service_call_timeout_sec(15.0) 보다 짧게 두어 client 가 먼저
  // 끊기지 않도록 한다.
  if (!keypoints_client_->wait_for_service(0s)) {
    RCLCPP_WARN(get_logger(),
      "keypoint service '%s' not available", keypoints_service_name_.c_str());
    return std::nullopt;
  }

  auto req = std::make_shared<omx_interfaces::srv::GetKeypointDetections::Request>();
  req->num_samples = num_samples;
  req->min_valid_samples = min_valid_samples;
  req->publish_debug = publish_debug;

  auto future = keypoints_client_->async_send_request(req);
  auto timeout = std::chrono::duration<double>(keypoints_timeout_sec_);
  auto status = future.wait_for(timeout);
  if (status != std::future_status::ready) {
    RCLCPP_WARN(get_logger(),
      "GetKeypointDetections response timeout after %.1f s, "
      "returning empty samples to client", keypoints_timeout_sec_);
    return std::nullopt;
  }
  return *future.get();
}

// ============================================================
// TF lookup
// ============================================================

std::optional<geometry_msgs::msg::TransformStamped>
BoxCupWorldPoseNode::lookup_transform(const builtin_interfaces::msg::Time & stamp)
{
  rclcpp::Time lookup_time(stamp.sec, stamp.nanosec, get_clock()->get_clock_type());
  if (stamp.sec == 0 && stamp.nanosec == 0) {
    lookup_time = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  try {
    auto timeout = tf2::durationFromSec(tf_lookup_timeout_sec_);
    auto tf = tf_buffer_->lookupTransform(
      target_frame_, camera_frame_, lookup_time, timeout);
    return tf;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "TF lookup %s->%s failed: %s",
      camera_frame_.c_str(), target_frame_.c_str(), ex.what());
    return std::nullopt;
  }
}

// ============================================================
// per-detection ray-plane intersection + world pose
// ============================================================

std::optional<BoxCupWorldPoseNode::DetectionSample>
BoxCupWorldPoseNode::process_detection(
  const omx_interfaces::msg::KeypointDetection & det,
  const geometry_msgs::msg::TransformStamped & tf_stamped)
{
  // check keypoint confidence
  // keypoints flat array: [k0_x, k0_y, k0_conf, k1_x, k1_y, k1_conf, ...]
  if (det.keypoints.size() < 12) { // 왜 12로 설정되어있는거지?
    RCLCPP_WARN(get_logger(),
      "detection has fewer than 12 keypoint values (class=%d), skipping", det.class_id);
    return std::nullopt;
  }

  double min_kp_conf = 1.0;
  std::vector<cv::Point2f> image_points;
  for (size_t ki = 0; ki < 4; ++ki) {
    size_t base = keypoint_order_[ki] * 3;
    float kx = det.keypoints[base];
    float ky = det.keypoints[base + 1];
    float kc = det.keypoints[base + 2];
    min_kp_conf = std::min(min_kp_conf, static_cast<double>(kc));
    image_points.emplace_back(kx, ky);
  }

  if (min_kp_conf < min_keypoint_confidence_) {
    RCLCPP_DEBUG(get_logger(),
      "detection (class=%d) min_kp_conf=%.3f < %.3f, skipping",
      det.class_id, min_kp_conf, min_keypoint_confidence_);
    return std::nullopt;
  }

  // class -> plane z prior (object 윗면이 world z = plane_z 에 놓여 있다고 가정).
  bool is_box = (det.class_id == omx_interfaces::msg::KeypointDetection::CLASS_BOX);
  bool is_cup = (det.class_id == omx_interfaces::msg::KeypointDetection::CLASS_CUP);
  double plane_z;
  if (is_box) {
    plane_z = box_top_z_m_;
  } else if (is_cup) {
    plane_z = cup_top_z_m_;
  } else {
    RCLCPP_WARN(get_logger(), "unsupported class_id=%d, skipping", det.class_id);
    return std::nullopt;
  }

  // TF: world <- camera.
  const auto & t = tf_stamped.transform.translation;
  const auto & q = tf_stamped.transform.rotation;
  cv::Matx33d R_world_cam = quat_to_rot(q.x, q.y, q.z, q.w);
  cv::Vec3d t_world_cam(t.x, t.y, t.z);

  // pixel keypoints -> world (x,y,plane_z).
  auto world_pts_opt = project_keypoints_to_plane(
    image_points, intrinsics_.K, intrinsics_.D,
    R_world_cam, t_world_cam, plane_z);
  if (!world_pts_opt.has_value()) {
    RCLCPP_WARN(get_logger(),
      "ray-plane intersection failed for class=%d (z=%.3f), skipping",
      det.class_id, plane_z);
    return std::nullopt;
  }
  const auto & world_pts = world_pts_opt.value();

  // 4 점 평균이 객체 평면-XY 중심. 객체 모델 크기에 비의존.
  auto c = centroid_xy(world_pts);

  DetectionSample sample;
  sample.class_id = det.class_id;
  // 측정된 systematic bias 보정. scan pose 가 고정인 경우에 유효하며, pose 가
  // 바뀌면 재교정 필요. 근본 해결은 URDF camera_mount 의 xyz/rpy 또는 hand-eye
  // calibration.
  sample.x = c.x + world_offset_x_m_;
  sample.y = c.y + world_offset_y_m_;
  sample.detection_confidence = det.detection_confidence;

  if (is_box) {
    sample.has_yaw = true;
    sample.yaw_world = box_yaw_from_corners(world_pts);
    sample.yaw_confidence = static_cast<float>(min_kp_conf);
    sample.color = det.color.empty() ? "unknown" : det.color;
  } else {
    sample.has_yaw = false;
    sample.yaw_world = 0.0;
    sample.yaw_confidence = 0.0f;
    sample.color = "cup";
  }

  return sample;
}

// ============================================================
// camera intrinsics loader
// ============================================================

CameraIntrinsics BoxCupWorldPoseNode::load_intrinsics(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & ex) {
    throw std::runtime_error("failed to load camera_intrinsics yaml: " + std::string(ex.what()));
  }

  auto cam_matrix = root["camera_matrix"];
  if (!cam_matrix || !cam_matrix["data"]) {
    throw std::runtime_error("camera_intrinsics yaml missing camera_matrix.data");
  }
  auto dist_coeff = root["distortion_coefficients"];
  if (!dist_coeff || !dist_coeff["data"]) {
    throw std::runtime_error("camera_intrinsics yaml missing distortion_coefficients.data");
  }

  auto K_data = cam_matrix["data"].as<std::vector<double>>();
  auto D_data = dist_coeff["data"].as<std::vector<double>>();

  if (K_data.size() != 9) {
    throw std::runtime_error("camera_matrix.data must have 9 elements");
  }

  CameraIntrinsics ci;
  ci.K = cv::Mat(3, 3, CV_64F, K_data.data()).clone();
  ci.D = cv::Mat(1, static_cast<int>(D_data.size()), CV_64F, D_data.data()).clone();

  RCLCPP_INFO(rclcpp::get_logger("box_cup_world_pose"),
    "loaded camera intrinsics from %s (K[0,0]=%.4f)", yaml_path.c_str(), K_data[0]);

  return ci;
}

}  // namespace omx_perception_world_pose

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::ExecutorOptions opts;
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(opts, 2);

  auto node = std::make_shared<omx_perception_world_pose::BoxCupWorldPoseNode>();
  executor->add_node(node);

  try {
    executor->spin();
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(node->get_logger(), "uncaught exception: %s", ex.what());
  }

  rclcpp::shutdown();
  return 0;
}
