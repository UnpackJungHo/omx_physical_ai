#include "omx_perception_world_pose/box_cup_world_pose_node.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
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

// 픽셀 keypoint -> normalized camera coord -> world frame ray -> 수평 plane(z=plane_z)
// 와의 교점. PnP 와 달리 객체 모델 크기(cube_size, cup_radius)에 비선형 의존하지
// 않는다. 작은 객체에서 PnP depth ambiguity 가 (x,y) 로 전이되는 문제도 사라진다.
// 대신 plane_z 와 TF (world<-camera) 정확도가 새 병목.
//
// 실패 케이스:
//   - ray 가 거의 수평(d_world.z ~ 0): plane 과 교점이 무한대.
//   - lambda <= 0: 교점이 카메라 뒤 (객체가 plane 아래/카메라 뒤에 있는 비물리적
//     상황).
// 둘 다 nullopt 반환.
std::optional<std::vector<cv::Point3d>> project_keypoints_to_plane(
  const std::vector<cv::Point2f> & image_points,
  const cv::Mat & K, const cv::Mat & D,
  const cv::Matx33d & R_world_cam, const cv::Vec3d & t_world_cam,
  double plane_z)
{
  if (image_points.empty()) {return std::nullopt;}

  std::vector<cv::Point2f> undistorted;
  cv::undistortPoints(image_points, undistorted, K, D);  // P = I -> normalized

  std::vector<cv::Point3d> world_points;
  world_points.reserve(image_points.size());

  constexpr double kEpsDz = 1e-6;

  for (const auto & p : undistorted) {
    cv::Vec3d d_cam(p.x, p.y, 1.0);
    cv::Vec3d d_world = R_world_cam * d_cam;

    if (std::abs(d_world[2]) < kEpsDz) {return std::nullopt;}
    double lambda = (plane_z - t_world_cam[2]) / d_world[2];
    if (lambda <= 0.0) {return std::nullopt;}

    cv::Vec3d hit = t_world_cam + lambda * d_world;
    world_points.emplace_back(hit[0], hit[1], hit[2]);
  }
  return world_points;
}

cv::Point2d centroid_xy(const std::vector<cv::Point3d> & pts)
{
  double sx = 0.0, sy = 0.0;
  for (const auto & p : pts) {sx += p.x; sy += p.y;}
  const double n = static_cast<double>(pts.size());
  return {sx / n, sy / n};
}

// ---------- box yaw from 4 planar corners ----------

// 박스 윗면 4 모서리가 object frame 에서 (TL, TR, BR, BL) 순서일 때
// 이웃 모서리 edge p[i] -> p[(i+1)%4] 의 object-frame 방향은:
//   i=0: +x, i=1: +y, i=2: -x, i=3: -y
// world-frame edge angle 에서 object-frame edge angle 을 빼면 yaw 가 되고,
// 4 개 edge 의 circular mean (sin/cos 평균 후 atan2) 으로 노이즈를 평탄화한다.
double box_yaw_from_corners(const std::vector<cv::Point3d> & p)
{
  if (p.size() < 4) {return 0.0;}
  static constexpr double kObjEdgeAngles[4] = {
    0.0, M_PI / 2.0, M_PI, -M_PI / 2.0,
  };
  double s_sum = 0.0, c_sum = 0.0;
  for (int i = 0; i < 4; ++i) {
    double dx = p[(i + 1) % 4].x - p[i].x;
    double dy = p[(i + 1) % 4].y - p[i].y;
    double yaw = std::atan2(dy, dx) - kObjEdgeAngles[i];
    s_sum += std::sin(yaw);
    c_sum += std::cos(yaw);
  }
  return std::atan2(s_sum, c_sum);
}

// ---------- yaw normalization & circular median ----------

// 정육면체(상단 정사각형) 박스는 yaw 90도 회전 대칭이라 어떤 표현 구간을 써도
// 동일 자세를 가리킨다. [0, pi/2) 로 정규화하면 yaw=0 이 "박스 윗면 아래
// 모서리가 world x 축과 평행" 인 자세에 대응해 사람이 읽기 쉽다. joint5 정렬은
// skill_executor 가 (box_yaw - gripper_yaw) delta 에 다시 [-45, 45] wrap 을
// 적용하므로 본 구간 변경에 영향받지 않는다.
double wrap_yaw_zero_pi_over_2(double yaw)
{
  constexpr double kPeriod = M_PI / 2.0;     // 90 deg
  double y = std::fmod(yaw, kPeriod);
  if (y < 0.0) {y += kPeriod;}
  return y;
}

double median_sorted(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const size_t n = values.size();
  if (n == 0) {return 0.0;}
  if (n % 2 == 1) {return values[n / 2];}
  return 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

// 정규화된 yaw 들의 circular median: 단위벡터 평균이 아닌 sin/cos 각각의
// median 으로부터 atan2 를 취해 outlier 에도 robust 한 대표값을 얻는다.
double circular_median_yaw_normalized(const std::vector<double> & yaws)
{
  if (yaws.empty()) {return 0.0;}
  std::vector<double> sins, coss;
  sins.reserve(yaws.size());
  coss.reserve(yaws.size());
  for (double y : yaws) {
    sins.push_back(std::sin(y));
    coss.push_back(std::cos(y));
  }
  double s = median_sorted(std::move(sins));
  double c = median_sorted(std::move(coss));
  if (std::abs(s) < 1e-12 && std::abs(c) < 1e-12) {return 0.0;}
  return std::atan2(s, c);
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
  declare_parameter("world_offset_x_m", 0.000);
  declare_parameter("world_offset_y_m", 0.015);
  declare_parameter("min_keypoint_confidence", 0.10);
  declare_parameter("keypoints_timeout_sec", 2.0);
  declare_parameter("tf_lookup_timeout_sec", 0.5);
  declare_parameter("keypoint_order", std::vector<int64_t>{0, 1, 2, 3});
  declare_parameter("num_samples", 3);
  declare_parameter("sample_interval_sec", 0.05);
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
  sample_interval_sec_ = get_parameter("sample_interval_sec").as_double();
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
  if (sample_interval_sec_ < 0.0) {
    throw std::runtime_error("sample_interval_sec must be >= 0");
  }

  std::string intrinsics_path = get_parameter("camera_intrinsics_path").as_string();
  if (intrinsics_path.empty()) {
    throw std::runtime_error("camera_intrinsics_path parameter must not be empty");
  }
  intrinsics_ = load_intrinsics(intrinsics_path);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
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
  const std::shared_ptr<omx_interfaces::srv::GetBlockPoses::Request> request,
  std::shared_ptr<omx_interfaces::srv::GetBlockPoses::Response> response)
{
  // cluster: 동일 객체(같은 color + 인접 XY)에 대해 모인 sample 집합.
  // 첫 sample 의 (x, y) 를 reference 로 잡고, 이후 sample 은 같은 color +
  // tolerance 내 cluster 에 join. 매칭 실패 sample 은 새 cluster 를 만든다.
  struct Cluster
  {
    int class_id;
    std::string color;
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> yaws_norm;     // 박스만 채움 (-pi/4, pi/4]
    std::vector<float> det_confs;
    std::vector<float> yaw_confs;
    bool has_yaw;                      // 박스=true, 컵=false
    double ref_x;
    double ref_y;
    builtin_interfaces::msg::Time latest_stamp;
  };

  std::vector<Cluster> clusters;
  builtin_interfaces::msg::Time latest_stamp;
  bool any_stamp_set = false;
  int successful_samples = 0;

  for (int sample_idx = 0; sample_idx < num_samples_; ++sample_idx) {
    if (sample_idx > 0 && sample_interval_sec_ > 0.0) {
      std::this_thread::sleep_for(
        std::chrono::duration<double>(sample_interval_sec_));
    }

    auto kp_response = call_keypoint_service(/*publish_debug=*/false);
    if (!kp_response.has_value()) {
      RCLCPP_WARN(get_logger(),
        "sample %d: GetKeypointDetections failed", sample_idx);
      continue;
    }
    const auto & kp_resp = kp_response.value();
    if (!kp_resp.success) {
      RCLCPP_WARN(get_logger(),
        "sample %d: keypoint node returned success=false: %s",
        sample_idx, kp_resp.message.c_str());
      continue;
    }

    auto tf_opt = lookup_transform(kp_resp.header.stamp);
    if (!tf_opt.has_value()) {
      RCLCPP_WARN(get_logger(), "sample %d: TF lookup failed", sample_idx);
      continue;
    }
    const auto & tf_stamped = tf_opt.value();
    latest_stamp = tf_stamped.header.stamp;
    any_stamp_set = true;
    ++successful_samples;

    for (const auto & det : kp_resp.detections) {
      auto sample_opt = process_detection(det, tf_stamped);
      if (!sample_opt.has_value()) {continue;}
      const auto & s = sample_opt.value();

      // 같은 color + XY 거리 < tolerance 인 가장 가까운 cluster 에 join.
      int best_idx = -1;
      double best_d2 = std::numeric_limits<double>::infinity();
      const double tol2 =
        cluster_match_tolerance_m_ * cluster_match_tolerance_m_;
      for (size_t ci = 0; ci < clusters.size(); ++ci) {
        if (clusters[ci].color != s.color) {continue;}
        double dx = clusters[ci].ref_x - s.x;
        double dy = clusters[ci].ref_y - s.y;
        double d2 = dx * dx + dy * dy;
        if (d2 < tol2 && d2 < best_d2) {
          best_d2 = d2;
          best_idx = static_cast<int>(ci);
        }
      }

      if (best_idx < 0) {
        Cluster c;
        c.class_id = s.class_id;
        c.color = s.color;
        c.has_yaw = s.has_yaw;
        c.ref_x = s.x;
        c.ref_y = s.y;
        c.latest_stamp = tf_stamped.header.stamp;
        c.xs.push_back(s.x);
        c.ys.push_back(s.y);
        c.det_confs.push_back(s.detection_confidence);
        c.yaw_confs.push_back(s.yaw_confidence);
        if (s.has_yaw) {
          c.yaws_norm.push_back(wrap_yaw_zero_pi_over_2(s.yaw_world));
        }
        clusters.push_back(std::move(c));
      } else {
        auto & c = clusters[best_idx];
        c.xs.push_back(s.x);
        c.ys.push_back(s.y);
        c.det_confs.push_back(s.detection_confidence);
        c.yaw_confs.push_back(s.yaw_confidence);
        if (s.has_yaw) {
          c.yaws_norm.push_back(wrap_yaw_zero_pi_over_2(s.yaw_world));
        }
        c.latest_stamp = tf_stamped.header.stamp;
      }
    }
  }

  if (successful_samples == 0) {
    RCLCPP_WARN(get_logger(),
      "all %d sample(s) failed; returning empty blocks", num_samples_);
    return;
  }

  // confidence 내림차순으로 정렬해 응답 (기존 호출부가 confidence 우선 가정).
  std::vector<std::pair<size_t, float>> ranking;
  for (size_t ci = 0; ci < clusters.size(); ++ci) {
    if (static_cast<int>(clusters[ci].xs.size()) < min_valid_samples_) {
      RCLCPP_DEBUG(get_logger(),
        "cluster color=%s dropped: %zu < min_valid_samples=%d",
        clusters[ci].color.c_str(), clusters[ci].xs.size(), min_valid_samples_);
      continue;
    }
    float conf_median = static_cast<float>(median_sorted(
      std::vector<double>(clusters[ci].det_confs.begin(),
                          clusters[ci].det_confs.end())));
    ranking.emplace_back(ci, conf_median);
  }
  std::sort(ranking.begin(), ranking.end(),
    [](const auto & a, const auto & b) {return a.second > b.second;});

  for (const auto & [ci, conf_median] : ranking) {
    const auto & c = clusters[ci];

    if (!request->color.empty() && c.color != request->color) {
      continue;
    }

    omx_interfaces::msg::BlockPose block;
    block.header.stamp = any_stamp_set ? latest_stamp : c.latest_stamp;
    block.header.frame_id = target_frame_;
    block.pose.header = block.header;

    double x_med = median_sorted(c.xs);
    double y_med = median_sorted(c.ys);
    block.pose.pose.position.x = x_med;
    block.pose.pose.position.y = y_med;
    block.pose.pose.position.z = (c.class_id ==
      omx_interfaces::msg::KeypointDetection::CLASS_BOX)
      ? box_output_z_m_ : cup_output_z_m_;

    if (c.has_yaw && !c.yaws_norm.empty()) {
      double yaw = circular_median_yaw_normalized(c.yaws_norm);
      // 수치 오차로 (-pi/4, pi/4] 밖으로 밀릴 수 있으니 한 번 더 wrap.
      yaw = wrap_yaw_zero_pi_over_2(yaw);
      block.pose.pose.orientation = yaw_to_quat(yaw);
      block.yaw_confidence = static_cast<float>(median_sorted(
        std::vector<double>(c.yaw_confs.begin(), c.yaw_confs.end())));
      block.color = c.color.empty() ? "unknown" : c.color;
    } else {
      block.pose.pose.orientation = identity_quat();
      block.yaw_confidence = 0.0f;
      block.color = c.color.empty() ? "unknown" : c.color;
    }
    block.confidence = conf_median;
    block.grasp_pose = block.pose;

    response->blocks.push_back(block);
  }

  RCLCPP_DEBUG(get_logger(),
    "fused %d/%d samples into %zu cluster(s), returned %zu block(s)",
    successful_samples, num_samples_, clusters.size(), response->blocks.size());
}

// ============================================================
// keypoint service call (event-based, no sleep)
// ============================================================

std::optional<omx_interfaces::srv::GetKeypointDetections::Response>
BoxCupWorldPoseNode::call_keypoint_service(bool publish_debug)
{
  if (!keypoints_client_->wait_for_service(0s)) {
    RCLCPP_WARN(get_logger(),
      "keypoint service '%s' not available", keypoints_service_name_.c_str());
    return std::nullopt;
  }

  auto req = std::make_shared<omx_interfaces::srv::GetKeypointDetections::Request>();
  req->publish_debug = publish_debug;

  auto future = keypoints_client_->async_send_request(req);

  auto timeout = std::chrono::duration<double>(keypoints_timeout_sec_);
  auto status = future.wait_for(timeout);
  if (status != std::future_status::ready) {
    RCLCPP_WARN(get_logger(),
      "GetKeypointDetections timed out after %.1f s", keypoints_timeout_sec_);
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
// per-detection solvePnP + world pose
// ============================================================

std::optional<BoxCupWorldPoseNode::DetectionSample>
BoxCupWorldPoseNode::process_detection(
  const omx_interfaces::msg::KeypointDetection & det,
  const geometry_msgs::msg::TransformStamped & tf_stamped)
{
  // check keypoint confidence
  // keypoints flat array: [k0_x, k0_y, k0_conf, k1_x, k1_y, k1_conf, ...]
  if (det.keypoints.size() < 12) {
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
