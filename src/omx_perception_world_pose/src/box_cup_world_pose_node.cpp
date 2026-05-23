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

// ---------- object model points ----------

std::vector<cv::Point3f> box_object_points(double cube_size_m)
{
  float half = static_cast<float>(cube_size_m / 2.0);
  return {
    {-half, -half, 0.0f},   // TL
    { half, -half, 0.0f},   // TR
    { half,  half, 0.0f},   // BR
    {-half,  half, 0.0f},   // BL
  };
}

std::vector<cv::Point3f> cup_object_points(double cup_radius_m)
{
  float r = static_cast<float>(cup_radius_m);
  return {
    {-r, 0.0f, 0.0f},
    { 0.0f, -r, 0.0f},
    { r, 0.0f, 0.0f},
    { 0.0f,  r, 0.0f},
  };
}

// ---------- box yaw from rvec + R_world_cam ----------

double box_yaw_world(const cv::Vec3d & rvec, const cv::Matx33d & R_world_cam)
{
  cv::Matx33d R_cam_obj;
  cv::Rodrigues(rvec, R_cam_obj);

  cv::Matx33d R_world_obj = R_world_cam * R_cam_obj;

  // x-axis of object in world frame
  cv::Vec3d x_world(R_world_obj(0, 0), R_world_obj(1, 0), R_world_obj(2, 0));
  return std::atan2(x_world[1], x_world[0]);
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
  declare_parameter("cube_size_m", 0.030);
  declare_parameter("box_output_z_m", 0.015);
  declare_parameter("cup_radius_m", 0.070);
  declare_parameter("cup_output_z_m", 0.080);
  declare_parameter("min_keypoint_confidence", 0.10);
  declare_parameter("keypoints_timeout_sec", 2.0);
  declare_parameter("tf_lookup_timeout_sec", 0.5);
  declare_parameter("keypoint_order", std::vector<int64_t>{0, 1, 2, 3});

  keypoints_service_name_ = get_parameter("keypoints_service_name").as_string();
  world_service_name_ = get_parameter("world_service_name").as_string();
  target_frame_ = get_parameter("target_frame").as_string();
  camera_frame_ = get_parameter("camera_frame").as_string();
  cube_size_m_ = get_parameter("cube_size_m").as_double();
  box_output_z_m_ = get_parameter("box_output_z_m").as_double();
  cup_radius_m_ = get_parameter("cup_radius_m").as_double();
  cup_output_z_m_ = get_parameter("cup_output_z_m").as_double();
  min_keypoint_confidence_ = get_parameter("min_keypoint_confidence").as_double();
  keypoints_timeout_sec_ = get_parameter("keypoints_timeout_sec").as_double();
  tf_lookup_timeout_sec_ = get_parameter("tf_lookup_timeout_sec").as_double();
  keypoint_order_ = get_parameter("keypoint_order").as_integer_array();

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
  auto kp_response = call_keypoint_service(/*publish_debug=*/false);
  if (!kp_response.has_value()) {
    RCLCPP_WARN(get_logger(), "GetKeypointDetections call failed; returning empty blocks");
    return;
  }

  const auto & kp_resp = kp_response.value();
  if (!kp_resp.success) {
    RCLCPP_WARN(get_logger(),
      "keypoint node returned success=false: %s", kp_resp.message.c_str());
    return;
  }

  auto tf_opt = lookup_transform(kp_resp.header.stamp);
  if (!tf_opt.has_value()) {
    RCLCPP_WARN(get_logger(), "TF lookup failed; returning empty blocks");
    return;
  }
  const auto & tf_stamped = tf_opt.value();

  for (const auto & det : kp_resp.detections) {
    auto block_opt = process_detection(det, tf_stamped);
    if (!block_opt.has_value()) {
      continue;
    }
    const auto & block = block_opt.value();

    // color filter
    if (!request->color.empty() && block.color != request->color) {
      continue;
    }
    response->blocks.push_back(block);
  }
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

std::optional<omx_interfaces::msg::BlockPose>
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

  // build object model
  std::vector<cv::Point3f> object_points;
  bool is_box = (det.class_id == omx_interfaces::msg::KeypointDetection::CLASS_BOX);
  bool is_cup = (det.class_id == omx_interfaces::msg::KeypointDetection::CLASS_CUP);

  if (is_box) {
    object_points = box_object_points(cube_size_m_);
  } else if (is_cup) {
    object_points = cup_object_points(cup_radius_m_);
  } else {
    RCLCPP_WARN(get_logger(), "unsupported class_id=%d, skipping", det.class_id);
    return std::nullopt;
  }

  // solvePnP: IPPE first, fallback to ITERATIVE
  cv::Vec3d rvec, tvec;
  bool solved = false;

  // IPPE_SQUARE requires exactly 4 coplanar points; use IPPE for general coplanar case
  std::vector<int> methods = {cv::SOLVEPNP_IPPE, cv::SOLVEPNP_ITERATIVE};
  for (int method : methods) {
    try {
      solved = cv::solvePnP(
        object_points, image_points,
        intrinsics_.K, intrinsics_.D,
        rvec, tvec, false, method);
      if (solved) {break;}
    } catch (const cv::Exception & ex) {
      RCLCPP_DEBUG(get_logger(),
        "solvePnP method=%d failed: %s; trying next", method, ex.what());
    }
  }

  if (!solved) {
    RCLCPP_WARN(get_logger(),
      "solvePnP failed for class=%d, skipping", det.class_id);
    return std::nullopt;
  }

  // tvec is in camera frame; convert to target_frame via TF
  const auto & t = tf_stamped.transform.translation;
  const auto & q = tf_stamped.transform.rotation;

  cv::Matx33d R_world_cam = quat_to_rot(q.x, q.y, q.z, q.w);

  // camera origin in world
  cv::Vec3d cam_origin(t.x, t.y, t.z);

  // point in world frame
  cv::Vec3d tvec_world = R_world_cam * tvec + cam_origin;

  // build block pose
  omx_interfaces::msg::BlockPose block;
  block.header.stamp = tf_stamped.header.stamp;
  block.header.frame_id = target_frame_;

  block.pose.header = block.header;
  block.pose.pose.position.x = tvec_world[0];
  block.pose.pose.position.y = tvec_world[1];
  block.pose.pose.position.z = is_box ? box_output_z_m_ : cup_output_z_m_;

  if (is_box) {
    double yaw = box_yaw_world(rvec, R_world_cam);
    block.pose.pose.orientation = yaw_to_quat(yaw);
    block.yaw_confidence = static_cast<float>(min_kp_conf);
    block.color = det.color.empty() ? "unknown" : det.color;
    block.confidence = det.detection_confidence;
  } else {
    // cup: circular rim => yaw meaningless
    block.pose.pose.orientation = identity_quat();
    block.yaw_confidence = 0.0f;
    block.color = "cup";
    block.confidence = det.detection_confidence;
  }

  block.grasp_pose = block.pose;

  return block;
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
