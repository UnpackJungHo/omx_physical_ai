#pragma once

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "omx_interfaces/msg/block_pose.hpp"
#include "omx_interfaces/msg/keypoint_detection.hpp"
#include "omx_interfaces/srv/get_block_poses.hpp"
#include "omx_interfaces/srv/get_keypoint_detections.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace omx_perception_world_pose
{

struct CameraIntrinsics
{
  cv::Mat K;  // 3x3 camera matrix
  cv::Mat D;  // distortion coefficients
};

class BoxCupWorldPoseNode : public rclcpp::Node
{
public:
  explicit BoxCupWorldPoseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void handle_get_block_poses(
    const std::shared_ptr<omx_interfaces::srv::GetBlockPoses::Request> request,
    std::shared_ptr<omx_interfaces::srv::GetBlockPoses::Response> response);

  std::optional<omx_interfaces::srv::GetKeypointDetections::Response>
  call_keypoint_service(bool publish_debug);

  std::optional<geometry_msgs::msg::TransformStamped>
  lookup_transform(const builtin_interfaces::msg::Time & stamp);

  std::optional<omx_interfaces::msg::BlockPose> process_detection(
    const omx_interfaces::msg::KeypointDetection & det,
    const geometry_msgs::msg::TransformStamped & transform);

  CameraIntrinsics load_intrinsics(const std::string & yaml_path);

  // parameters
  std::string keypoints_service_name_;
  std::string world_service_name_;
  std::string target_frame_;
  std::string camera_frame_;
  double cube_size_m_;
  double box_output_z_m_;
  double cup_radius_m_;
  double cup_output_z_m_;
  double min_keypoint_confidence_;
  double keypoints_timeout_sec_;
  double tf_lookup_timeout_sec_;
  std::vector<int64_t> keypoint_order_;

  CameraIntrinsics intrinsics_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Client<omx_interfaces::srv::GetKeypointDetections>::SharedPtr keypoints_client_;
  rclcpp::Service<omx_interfaces::srv::GetBlockPoses>::SharedPtr world_pose_srv_;

  rclcpp::CallbackGroup::SharedPtr cb_group_;
};

}  // namespace omx_perception_world_pose
