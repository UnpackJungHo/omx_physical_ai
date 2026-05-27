#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "omx_interfaces/msg/keypoint_detection.hpp"
#include "omx_interfaces/srv/get_block_poses.hpp"
#include "omx_interfaces/srv/get_keypoint_detections.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <opencv2/core.hpp>

namespace omx_perception_world_pose
{

struct CameraIntrinsics
{
  cv::Mat K;  // 카메라 내부 파라미터 행렬(3 x 3)
  cv::Mat D;  // 랜즈 왜곡 계수
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
  call_keypoint_service(int num_samples, int min_valid_samples, bool publish_debug);

  std::optional<geometry_msgs::msg::TransformStamped>
  lookup_transform(const builtin_interfaces::msg::Time & stamp);

  struct DetectionSample
  {
    int class_id;
    std::string color;
    double x;             // world frame
    double y;             // world frame
    double yaw_world;     // box only; ignored when has_yaw=false
    bool has_yaw;
    float detection_confidence;
    float yaw_confidence;
    // cup 일 때만 채움: 4 모서리의 world (x, y) (keypoint_order_ 순서 그대로).
    // box 는 빈 벡터 유지. skill 측 cup-내부 박스 판정에 사용.
    std::vector<cv::Point2d> corners_world;
    // box detection 일 때만 의미 있음: 같은 sample 안의 cup detection 의 image
    // keypoint polygon 에 box image center 가 들어갔는지. 카메라 광선이 컵
    // prism 을 통과하면 박스의 실제 z 와 무관하게 "컵 위/안" 으로 분류된다.
    // cup detection 자체이거나 같은 sample 에 cup 이 없으면 false.
    bool in_cup_image_polygon = false;
  };

  std::optional<DetectionSample> process_detection(
    const omx_interfaces::msg::KeypointDetection & det,
    const geometry_msgs::msg::TransformStamped & transform);

  CameraIntrinsics load_intrinsics(const std::string & yaml_path);

  // parameters
  std::string keypoints_service_name_;
  std::string world_service_name_;
  std::string target_frame_;
  std::string camera_frame_;
  // Ray-plane intersection prior: each detection's 4 keypoints are assumed to
  // lie on a horizontal plane at world z = *_top_z_m. Output z is set to
  // *_output_z_m (typically object center/grasp height, distinct from rim).
  double box_top_z_m_;
  double box_output_z_m_;
  double cup_top_z_m_;
  double cup_output_z_m_;
  // 측정된 perception (x,y) - ground truth (x,y) bias 의 반대 부호로 채워
  // process_detection 결과에 더해 보정한다. eye-in-hand 구성이므로 scan pose
  // 가 바뀌면 이 값도 재교정해야 한다.
  double world_offset_x_m_;
  double world_offset_y_m_;
  double min_keypoint_confidence_;
  double keypoints_timeout_sec_;
  double tf_lookup_timeout_sec_;
  std::vector<int64_t> keypoint_order_;
  int num_samples_;
  int min_valid_samples_;
  double cluster_match_tolerance_m_;
  // box cluster 의 in_cup_image_polygon vote 비율이 이 임계값 이상이면 응답에서
  // 제외한다. 0.0 이면 image-space 필터 비활성. 1.0 이면 모든 sample 에서 컵
  // image polygon 안이어야 제외.
  double cup_image_polygon_vote_threshold_;

  CameraIntrinsics intrinsics_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Client<omx_interfaces::srv::GetKeypointDetections>::SharedPtr keypoints_client_;
  rclcpp::Service<omx_interfaces::srv::GetBlockPoses>::SharedPtr world_pose_srv_;

  rclcpp::CallbackGroup::SharedPtr cb_group_;
};

}  // namespace omx_perception_world_pose
