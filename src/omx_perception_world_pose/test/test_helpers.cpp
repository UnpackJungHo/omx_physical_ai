#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

// ============================================================
// Pure helpers duplicated from the production translation unit.
// No rclcpp dependency here.
// ============================================================

namespace
{

std::vector<cv::Point3f> box_object_points(double cube_size_m)
{
  float half = static_cast<float>(cube_size_m / 2.0);
  return {
    {-half, -half, 0.0f},
    { half, -half, 0.0f},
    { half,  half, 0.0f},
    {-half,  half, 0.0f},
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

cv::Matx33d quat_to_rot(double qx, double qy, double qz, double qw)
{
  double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (n < 1e-9) {return cv::Matx33d::eye();}
  qx /= n; qy /= n; qz /= n; qw /= n;
  return cv::Matx33d(
    1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw),
    2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw),
    2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy));
}

double box_yaw_world(const cv::Vec3d & rvec, const cv::Matx33d & R_world_cam)
{
  cv::Matx33d R_cam_obj;
  cv::Rodrigues(rvec, R_cam_obj);
  cv::Matx33d R_world_obj = R_world_cam * R_cam_obj;
  cv::Vec3d x_world(R_world_obj(0, 0), R_world_obj(1, 0), R_world_obj(2, 0));
  return std::atan2(x_world[1], x_world[0]);
}

constexpr double kEps = 1e-5;
constexpr double kDeg = M_PI / 180.0;

}  // namespace

// ============================================================
// object_points_box_square_corners
// ============================================================

TEST(ObjectPoints, BoxSquareCorners)
{
  auto pts = box_object_points(0.030);
  ASSERT_EQ(pts.size(), 4u);
  float half = 0.015f;
  EXPECT_NEAR(pts[0].x, -half, kEps);
  EXPECT_NEAR(pts[0].y, -half, kEps);
  EXPECT_NEAR(pts[1].x,  half, kEps);
  EXPECT_NEAR(pts[1].y, -half, kEps);
  EXPECT_NEAR(pts[2].x,  half, kEps);
  EXPECT_NEAR(pts[2].y,  half, kEps);
  EXPECT_NEAR(pts[3].x, -half, kEps);
  EXPECT_NEAR(pts[3].y,  half, kEps);
  for (const auto & p : pts) {EXPECT_NEAR(p.z, 0.0f, kEps);}
}

// ============================================================
// object_points_cup_rim_cardinals
// ============================================================

TEST(ObjectPoints, CupRimCardinals)
{
  auto pts = cup_object_points(0.07);
  ASSERT_EQ(pts.size(), 4u);
  float r = 0.07f;
  EXPECT_NEAR(pts[0].x, -r, kEps);
  EXPECT_NEAR(pts[0].y,  0.0f, kEps);
  EXPECT_NEAR(pts[1].x,  0.0f, kEps);
  EXPECT_NEAR(pts[1].y, -r, kEps);
  EXPECT_NEAR(pts[2].x,  r, kEps);
  EXPECT_NEAR(pts[2].y,  0.0f, kEps);
  EXPECT_NEAR(pts[3].x,  0.0f, kEps);
  EXPECT_NEAR(pts[3].y,  r, kEps);
  for (const auto & p : pts) {EXPECT_NEAR(p.z, 0.0f, kEps);}
}

// ============================================================
// quaternion_identity_is_eye
// ============================================================

TEST(QuaternionToRot, IdentityIsEye)
{
  auto R = quat_to_rot(0.0, 0.0, 0.0, 1.0);
  auto I = cv::Matx33d::eye();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(R(i, j), I(i, j), kEps);
    }
  }
}

// ============================================================
// quaternion_z180_flips_xy
// ============================================================

TEST(QuaternionToRot, Z180FlipsXY)
{
  // q = (0, 0, 1, 0) represents 180-deg rotation about Z
  auto R = quat_to_rot(0.0, 0.0, 1.0, 0.0);
  // x -> -x, y -> -y, z -> z
  EXPECT_NEAR(R(0, 0), -1.0, kEps);
  EXPECT_NEAR(R(1, 1), -1.0, kEps);
  EXPECT_NEAR(R(2, 2),  1.0, kEps);
}

// ============================================================
// box_yaw_identity_zero
// ============================================================

TEST(BoxYaw, IdentityZero)
{
  cv::Vec3d rvec(0.0, 0.0, 0.0);
  cv::Matx33d R_I = cv::Matx33d::eye();
  double yaw = box_yaw_world(rvec, R_I);
  EXPECT_NEAR(yaw, 0.0, kEps);
}

// ============================================================
// box_yaw_object_30deg (object rotated 30° about Z in camera frame)
// ============================================================

TEST(BoxYaw, Object30Deg)
{
  double angle = 30.0 * kDeg;
  cv::Vec3d rvec(0.0, 0.0, angle);
  cv::Matx33d R_I = cv::Matx33d::eye();
  double yaw = box_yaw_world(rvec, R_I);
  EXPECT_NEAR(yaw, angle, kEps);
}

// ============================================================
// box_yaw_world_cam_90deg (camera->world 90° about Z)
// ============================================================

TEST(BoxYaw, WorldCam90Deg)
{
  cv::Vec3d rvec(0.0, 0.0, 0.0);
  double angle = 90.0 * kDeg;
  cv::Matx33d R_world_cam = quat_to_rot(
    0.0, 0.0, std::sin(angle / 2.0), std::cos(angle / 2.0));
  double yaw = box_yaw_world(rvec, R_world_cam);
  EXPECT_NEAR(yaw, angle, kEps);
}

// ============================================================
// box_yaw_compose_30_plus_90 = 120
// ============================================================

TEST(BoxYaw, Compose30Plus90)
{
  double obj_angle = 30.0 * kDeg;
  double cam_angle = 90.0 * kDeg;
  cv::Vec3d rvec(0.0, 0.0, obj_angle);
  cv::Matx33d R_world_cam = quat_to_rot(
    0.0, 0.0, std::sin(cam_angle / 2.0), std::cos(cam_angle / 2.0));
  double yaw = box_yaw_world(rvec, R_world_cam);
  EXPECT_NEAR(yaw, obj_angle + cam_angle, kEps);
}

// ============================================================
// quaternion_from_yaw_roundtrip
// ============================================================

TEST(BoxYaw, QuaternionFromYawRoundtrip)
{
  double input_yaw = 45.0 * kDeg;
  double qz = std::sin(input_yaw / 2.0);
  double qw = std::cos(input_yaw / 2.0);

  // q = (0, 0, qz, qw) -> R
  cv::Matx33d R = quat_to_rot(0.0, 0.0, qz, qw);

  // recover yaw from x-axis
  cv::Vec3d x_world(R(0, 0), R(1, 0), R(2, 0));
  double recovered_yaw = std::atan2(x_world[1], x_world[0]);
  EXPECT_NEAR(recovered_yaw, input_yaw, kEps);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
