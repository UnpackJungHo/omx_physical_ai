#include <cmath>
#include <optional>
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

std::optional<std::vector<cv::Point3d>> project_keypoints_to_plane(
  const std::vector<cv::Point2f> & image_points,
  const cv::Mat & K, const cv::Mat & D,
  const cv::Matx33d & R_world_cam, const cv::Vec3d & t_world_cam,
  double plane_z)
{
  if (image_points.empty()) {return std::nullopt;}

  std::vector<cv::Point2f> undistorted;
  cv::undistortPoints(image_points, undistorted, K, D);

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

// 합성 카메라: fx=fy=500, cx=cy=320/240, no distortion. 카메라가 world (0,0,H)
// 에 놓여 -z 방향(아래)을 본다. OpenCV camera frame: +x=image right, +y=image
// down, +z=forward. 즉 R_world_cam 은 +x_cam -> +x_world, +y_cam -> -y_world,
// +z_cam -> -z_world (180-deg roll about x_world).
cv::Mat make_K(double fx = 500.0, double fy = 500.0,
  double cx = 320.0, double cy = 240.0)
{
  return (cv::Mat_<double>(3, 3) <<
    fx, 0.0, cx,
    0.0, fy, cy,
    0.0, 0.0, 1.0);
}

cv::Mat make_zero_D() {return cv::Mat::zeros(1, 5, CV_64F);}

cv::Matx33d R_downward_camera()
{
  // x_cam -> x_world, y_cam -> -y_world, z_cam -> -z_world
  return cv::Matx33d(
    1.0,  0.0,  0.0,
    0.0, -1.0,  0.0,
    0.0,  0.0, -1.0);
}

constexpr double kEps = 1e-5;
constexpr double kPosEps = 1e-4;
constexpr double kDeg = M_PI / 180.0;

}  // namespace

// ============================================================
// quaternion tests (kept from before)
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

TEST(QuaternionToRot, Z180FlipsXY)
{
  auto R = quat_to_rot(0.0, 0.0, 1.0, 0.0);
  EXPECT_NEAR(R(0, 0), -1.0, kEps);
  EXPECT_NEAR(R(1, 1), -1.0, kEps);
  EXPECT_NEAR(R(2, 2),  1.0, kEps);
}

// ============================================================
// ray-plane intersection tests
// ============================================================

// 카메라가 (0,0,1) 위에서 아래를 본다. 이미지 중심 픽셀(cx,cy) 의 ray 는 정확히
// -z_world 방향이므로 plane z=0 과 (0,0,0) 에서 만난다.
TEST(RayPlane, CenterPixelHitsOriginOnGround)
{
  auto K = make_K();
  auto D = make_zero_D();
  auto R = R_downward_camera();
  cv::Vec3d t(0.0, 0.0, 1.0);

  std::vector<cv::Point2f> px{{320.0f, 240.0f}};
  auto result = project_keypoints_to_plane(px, K, D, R, t, 0.0);
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 1u);
  EXPECT_NEAR((*result)[0].x, 0.0, kPosEps);
  EXPECT_NEAR((*result)[0].y, 0.0, kPosEps);
  EXPECT_NEAR((*result)[0].z, 0.0, kPosEps);
}

// 픽셀 (cx + fx*dx_world, cy) 의 ray 는 camera frame 에서 (dx_world, 0, 1) 방향.
// world frame 으로 회전 시 (dx_world, 0, -1). camera 높이 H=1 에서 plane z=0 과
// 교점 = (dx_world*1, 0, 0).
TEST(RayPlane, OffsetPixelMapsToScaledWorldXY)
{
  auto K = make_K();
  auto D = make_zero_D();
  auto R = R_downward_camera();
  cv::Vec3d t(0.0, 0.0, 1.0);

  // shift +50 px in u, -30 px in v.
  std::vector<cv::Point2f> px{{370.0f, 210.0f}};  // (cx+50, cy-30)
  auto result = project_keypoints_to_plane(px, K, D, R, t, 0.0);
  ASSERT_TRUE(result.has_value());
  // expected: x = +50/500 * 1 = 0.10, y = -(-30/500) * 1 = +0.06 (y_cam->-y_world)
  EXPECT_NEAR((*result)[0].x,  0.10, kPosEps);
  EXPECT_NEAR((*result)[0].y,  0.06, kPosEps);
}

// plane_z = 0.030 (박스 윗면) 으로 올리면 광선 진행 거리가 줄어 (1-0.03)배로
// 스케일.
TEST(RayPlane, NonZeroPlaneScalesXY)
{
  auto K = make_K();
  auto D = make_zero_D();
  auto R = R_downward_camera();
  cv::Vec3d t(0.0, 0.0, 1.0);
  const double plane_z = 0.030;

  std::vector<cv::Point2f> px{{370.0f, 240.0f}};  // u shift +50
  auto result = project_keypoints_to_plane(px, K, D, R, t, plane_z);
  ASSERT_TRUE(result.has_value());
  const double expected_x = 0.10 * (1.0 - plane_z);
  EXPECT_NEAR((*result)[0].x, expected_x, kPosEps);
  EXPECT_NEAR((*result)[0].y, 0.0, kPosEps);
  EXPECT_NEAR((*result)[0].z, plane_z, kPosEps);
}

// ray 가 plane 과 평행(horizontal)이면 nullopt.
TEST(RayPlane, HorizontalRayFails)
{
  auto K = make_K();
  auto D = make_zero_D();
  // camera 가 +x 방향을 보는 자세 (z_cam = +x_world). plane z=0 과 평행.
  cv::Matx33d R(
    0.0, 0.0, 1.0,
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0);
  cv::Vec3d t(0.0, 0.0, 0.5);
  std::vector<cv::Point2f> px{{320.0f, 240.0f}};
  auto result = project_keypoints_to_plane(px, K, D, R, t, 0.0);
  EXPECT_FALSE(result.has_value());
}

// 카메라가 plane 아래에서 위 plane 을 향해도(즉 lambda > 0) 정상 처리되어야
// 하지만, 위쪽 plane 을 카메라가 등지면(lambda<0) 실패해야 함.
TEST(RayPlane, BehindCameraFails)
{
  auto K = make_K();
  auto D = make_zero_D();
  auto R = R_downward_camera();  // 아래를 봄
  cv::Vec3d t(0.0, 0.0, 1.0);
  // plane z = 2.0 -> camera 가 등지고 있는 위쪽 -> intersect behind camera
  std::vector<cv::Point2f> px{{320.0f, 240.0f}};
  auto result = project_keypoints_to_plane(px, K, D, R, t, 2.0);
  EXPECT_FALSE(result.has_value());
}

// ============================================================
// centroid + box_yaw_from_corners
// ============================================================

TEST(Centroid, MeanOfFourCorners)
{
  std::vector<cv::Point3d> pts{
    {1.0, 2.0, 0.0},
    {3.0, 2.0, 0.0},
    {3.0, 4.0, 0.0},
    {1.0, 4.0, 0.0},
  };
  auto c = centroid_xy(pts);
  EXPECT_NEAR(c.x, 2.0, kEps);
  EXPECT_NEAR(c.y, 3.0, kEps);
}

// 축 정렬 정사각형 -> yaw = 0.
TEST(BoxYawFromCorners, AxisAlignedZero)
{
  std::vector<cv::Point3d> p{
    {-0.5, -0.5, 0.0},
    { 0.5, -0.5, 0.0},
    { 0.5,  0.5, 0.0},
    {-0.5,  0.5, 0.0},
  };
  EXPECT_NEAR(box_yaw_from_corners(p), 0.0, kEps);
}

// 30도 회전된 정사각형 -> yaw = 30 deg.
TEST(BoxYawFromCorners, Rotated30Deg)
{
  const double a = 30.0 * kDeg;
  const double c = std::cos(a), s = std::sin(a);
  auto rot = [&](double x, double y) {
      return cv::Point3d(c * x - s * y, s * x + c * y, 0.0);
    };
  std::vector<cv::Point3d> p{
    rot(-0.5, -0.5),
    rot( 0.5, -0.5),
    rot( 0.5,  0.5),
    rot(-0.5,  0.5),
  };
  EXPECT_NEAR(box_yaw_from_corners(p), a, kEps);
}

// 노이즈가 있어도 평균(circular mean) 이 동작해야 함.
TEST(BoxYawFromCorners, NoisyRotatedStillClose)
{
  const double a = -15.0 * kDeg;
  const double c = std::cos(a), s = std::sin(a);
  auto rot = [&](double x, double y) {
      return cv::Point3d(c * x - s * y, s * x + c * y, 0.0);
    };
  std::vector<cv::Point3d> p{
    rot(-0.5 + 0.005, -0.5 - 0.003),
    rot( 0.5 - 0.004, -0.5 + 0.002),
    rot( 0.5 + 0.001,  0.5 + 0.006),
    rot(-0.5 - 0.002,  0.5 - 0.005),
  };
  EXPECT_NEAR(box_yaw_from_corners(p), a, 2.0 * kDeg);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
