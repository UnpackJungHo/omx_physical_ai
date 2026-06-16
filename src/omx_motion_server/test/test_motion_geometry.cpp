// omx_motion_server — test_motion_geometry.cpp
//
// motion_geometry 순수 함수의 결정적 단위 테스트.
// 로봇/MoveIt/TF 없이 실행되며, yaw 정렬 수식의 회귀를 잡는다.

#include <cmath>

#include <gtest/gtest.h>

#include "omx_motion_server/motion_geometry.hpp"

using omx_motion_server::jaw_axis_yaw_from_quaternion;
using omx_motion_server::position_in_workspace_box;
using omx_motion_server::wrap_to_pm45;

namespace
{
constexpr double kTol = 1e-9;
}

// ── jaw_axis_yaw_from_quaternion ─────────────────────────────────────────
// end_effector_link 의 +y 축이 world 에서 어느 방향을 향하는지(heading).

TEST(JawAxisYaw, IdentityPointsToWorldPlusY)
{
  // 회전 없음 -> +y 축은 그대로 world +y -> heading = +π/2.
  EXPECT_NEAR(jaw_axis_yaw_from_quaternion(0.0, 0.0, 0.0, 1.0), M_PI / 2.0, kTol);
}

TEST(JawAxisYaw, Yaw90AboutZPointsToWorldMinusX)
{
  // z 축 +90° 회전 -> +y 축이 world -x 방향 -> heading = π.
  const double s = std::sin(M_PI / 4.0);
  const double c = std::cos(M_PI / 4.0);
  EXPECT_NEAR(jaw_axis_yaw_from_quaternion(0.0, 0.0, s, c), M_PI, kTol);
}

TEST(JawAxisYaw, Yaw180AboutZPointsToWorldMinusY)
{
  // z 축 180° 회전 -> +y 축이 world -y -> heading = -π/2.
  EXPECT_NEAR(jaw_axis_yaw_from_quaternion(0.0, 0.0, 1.0, 0.0), -M_PI / 2.0, kTol);
}

// ── wrap_to_pm45 ─────────────────────────────────────────────────────────
// 박스 90° 대칭으로 잔차각을 (-π/4, π/4] 로 접는다.

TEST(WrapToPm45, ZeroStaysZero)
{
  EXPECT_NEAR(wrap_to_pm45(0.0), 0.0, kTol);
}

TEST(WrapToPm45, PlusQuarterPiBoundaryStays)
{
  // +π/4 는 경계 (조건 `> π/4` 이 거짓) 라 그대로 유지된다.
  EXPECT_NEAR(wrap_to_pm45(M_PI / 4.0), M_PI / 4.0, kTol);
}

TEST(WrapToPm45, SixtyDegFoldsToMinusThirty)
{
  // 60° -> 90° 대칭으로 -30°.
  EXPECT_NEAR(wrap_to_pm45(M_PI / 3.0), -M_PI / 6.0, kTol);
}

TEST(WrapToPm45, MinusSixtyDegFoldsToPlusThirty)
{
  // -60° -> +30°.
  EXPECT_NEAR(wrap_to_pm45(-M_PI / 3.0), M_PI / 6.0, kTol);
}

TEST(WrapToPm45, HalfPiFoldsToZero)
{
  // 90° 는 박스의 한 변 차이 -> 0.
  EXPECT_NEAR(wrap_to_pm45(M_PI / 2.0), 0.0, kTol);
}

TEST(WrapToPm45, MinusThreeQuarterPiFoldsToPlusQuarter)
{
  // -135° -> +45° (경계로 접힘).
  EXPECT_NEAR(wrap_to_pm45(-3.0 * M_PI / 4.0), M_PI / 4.0, kTol);
}

// ── position_in_workspace_box ────────────────────────────────────────────
// MoveToPose 의 goal 사전 거부 / execute 직전 검사 단일 소스.
// 한계: |x|<=0.3, |y|<=0.3, 0.0<=z<=0.45 로 가정.

namespace
{
constexpr double kXMax = 0.3;
constexpr double kYMax = 0.3;
constexpr double kZMin = 0.0;
constexpr double kZMax = 0.45;

bool in_box(double x, double y, double z)
{
  return position_in_workspace_box(x, y, z, kXMax, kYMax, kZMin, kZMax);
}
}  // namespace

TEST(WorkspaceBox, CenterIsInside)
{
  EXPECT_TRUE(in_box(0.1, 0.1, 0.2));
}

TEST(WorkspaceBox, BoundaryIsInsideInclusive)
{
  // 경계값은 포함(<=, >=).
  EXPECT_TRUE(in_box(kXMax, -kYMax, kZMax));
  EXPECT_TRUE(in_box(0.0, 0.0, kZMin));
}

TEST(WorkspaceBox, OutsideXRejected)
{
  EXPECT_FALSE(in_box(kXMax + 1e-6, 0.0, 0.2));
  EXPECT_FALSE(in_box(-(kXMax + 1e-6), 0.0, 0.2));
}

TEST(WorkspaceBox, OutsideYRejected)
{
  EXPECT_FALSE(in_box(0.0, kYMax + 1e-6, 0.2));
}

TEST(WorkspaceBox, BelowFloorAndAboveCeilingRejected)
{
  EXPECT_FALSE(in_box(0.0, 0.0, kZMin - 1e-6));
  EXPECT_FALSE(in_box(0.0, 0.0, kZMax + 1e-6));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
