# box_cup_world_pose + PnP 기하 C++ 이식 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Python `box_cup_world_pose_node` 와 PnP 기하 헬퍼를 새 ament_cmake 패키지 `omx_perception_cpp` 의 C++ 노드/라이브러리로 이식한다.

**Architecture:** 순수 기하 함수를 rclcpp 비의존 라이브러리 `pnp_geometry` 로 분리해 gtest 로 검증한다. `box_cup_world_pose_node` 는 그 라이브러리를 쓰는 rclcpp 서비스 노드로, 서버/클라이언트 콜백 그룹을 분리한 MultiThreadedExecutor 로 서비스 체이닝 deadlock 을 피한다. Python YOLO 노드와 `pnp_geometry.py` 는 그대로 둔다.

**Tech Stack:** C++17, rclcpp, tf2_ros, OpenCV(core/calib3d), yaml-cpp, ament_cmake, ament_cmake_gtest.

---

## File Structure

- Create: `src/omx_perception_cpp/package.xml` — ament_cmake 패키지 매니페스트.
- Create: `src/omx_perception_cpp/CMakeLists.txt` — 라이브러리/노드/테스트 빌드.
- Create: `src/omx_perception_cpp/include/omx_perception_cpp/pnp_geometry.hpp` — 순수 기하 함수 선언.
- Create: `src/omx_perception_cpp/src/pnp_geometry.cpp` — 순수 기하 함수 구현.
- Create: `src/omx_perception_cpp/src/box_cup_world_pose_node.cpp` — rclcpp 서비스 노드.
- Create: `src/omx_perception_cpp/test/test_pnp_geometry.cpp` — gtest 단위 테스트.
- Delete: `src/omx_perception/omx_perception/box_cup_world_pose_node.py`
- Delete: `src/omx_perception/test/test_box_cup_world_pose_helpers.py`
- Modify: `src/omx_perception/setup.py` — `box_cup_world_pose_node` entry point 제거.
- Modify: `src/omx_perception/launch/perception.launch.py` — world pose Node 의 package/executable 변경.

빌드/테스트 명령은 워크스페이스 루트 `/home/kjhz/omx_ws` 에서 실행하며, ROS2 환경은 셸 프로필에서 source 된 상태를 가정한다.

---

### Task 1: 패키지 스캐폴드 + pnp_geometry 헤더/스텁

순수 기하 라이브러리의 모든 함수 시그니처를 헤더에 확정하고, 본문은 예외를 던지는 스텁으로 두어 이후 테스트가 컴파일·링크되도록 한다.

**Files:**
- Create: `src/omx_perception_cpp/package.xml`
- Create: `src/omx_perception_cpp/CMakeLists.txt`
- Create: `src/omx_perception_cpp/include/omx_perception_cpp/pnp_geometry.hpp`
- Create: `src/omx_perception_cpp/src/pnp_geometry.cpp`
- Create: `src/omx_perception_cpp/test/test_pnp_geometry.cpp`

- [ ] **Step 1: package.xml 작성**

`src/omx_perception_cpp/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>omx_perception_cpp</name>
  <version>0.0.0</version>
  <description>OMX perception C++ nodes — box/cup world-frame pose estimation</description>
  <maintainer email="kjhgfd6632@gmail.com">kjhz</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>omx_interfaces</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>libopencv-dev</depend>
  <depend>yaml-cpp</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: CMakeLists.txt 작성 (라이브러리 + 테스트만)**

`src/omx_perception_cpp/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(omx_perception_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(omx_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(pnp_geometry src/pnp_geometry.cpp)
target_include_directories(pnp_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pnp_geometry PUBLIC cxx_std_17)
target_link_libraries(pnp_geometry ${OpenCV_LIBS} yaml-cpp)
ament_target_dependencies(pnp_geometry omx_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pnp_geometry test/test_pnp_geometry.cpp)
  target_link_libraries(test_pnp_geometry pnp_geometry ${OpenCV_LIBS})
  ament_target_dependencies(test_pnp_geometry omx_interfaces)
endif()

ament_package()
```

- [ ] **Step 3: pnp_geometry.hpp 작성 (전체 선언)**

`src/omx_perception_cpp/include/omx_perception_cpp/pnp_geometry.hpp`:

```cpp
#ifndef OMX_PERCEPTION_CPP__PNP_GEOMETRY_HPP_
#define OMX_PERCEPTION_CPP__PNP_GEOMETRY_HPP_

#include <array>
#include <string>

#include <opencv2/core.hpp>

namespace omx_perception_cpp
{

/// solvePnP 결과: 성공 여부와 object->camera rvec/tvec.
struct PnpResult
{
  bool ok;
  cv::Vec3d rvec;
  cv::Vec3d tvec;
};

/// 카메라 intrinsics: 3x3 행렬과 왜곡 계수(Nx1).
struct Intrinsics
{
  cv::Matx33d camera_matrix;
  cv::Mat dist_coeffs;
};

/// KeypointDetection class 별 solvePnP 용 표준 4x3 object-points (CV_64F).
cv::Mat objectPointsForClass(int class_id, double cube_size_m, double cup_radius_m);

/// 평면 타깃 solvePnP: IPPE 의 2중해 모호성을 해소한다.
PnpResult solvePnpPlanarDisambiguated(
  const cv::Mat & object_points,
  const cv::Mat & image_points,
  const cv::Matx33d & camera_matrix,
  const cv::Mat & dist_coeffs);

/// 정육면체 90° 대칭으로 yaw 를 (-pi/4, pi/4] 로 접는다.
double wrapYawToPm45(double yaw);

/// YAML 파일에서 camera_matrix 와 distortion_coefficients 를 읽는다.
Intrinsics loadIntrinsics(const std::string & path);

/// quaternion (x, y, z, w) -> 3x3 회전 행렬.
cv::Matx33d quaternionToRotationMatrix(double x, double y, double z, double w);

/// world-z yaw 회전 -> quaternion {x, y, z, w}.
std::array<double, 4> quaternionFromYaw(double yaw);

/// solvePnP rvec 과 camera->world 회전으로 box +x 축의 world yaw 산출.
double boxYawWorld(const cv::Vec3d & rvec, const cv::Matx33d & rotation_world_cam);

}  // namespace omx_perception_cpp

#endif  // OMX_PERCEPTION_CPP__PNP_GEOMETRY_HPP_
```

- [ ] **Step 4: pnp_geometry.cpp 스텁 작성**

`src/omx_perception_cpp/src/pnp_geometry.cpp`:

```cpp
#include "omx_perception_cpp/pnp_geometry.hpp"

#include <stdexcept>

namespace omx_perception_cpp
{

cv::Mat objectPointsForClass(int, double, double)
{
  throw std::logic_error("objectPointsForClass not implemented");
}

PnpResult solvePnpPlanarDisambiguated(
  const cv::Mat &, const cv::Mat &, const cv::Matx33d &, const cv::Mat &)
{
  throw std::logic_error("solvePnpPlanarDisambiguated not implemented");
}

double wrapYawToPm45(double)
{
  throw std::logic_error("wrapYawToPm45 not implemented");
}

Intrinsics loadIntrinsics(const std::string &)
{
  throw std::logic_error("loadIntrinsics not implemented");
}

cv::Matx33d quaternionToRotationMatrix(double, double, double, double)
{
  throw std::logic_error("quaternionToRotationMatrix not implemented");
}

std::array<double, 4> quaternionFromYaw(double)
{
  throw std::logic_error("quaternionFromYaw not implemented");
}

double boxYawWorld(const cv::Vec3d &, const cv::Matx33d &)
{
  throw std::logic_error("boxYawWorld not implemented");
}

}  // namespace omx_perception_cpp
```

- [ ] **Step 5: test_pnp_geometry.cpp 초기 작성 (sanity test)**

`src/omx_perception_cpp/test/test_pnp_geometry.cpp`:

```cpp
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "omx_interfaces/msg/keypoint_detection.hpp"
#include "omx_perception_cpp/pnp_geometry.hpp"

using omx_perception_cpp::boxYawWorld;
using omx_perception_cpp::loadIntrinsics;
using omx_perception_cpp::objectPointsForClass;
using omx_perception_cpp::quaternionFromYaw;
using omx_perception_cpp::quaternionToRotationMatrix;
using omx_perception_cpp::solvePnpPlanarDisambiguated;
using omx_perception_cpp::wrapYawToPm45;
using KP = omx_interfaces::msg::KeypointDetection;

TEST(Sanity, Builds)
{
  SUCCEED();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

- [ ] **Step 6: 빌드 검증**

Run: `cd /home/kjhz/omx_ws && colcon build --symlink-install --packages-select omx_perception_cpp`
Expected: `Finished <<< omx_perception_cpp` — 빌드 성공.

- [ ] **Step 7: sanity 테스트 통과 확인**

Run: `cd /home/kjhz/omx_ws && source install/setup.bash && ./build/omx_perception_cpp/test_pnp_geometry`
Expected: `[  PASSED  ] 1 test.`

- [ ] **Step 8: Commit**

```bash
cd /home/kjhz/omx_ws
git add src/omx_perception_cpp
git commit -m "feat(perception_cpp): scaffold omx_perception_cpp package with pnp_geometry stubs

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 2: objectPointsForClass + wrapYawToPm45

**Files:**
- Modify: `src/omx_perception_cpp/test/test_pnp_geometry.cpp`
- Modify: `src/omx_perception_cpp/src/pnp_geometry.cpp`

- [ ] **Step 1: 실패 테스트 작성**

`test_pnp_geometry.cpp` 의 `TEST(Sanity, Builds)` 와 `int main` 사이에 다음 테스트를 추가한다:

```cpp
TEST(ObjectPoints, BoxReturnsSquareCorners)
{
  cv::Mat pts = objectPointsForClass(KP::CLASS_BOX, 0.030, 0.07);
  ASSERT_EQ(pts.rows, 4);
  ASSERT_EQ(pts.cols, 3);
  const double expected[4][3] = {
    {-0.015, -0.015, 0.0}, {0.015, -0.015, 0.0},
    {0.015, 0.015, 0.0}, {-0.015, 0.015, 0.0}};
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(pts.at<double>(i, j), expected[i][j], 1e-9);
    }
  }
}

TEST(ObjectPoints, CupReturnsRimCardinals)
{
  cv::Mat pts = objectPointsForClass(KP::CLASS_CUP, 0.030, 0.07);
  ASSERT_EQ(pts.rows, 4);
  ASSERT_EQ(pts.cols, 3);
  const double expected[4][3] = {
    {-0.07, 0.0, 0.0}, {0.0, -0.07, 0.0},
    {0.07, 0.0, 0.0}, {0.0, 0.07, 0.0}};
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(pts.at<double>(i, j), expected[i][j], 1e-9);
    }
  }
}

TEST(ObjectPoints, UnknownClassThrows)
{
  EXPECT_THROW(objectPointsForClass(99, 0.030, 0.07), std::invalid_argument);
}

TEST(WrapYaw, KeepsInRangeValues)
{
  for (double deg : {0.0, 30.0, 44.0, 45.0, -44.0}) {
    const double rad = deg * M_PI / 180.0;
    EXPECT_NEAR(wrapYawToPm45(rad), rad, 1e-9);
  }
}

TEST(WrapYaw, FoldsBoxSymmetry)
{
  const std::vector<std::pair<double, double>> cases = {
    {46.0, -44.0}, {90.0, 0.0}, {135.0, 45.0},
    {-45.0, 45.0}, {-46.0, 44.0}, {-135.0, 45.0}};
  for (const auto & [raw_deg, expected_deg] : cases) {
    EXPECT_NEAR(
      wrapYawToPm45(raw_deg * M_PI / 180.0),
      expected_deg * M_PI / 180.0, 1e-9);
  }
}
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/kjhz/omx_ws && colcon build --symlink-install --packages-select omx_perception_cpp && source install/setup.bash && ./build/omx_perception_cpp/test_pnp_geometry`
Expected: `ObjectPoints.*`, `WrapYaw.*` 테스트가 FAIL (스텁이 `std::logic_error` 를 던짐).

- [ ] **Step 3: objectPointsForClass / wrapYawToPm45 구현**

`pnp_geometry.cpp` 의 `#include <stdexcept>` 아래에 추가:

```cpp
#include <cmath>

#include "omx_interfaces/msg/keypoint_detection.hpp"
```

`objectPointsForClass` 스텁 본문을 교체:

```cpp
cv::Mat objectPointsForClass(int class_id, double cube_size_m, double cup_radius_m)
{
  using KP = omx_interfaces::msg::KeypointDetection;
  if (class_id == KP::CLASS_BOX) {
    const double h = cube_size_m / 2.0;
    return (cv::Mat_<double>(4, 3) <<
      -h, -h, 0.0,
      h, -h, 0.0,
      h, h, 0.0,
      -h, h, 0.0);
  }
  if (class_id == KP::CLASS_CUP) {
    const double r = cup_radius_m;
    return (cv::Mat_<double>(4, 3) <<
      -r, 0.0, 0.0,
      0.0, -r, 0.0,
      r, 0.0, 0.0,
      0.0, r, 0.0);
  }
  throw std::invalid_argument(
    "Unsupported class_id for PnP: " + std::to_string(class_id));
}
```

`wrapYawToPm45` 스텁 본문을 교체:

```cpp
double wrapYawToPm45(double yaw)
{
  const double quarter = M_PI / 2.0;
  double wrapped = std::fmod(yaw, quarter);
  if (wrapped > M_PI / 4.0) {
    wrapped -= quarter;
  } else if (wrapped <= -M_PI / 4.0) {
    wrapped += quarter;
  }
  return wrapped;
}
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/kjhz/omx_ws && colcon build --symlink-install --packages-select omx_perception_cpp && source install/setup.bash && ./build/omx_perception_cpp/test_pnp_geometry --gtest_filter=ObjectPoints.*:WrapYaw.*`
Expected: `[  PASSED  ] 5 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/kjhz/omx_ws
git add src/omx_perception_cpp
git commit -m "feat(perception_cpp): implement objectPointsForClass and wrapYawToPm45

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 3: quaternion 헬퍼 + boxYawWorld

**Files:**
- Modify: `src/omx_perception_cpp/test/test_pnp_geometry.cpp`
- Modify: `src/omx_perception_cpp/src/pnp_geometry.cpp`

- [ ] **Step 1: 실패 테스트 작성**

`test_pnp_geometry.cpp` 의 `int main` 위에 추가:

```cpp
TEST(Quaternion, IdentityReturnsEye)
{
  cv::Matx33d m = quaternionToRotationMatrix(0.0, 0.0, 0.0, 1.0);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(m(i, j), (i == j) ? 1.0 : 0.0, 1e-9);
    }
  }
}

TEST(Quaternion, Z180FlipsXAndY)
{
  cv::Matx33d m = quaternionToRotationMatrix(0.0, 0.0, 1.0, 0.0);
  cv::Vec3d p = m * cv::Vec3d(1.0, 2.0, 3.0);
  EXPECT_NEAR(p[0], -1.0, 1e-9);
  EXPECT_NEAR(p[1], -2.0, 1e-9);
  EXPECT_NEAR(p[2], 3.0, 1e-9);
}

TEST(BoxYawWorld, IdentityReturnsZero)
{
  EXPECT_NEAR(boxYawWorld(cv::Vec3d(0.0, 0.0, 0.0), cv::Matx33d::eye()), 0.0, 1e-9);
}

TEST(BoxYawWorld, ObjectRotated30AboutZ)
{
  cv::Vec3d rvec(0.0, 0.0, 30.0 * M_PI / 180.0);
  EXPECT_NEAR(boxYawWorld(rvec, cv::Matx33d::eye()), 30.0 * M_PI / 180.0, 1e-6);
}

TEST(BoxYawWorld, AppliesWorldCamRotation)
{
  cv::Matx33d wc = quaternionToRotationMatrix(
    0.0, 0.0, std::sin(M_PI / 4.0), std::cos(M_PI / 4.0));
  EXPECT_NEAR(boxYawWorld(cv::Vec3d(0.0, 0.0, 0.0), wc), M_PI / 2.0, 1e-6);
}

TEST(BoxYawWorld, ComposesObjectAndWorldCamRotations)
{
  cv::Vec3d rvec(0.0, 0.0, 30.0 * M_PI / 180.0);
  cv::Matx33d wc = quaternionToRotationMatrix(
    0.0, 0.0, std::sin(M_PI / 4.0), std::cos(M_PI / 4.0));
  EXPECT_NEAR(boxYawWorld(rvec, wc), 120.0 * M_PI / 180.0, 1e-6);
}

TEST(QuaternionFromYaw, Roundtrip)
{
  for (double deg : {-90.0, -30.0, 0.0, 45.0, 120.0}) {
    std::array<double, 4> q = quaternionFromYaw(deg * M_PI / 180.0);
    cv::Matx33d m = quaternionToRotationMatrix(q[0], q[1], q[2], q[3]);
    cv::Vec3d x = m * cv::Vec3d(1.0, 0.0, 0.0);
    EXPECT_NEAR(std::atan2(x[1], x[0]), deg * M_PI / 180.0, 1e-6);
  }
}
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/kjhz/omx_ws && colcon build --symlink-install --packages-select omx_perception_cpp && source install/setup.bash && ./build/omx_perception_cpp/test_pnp_geometry --gtest_filter=Quaternion.*:BoxYawWorld.*:QuaternionFromYaw.*`
Expected: 모두 FAIL (스텁이 `std::logic_error` 를 던짐).

- [ ] **Step 3: quaternion 헬퍼 / boxYawWorld 구현**

`pnp_geometry.cpp` 의 `#include <opencv2/calib3d.hpp>` 가 없으면 파일 상단 include 에 추가:

```cpp
#include <opencv2/calib3d.hpp>
```

`quaternionToRotationMatrix` 스텁 본문 교체:

```cpp
cv::Matx33d quaternionToRotationMatrix(double x, double y, double z, double w)
{
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm == 0.0) {
    return cv::Matx33d::eye();
  }
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
  return cv::Matx33d(
    1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w),
    2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w),
    2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y));
}
```

`quaternionFromYaw` 스텁 본문 교체:

```cpp
std::array<double, 4> quaternionFromYaw(double yaw)
{
  const double half = yaw / 2.0;
  return {0.0, 0.0, std::sin(half), std::cos(half)};
}
```

`boxYawWorld` 스텁 본문 교체:

```cpp
double boxYawWorld(const cv::Vec3d & rvec, const cv::Matx33d & rotation_world_cam)
{
  cv::Matx33d rotation_cam_obj;
  cv::Rodrigues(rvec, rotation_cam_obj);
  const cv::Matx33d rotation_world_obj = rotation_world_cam * rotation_cam_obj;
  const cv::Vec3d x_axis_world = rotation_world_obj * cv::Vec3d(1.0, 0.0, 0.0);
  return std::atan2(x_axis_world[1], x_axis_world[0]);
}
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/kjhz/omx_ws && colcon build --symlink-install --packages-select omx_perception_cpp && source install/setup.bash && ./build/omx_perception_cpp/test_pnp_geometry --gtest_filter=Quaternion.*:BoxYawWorld.*:QuaternionFromYaw.*`
Expected: `[  PASSED  ] 7 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/kjhz/omx_ws
git add src/omx_perception_cpp
git commit -m "feat(perception_cpp): implement quaternion helpers and boxYawWorld

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 4: solvePnpPlanarDisambiguated

**Files:**
- Modify: `src/omx_perception_cpp/test/test_pnp_geometry.cpp`
- Modify: `src/omx_perception_cpp/src/pnp_geometry.cpp`

- [ ] **Step 1: 실패 테스트 작성**

`test_pnp_geometry.cpp` 의 마지막 `using KP = ...;` 줄 아래에 테스트 헬퍼를 추가:

```cpp
namespace
{
cv::Matx33d rotZ(double r)
{
  const double c = std::cos(r), s = std::sin(r);
  return cv::Matx33d(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0);
}
cv::Matx33d rotX(double r)
{
  const double c = std::cos(r), s = std::sin(r);
  return cv::Matx33d(1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c);
}
cv::Matx33d rotY(double r)
{
  const double c = std::cos(r), s = std::sin(r);
  return cv::Matx33d(c, 0.0, s, 0.0, 1.0, 0.0, -s, 0.0, c);
}

const cv::Matx33d kCameraMatrix(600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0);
const cv::Mat kDistCoeffs = cv::Mat::zeros(5, 1, CV_64F);

cv::Mat projectImagePoints(
  const cv::Mat & object_points, const cv::Vec3d & rvec, const cv::Vec3d & tvec)
{
  std::vector<cv::Point2d> projected;
  cv::projectPoints(object_points, rvec, tvec, kCameraMatrix, kDistCoeffs, projected);
  cv::Mat result(static_cast<int>(projected.size()), 2, CV_64F);
  for (size_t i = 0; i < projected.size(); ++i) {
    result.at<double>(static_cast<int>(i), 0) = projected[i].x;
    result.at<double>(static_cast<int>(i), 1) = projected[i].y;
  }
  return result;
}

double meanReprojectionError(
  const cv::Mat & object_points, const cv::Vec3d & rvec, const cv::Vec3d & tvec,
  const cv::Mat & image_points)
{
  std::vector<cv::Point2d> projected;
  cv::projectPoints(object_points, rvec, tvec, kCameraMatrix, kDistCoeffs, projected);
  double total = 0.0;
  for (size_t i = 0; i < projected.size(); ++i) {
    const double dx = projected[i].x - image_points.at<double>(static_cast<int>(i), 0);
    const double dy = projected[i].y - image_points.at<double>(static_cast<int>(i), 1);
    total += std::sqrt(dx * dx + dy * dy);
  }
  return total / static_cast<double>(projected.size());
}

double cameraFrameYaw(const cv::Vec3d & rvec)
{
  cv::Matx33d rotation;
  cv::Rodrigues(rvec, rotation);
  const cv::Vec3d x_axis = rotation * cv::Vec3d(1.0, 0.0, 0.0);
  return std::atan2(x_axis[1], x_axis[0]);
}
}  // namespace
```

`int main` 위에 테스트를 추가:

```cpp
TEST(SolvePnp, RecoversKnownPose)
{
  cv::Mat object_points = objectPointsForClass(KP::CLASS_BOX, 0.030, 0.07);
  const cv::Matx33d rotation_gt = rotZ(25.0 * M_PI / 180.0) * rotX(M_PI);
  cv::Vec3d true_rvec;
  cv::Rodrigues(rotation_gt, true_rvec);
  const cv::Vec3d true_tvec(0.01, -0.02, 0.40);
  cv::Mat image_points = projectImagePoints(object_points, true_rvec, true_tvec);

  auto result = solvePnpPlanarDisambiguated(
    object_points, image_points, kCameraMatrix, kDistCoeffs);

  ASSERT_TRUE(result.ok);
  EXPECT_NEAR(result.tvec[0], 0.01, 1e-3);
  EXPECT_NEAR(result.tvec[1], -0.02, 1e-3);
  EXPECT_NEAR(result.tvec[2], 0.40, 1e-3);
  cv::Matx33d rotation;
  cv::Rodrigues(result.rvec, rotation);
  EXPECT_LT((rotation * cv::Vec3d(0.0, 0.0, 1.0))[2], 0.0);
}

TEST(SolvePnp, RecoversNonDegeneratePoses)
{
  cv::Mat object_points = objectPointsForClass(KP::CLASS_BOX, 0.030, 0.07);
  for (double yaw_deg : {-30.0, 40.0, 80.0}) {
    const cv::Matx33d rotation_gt =
      rotZ(yaw_deg * M_PI / 180.0) * rotX(M_PI) * rotY(18.0 * M_PI / 180.0);
    cv::Vec3d true_rvec;
    cv::Rodrigues(rotation_gt, true_rvec);
    const cv::Vec3d true_tvec(0.04, -0.03, 0.35);
    cv::Mat image_points = projectImagePoints(object_points, true_rvec, true_tvec);

    auto result = solvePnpPlanarDisambiguated(
      object_points, image_points, kCameraMatrix, kDistCoeffs);

    ASSERT_TRUE(result.ok);
    EXPECT_LT(
      meanReprojectionError(object_points, result.rvec, result.tvec, image_points), 0.1);
    EXPECT_NEAR(cameraFrameYaw(result.rvec), yaw_deg * M_PI / 180.0, 1.0 * M_PI / 180.0);
  }
}

TEST(SolvePnp, NearDegenerateViewReturnsValidFit)
{
  cv::Mat object_points = objectPointsForClass(KP::CLASS_BOX, 0.030, 0.07);
  const cv::Matx33d rotation_gt = rotX(M_PI) * rotY(18.0 * M_PI / 180.0);
  cv::Vec3d true_rvec;
  cv::Rodrigues(rotation_gt, true_rvec);
  const cv::Vec3d true_tvec(0.04, -0.03, 0.35);
  cv::Mat image_points = projectImagePoints(object_points, true_rvec, true_tvec);

  auto result = solvePnpPlanarDisambiguated(
    object_points, image_points, kCameraMatrix, kDistCoeffs);

  ASSERT_TRUE(result.ok);
  EXPECT_LT(
    meanReprojectionError(object_points, result.rvec, result.tvec, image_points), 1.0);
}
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/kjhz/omx_ws && colcon build --symlink-install --packages-select omx_perception_cpp && source install/setup.bash && ./build/omx_perception_cpp/test_pnp_geometry --gtest_filter=SolvePnp.*`
Expected: 모두 FAIL (스텁이 `std::logic_error` 를 던짐).

- [ ] **Step 3: solvePnpPlanarDisambiguated 구현**

`pnp_geometry.cpp` 의 include 에 추가:

```cpp
#include <limits>
#include <vector>
```

`solvePnpPlanarDisambiguated` 스텁 본문을 교체하고, 그 위에 익명 네임스페이스 헬퍼를 추가:

```cpp
namespace
{
bool objectZTowardCamera(const cv::Vec3d & rvec)
{
  cv::Matx33d rotation;
  cv::Rodrigues(rvec, rotation);
  return (rotation * cv::Vec3d(0.0, 0.0, 1.0))[2] < 0.0;
}
}  // namespace

PnpResult solvePnpPlanarDisambiguated(
  const cv::Mat & object_points,
  const cv::Mat & image_points,
  const cv::Matx33d & camera_matrix,
  const cv::Mat & dist_coeffs)
{
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  cv::Mat errors;
  const int retval = cv::solvePnPGeneric(
    object_points, image_points, camera_matrix, dist_coeffs,
    rvecs, tvecs, false, cv::SOLVEPNP_IPPE, cv::noArray(), cv::noArray(), errors);

  if (retval > 0) {
    int best_index = -1;
    double best_error = std::numeric_limits<double>::infinity();
    bool toward_exists = false;
    for (int i = 0; i < retval; ++i) {
      if (objectZTowardCamera(cv::Vec3d(rvecs[i]))) {
        toward_exists = true;
        break;
      }
    }
    for (int i = 0; i < retval; ++i) {
      const cv::Vec3d rvec(rvecs[i]);
      if (toward_exists && !objectZTowardCamera(rvec)) {
        continue;
      }
      const double error = errors.at<double>(i);
      if (error < best_error) {
        best_error = error;
        best_index = i;
      }
    }
    if (best_index >= 0) {
      return {true, cv::Vec3d(rvecs[best_index]), cv::Vec3d(tvecs[best_index])};
    }
  }

  cv::Mat rvec;
  cv::Mat tvec;
  const bool ok = cv::solvePnP(
    object_points, image_points, camera_matrix, dist_coeffs,
    rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
  if (ok) {
    return {true, cv::Vec3d(rvec), cv::Vec3d(tvec)};
  }
  return {false, cv::Vec3d(), cv::Vec3d()};
}
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/kjhz/omx_ws && colcon build --symlink-install --packages-select omx_perception_cpp && source install/setup.bash && ./build/omx_perception_cpp/test_pnp_geometry --gtest_filter=SolvePnp.*`
Expected: `[  PASSED  ] 3 tests.`

- [ ] **Step 5: Commit**

```bash
cd /home/kjhz/omx_ws
git add src/omx_perception_cpp
git commit -m "feat(perception_cpp): implement solvePnpPlanarDisambiguated

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 5: loadIntrinsics

**Files:**
- Modify: `src/omx_perception_cpp/test/test_pnp_geometry.cpp`
- Modify: `src/omx_perception_cpp/src/pnp_geometry.cpp`

- [ ] **Step 1: 실패 테스트 작성**

`test_pnp_geometry.cpp` include 에 `#include <fstream>` 를 추가하고, `int main` 위에 테스트를 추가:

```cpp
TEST(LoadIntrinsics, ParsesCameraMatrixAndDist)
{
  const std::string path = "/tmp/omx_test_intrinsics.yaml";
  std::ofstream out(path);
  out << "camera_matrix:\n";
  out << "  rows: 3\n  cols: 3\n";
  out << "  data: [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]\n";
  out << "distortion_coefficients:\n";
  out << "  data: [0.1, -0.2, 0.0, 0.0, 0.05]\n";
  out.close();

  auto intrinsics = loadIntrinsics(path);
  EXPECT_NEAR(intrinsics.camera_matrix(0, 0), 600.0, 1e-9);
  EXPECT_NEAR(intrinsics.camera_matrix(0, 2), 320.0, 1e-9);
  EXPECT_NEAR(intrinsics.camera_matrix(1, 1), 600.0, 1e-9);
  EXPECT_NEAR(intrinsics.camera_matrix(2, 2), 1.0, 1e-9);
  ASSERT_EQ(intrinsics.dist_coeffs.rows, 5);
  EXPECT_NEAR(intrinsics.dist_coeffs.at<double>(0, 0), 0.1, 1e-9);
  EXPECT_NEAR(intrinsics.dist_coeffs.at<double>(4, 0), 0.05, 1e-9);
}

TEST(LoadIntrinsics, MissingFileThrows)
{
  EXPECT_ANY_THROW(loadIntrinsics("/tmp/omx_no_such_intrinsics.yaml"));
}
```

- [ ] **Step 2: 테스트 실패 확인**

Run: `cd /home/kjhz/omx_ws && colcon build --symlink-install --packages-select omx_perception_cpp && source install/setup.bash && ./build/omx_perception_cpp/test_pnp_geometry --gtest_filter=LoadIntrinsics.*`
Expected: `LoadIntrinsics.ParsesCameraMatrixAndDist` FAIL (스텁이 `std::logic_error`). `MissingFileThrows` 는 우연히 PASS 할 수 있음.

- [ ] **Step 3: loadIntrinsics 구현**

`pnp_geometry.cpp` 의 include 에 추가:

```cpp
#include <yaml-cpp/yaml.h>
```

`loadIntrinsics` 스텁 본문을 교체:

```cpp
Intrinsics loadIntrinsics(const std::string & path)
{
  const YAML::Node data = YAML::LoadFile(path);

  const YAML::Node matrix_data = data["camera_matrix"]["data"];
  if (!matrix_data || matrix_data.size() != 9) {
    throw std::runtime_error("camera_matrix.data must hold 9 values: " + path);
  }
  cv::Matx33d camera_matrix;
  for (int i = 0; i < 9; ++i) {
    camera_matrix.val[i] = matrix_data[i].as<double>();
  }

  cv::Mat dist_coeffs;
  const YAML::Node dist_node = data["distortion_coefficients"];
  if (dist_node && dist_node["data"] && dist_node["data"].size() > 0) {
    const YAML::Node dist_data = dist_node["data"];
    dist_coeffs = cv::Mat(static_cast<int>(dist_data.size()), 1, CV_64F);
    for (std::size_t i = 0; i < dist_data.size(); ++i) {
      dist_coeffs.at<double>(static_cast<int>(i), 0) = dist_data[i].as<double>();
    }
  } else {
    dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
  }

  return {camera_matrix, dist_coeffs};
}
```

- [ ] **Step 4: 테스트 통과 확인**

Run: `cd /home/kjhz/omx_ws && colcon build --symlink-install --packages-select omx_perception_cpp && source install/setup.bash && ./build/omx_perception_cpp/test_pnp_geometry`
Expected: `[  PASSED  ] 19 tests.` (Sanity 1 + ObjectPoints 3 + WrapYaw 2 + Quaternion 2 + BoxYawWorld 4 + QuaternionFromYaw 1 + SolvePnp 3 + LoadIntrinsics 2 + QuaternionFromYaw 1 = 본 빌드의 전체 테스트).

- [ ] **Step 5: Commit**

```bash
cd /home/kjhz/omx_ws
git add src/omx_perception_cpp
git commit -m "feat(perception_cpp): implement loadIntrinsics via yaml-cpp

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 6: box_cup_world_pose_node 노드

순수 함수는 gtest 로 검증됐다. 노드는 카메라/TF 의존이라 단위 테스트 대신 빌드 성공으로 검증하고, 런타임은 Task 9 launch 변경 후 수동 확인한다.

**Files:**
- Create: `src/omx_perception_cpp/src/box_cup_world_pose_node.cpp`
- Modify: `src/omx_perception_cpp/CMakeLists.txt`

- [ ] **Step 1: box_cup_world_pose_node.cpp 작성**

`src/omx_perception_cpp/src/box_cup_world_pose_node.cpp`:

```cpp
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/header.hpp"

#include "omx_interfaces/msg/block_pose.hpp"
#include "omx_interfaces/msg/keypoint_detection.hpp"
#include "omx_interfaces/srv/get_block_poses.hpp"
#include "omx_interfaces/srv/get_keypoint_detections.hpp"

#include "omx_perception_cpp/pnp_geometry.hpp"

namespace
{
using GetBlockPoses = omx_interfaces::srv::GetBlockPoses;
using GetKeypointDetections = omx_interfaces::srv::GetKeypointDetections;
using BlockPose = omx_interfaces::msg::BlockPose;
using KeypointDetection = omx_interfaces::msg::KeypointDetection;
}  // namespace

class BoxCupWorldPoseNode : public rclcpp::Node
{
public:
  BoxCupWorldPoseNode()
  : rclcpp::Node("box_cup_world_pose")
  {
    declare_parameter<std::string>("camera_intrinsics_path", "");
    declare_parameter<std::string>(
      "keypoints_service_name", "/perception/get_box_cup_keypoints");
    declare_parameter<std::string>(
      "world_service_name", "/perception/get_box_cup_world_poses");
    declare_parameter<std::string>("target_frame", "world");
    declare_parameter<std::string>("camera_frame", "default_cam");
    declare_parameter<double>("cube_size_m", 0.030);
    declare_parameter<double>("box_output_z_m", 0.015);
    declare_parameter<double>("cup_radius_m", 0.07);
    declare_parameter<double>("cup_height_m", 0.08);
    declare_parameter<double>("cup_output_z_m", 0.08);
    declare_parameter<double>("min_keypoint_confidence", 0.10);
    declare_parameter<double>("keypoints_timeout_sec", 2.0);
    declare_parameter<std::vector<int64_t>>(
      "keypoint_order", std::vector<int64_t>{0, 1, 2, 3});

    std::filesystem::path intrinsics_path(
      get_parameter("camera_intrinsics_path").as_string());
    if (intrinsics_path.empty()) {
      throw std::runtime_error("camera_intrinsics_path parameter is empty");
    }
    if (!intrinsics_path.is_absolute()) {
      intrinsics_path = std::filesystem::current_path() / intrinsics_path;
    }
    intrinsics_path = std::filesystem::weakly_canonical(intrinsics_path);
    if (!std::filesystem::exists(intrinsics_path)) {
      throw std::runtime_error(
        "camera_intrinsics.yaml not found: " + intrinsics_path.string());
    }
    intrinsics_ = omx_perception_cpp::loadIntrinsics(intrinsics_path.string());

    target_frame_ = get_parameter("target_frame").as_string();
    camera_frame_ = get_parameter("camera_frame").as_string();
    cube_size_m_ = get_parameter("cube_size_m").as_double();
    box_output_z_m_ = get_parameter("box_output_z_m").as_double();
    cup_radius_m_ = get_parameter("cup_radius_m").as_double();
    cup_output_z_m_ = get_parameter("cup_output_z_m").as_double();
    min_keypoint_confidence_ = get_parameter("min_keypoint_confidence").as_double();
    keypoints_timeout_sec_ = get_parameter("keypoints_timeout_sec").as_double();

    const auto order = get_parameter("keypoint_order").as_integer_array();
    keypoint_order_.assign(order.begin(), order.end());
    std::vector<int> sorted_order = keypoint_order_;
    std::sort(sorted_order.begin(), sorted_order.end());
    if (sorted_order != std::vector<int>{0, 1, 2, 3}) {
      throw std::runtime_error(
        "keypoint_order must contain each index 0, 1, 2, 3 exactly once");
    }

    server_cb_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cb_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto keypoints_service_name =
      get_parameter("keypoints_service_name").as_string();
    const auto world_service_name =
      get_parameter("world_service_name").as_string();

    keypoints_client_ = create_client<GetKeypointDetections>(
      keypoints_service_name, rclcpp::ServicesQoS(), client_cb_group_);
    world_service_ = create_service<GetBlockPoses>(
      world_service_name,
      std::bind(
        &BoxCupWorldPoseNode::onGetWorldPoses, this,
        std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), server_cb_group_);

    RCLCPP_INFO(
      get_logger(),
      "box_cup world pose service ready "
      "(keypoints_service=%s, world_service=%s, target_frame=%s, "
      "box_output_z_m=%.3f, cup_output_z_m=%.3f)",
      keypoints_service_name.c_str(), world_service_name.c_str(),
      target_frame_.c_str(), box_output_z_m_, cup_output_z_m_);
  }

private:
  void onGetWorldPoses(
    const std::shared_ptr<GetBlockPoses::Request> request,
    std::shared_ptr<GetBlockPoses::Response> response)
  {
    (void)request;

    auto keypoints_response = callKeypointsService();
    if (!keypoints_response || !keypoints_response->success) {
      return;
    }

    const std::string source_frame = keypoints_response->header.frame_id.empty()
      ? camera_frame_ : keypoints_response->header.frame_id;
    const auto & stamp = keypoints_response->header.stamp;
    const bool use_latest = (stamp.sec == 0 && stamp.nanosec == 0);
    tf2::TimePoint lookup_time = tf2::TimePointZero;
    if (!use_latest) {
      lookup_time = tf2::TimePoint(
        std::chrono::seconds(stamp.sec) +
        std::chrono::nanoseconds(stamp.nanosec));
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        target_frame_, source_frame, lookup_time,
        std::chrono::milliseconds(500));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(), "failed to lookup transform %s <- %s: %s",
        target_frame_.c_str(), source_frame.c_str(), ex.what());
      return;
    }

    std::vector<BlockPose> blocks;
    for (const auto & det : keypoints_response->detections) {
      BlockPose block;
      if (detectionToBlockPose(det, keypoints_response->header, transform, block)) {
        blocks.push_back(block);
      }
    }
    response->blocks = blocks;
  }

  GetKeypointDetections::Response::SharedPtr callKeypointsService()
  {
    const auto timeout =
      std::chrono::duration<double>(keypoints_timeout_sec_);
    if (!keypoints_client_->wait_for_service(timeout)) {
      RCLCPP_WARN(get_logger(), "box_cup keypoint service is not available");
      return nullptr;
    }

    auto request = std::make_shared<GetKeypointDetections::Request>();
    request->publish_debug = true;
    auto future = keypoints_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "box_cup keypoint service call timed out");
      return nullptr;
    }
    return future.get();
  }

  bool detectionToBlockPose(
    const KeypointDetection & det,
    const std_msgs::msg::Header & header,
    const geometry_msgs::msg::TransformStamped & transform,
    BlockPose & block)
  {
    cv::Mat image_points(static_cast<int>(keypoint_order_.size()), 2, CV_64F);
    std::vector<double> confidences;
    for (std::size_t i = 0; i < keypoint_order_.size(); ++i) {
      const int base = keypoint_order_[i] * 3;
      const double confidence = det.keypoints[base + 2];
      if (confidence < min_keypoint_confidence_) {
        return false;
      }
      confidences.push_back(confidence);
      image_points.at<double>(static_cast<int>(i), 0) = det.keypoints[base];
      image_points.at<double>(static_cast<int>(i), 1) = det.keypoints[base + 1];
    }

    cv::Mat object_points;
    try {
      object_points = omx_perception_cpp::objectPointsForClass(
        det.class_id, cube_size_m_, cup_radius_m_);
    } catch (const std::invalid_argument & ex) {
      RCLCPP_WARN(get_logger(), "%s", ex.what());
      return false;
    }

    const auto pnp = omx_perception_cpp::solvePnpPlanarDisambiguated(
      object_points, image_points, intrinsics_.camera_matrix,
      intrinsics_.dist_coeffs);
    if (!pnp.ok) {
      return false;
    }

    double output_z;
    if (det.class_id == KeypointDetection::CLASS_BOX) {
      output_z = box_output_z_m_;
    } else if (det.class_id == KeypointDetection::CLASS_CUP) {
      output_z = cup_output_z_m_;
    } else {
      RCLCPP_WARN(
        get_logger(), "unsupported class_id %d; dropping detection",
        static_cast<int>(det.class_id));
      return false;
    }

    const cv::Vec3d center_world = transformPoint(pnp.tvec, transform);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = header.stamp;
    pose.header.frame_id = target_frame_;
    pose.pose.position.x = center_world[0];
    pose.pose.position.y = center_world[1];
    pose.pose.position.z = output_z;

    block.header = pose.header;
    if (det.class_id == KeypointDetection::CLASS_CUP) {
      block.color = "cup";
      pose.pose.orientation.w = 1.0;
      block.yaw_confidence = 0.0F;
    } else {
      block.color = det.color.empty() ? "unknown" : det.color;
      const auto & rotation = transform.transform.rotation;
      const cv::Matx33d rotation_world_cam =
        omx_perception_cpp::quaternionToRotationMatrix(
          rotation.x, rotation.y, rotation.z, rotation.w);
      const double yaw = omx_perception_cpp::wrapYawToPm45(
        omx_perception_cpp::boxYawWorld(pnp.rvec, rotation_world_cam));
      const auto quat = omx_perception_cpp::quaternionFromYaw(yaw);
      pose.pose.orientation.x = quat[0];
      pose.pose.orientation.y = quat[1];
      pose.pose.orientation.z = quat[2];
      pose.pose.orientation.w = quat[3];
      block.yaw_confidence = static_cast<float>(
        *std::min_element(confidences.begin(), confidences.end()));
    }

    block.pose = pose;
    block.grasp_pose = pose;
    block.confidence = det.detection_confidence;
    return true;
  }

  cv::Vec3d transformPoint(
    const cv::Vec3d & point,
    const geometry_msgs::msg::TransformStamped & transform) const
  {
    const auto & translation = transform.transform.translation;
    const auto & rotation = transform.transform.rotation;
    const cv::Matx33d matrix = omx_perception_cpp::quaternionToRotationMatrix(
      rotation.x, rotation.y, rotation.z, rotation.w);
    return matrix * point +
           cv::Vec3d(translation.x, translation.y, translation.z);
  }

  omx_perception_cpp::Intrinsics intrinsics_;
  std::string target_frame_;
  std::string camera_frame_;
  double cube_size_m_{};
  double box_output_z_m_{};
  double cup_radius_m_{};
  double cup_output_z_m_{};
  double min_keypoint_confidence_{};
  double keypoints_timeout_sec_{};
  std::vector<int> keypoint_order_;

  rclcpp::CallbackGroup::SharedPtr server_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<GetKeypointDetections>::SharedPtr keypoints_client_;
  rclcpp::Service<GetBlockPoses>::SharedPtr world_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<BoxCupWorldPoseNode>();
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(
      rclcpp::get_logger("box_cup_world_pose"),
      "node failed to start: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 2: CMakeLists.txt 에 노드 타깃 추가**

`src/omx_perception_cpp/CMakeLists.txt` 의 `ament_target_dependencies(pnp_geometry omx_interfaces)` 줄과 `if(BUILD_TESTING)` 줄 사이에 다음을 삽입한다:

```cmake
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(box_cup_world_pose_node src/box_cup_world_pose_node.cpp)
target_compile_features(box_cup_world_pose_node PUBLIC cxx_std_17)
target_link_libraries(box_cup_world_pose_node pnp_geometry ${OpenCV_LIBS})
ament_target_dependencies(box_cup_world_pose_node
  rclcpp omx_interfaces geometry_msgs std_msgs tf2 tf2_ros)

install(TARGETS box_cup_world_pose_node
  DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 3: 빌드 검증**

Run: `cd /home/kjhz/omx_ws && colcon build --symlink-install --packages-select omx_perception_cpp`
Expected: `Finished <<< omx_perception_cpp` — 노드 + 라이브러리 + 테스트 빌드 성공.

- [ ] **Step 4: executable 등록 확인**

Run: `cd /home/kjhz/omx_ws && source install/setup.bash && ros2 pkg executables omx_perception_cpp`
Expected: `omx_perception_cpp box_cup_world_pose_node`

- [ ] **Step 5: Commit**

```bash
cd /home/kjhz/omx_ws
git add src/omx_perception_cpp
git commit -m "feat(perception_cpp): add box_cup_world_pose_node C++ service node

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 7: Python world pose 노드/테스트 제거

**Files:**
- Delete: `src/omx_perception/omx_perception/box_cup_world_pose_node.py`
- Delete: `src/omx_perception/test/test_box_cup_world_pose_helpers.py`
- Modify: `src/omx_perception/setup.py`

- [ ] **Step 1: Python 파일 삭제**

```bash
cd /home/kjhz/omx_ws
git rm src/omx_perception/omx_perception/box_cup_world_pose_node.py
git rm src/omx_perception/test/test_box_cup_world_pose_helpers.py
```

- [ ] **Step 2: setup.py entry point 제거**

`src/omx_perception/setup.py` 의 `console_scripts` 리스트에서 다음 줄을 삭제한다:

```python
            "box_cup_world_pose_node = omx_perception.box_cup_world_pose_node:main",
```

삭제 후 `console_scripts` 는 다음 3개만 남는다:

```python
        "console_scripts": [
            "camera_control_node = omx_perception.camera_control_node:main",
            "box_cup_pose_node = omx_perception.box_cup_pose_node:main",
            "box_color_calibrate = omx_perception.box_color_calibrate:main",
        ],
```

- [ ] **Step 3: omx_perception 재빌드 + 잔존 테스트 확인**

Run: `cd /home/kjhz/omx_ws && colcon build --symlink-install --packages-select omx_perception && source install/setup.bash && colcon test --packages-select omx_perception --pytest-args -k test_pnp_geometry && colcon test-result --verbose`
Expected: 빌드 성공, `test_pnp_geometry.py` (Python, 잔존) 통과. `box_cup_world_pose_node` Python entry point 가 사라졌어도 빌드 정상.

- [ ] **Step 4: Commit**

```bash
cd /home/kjhz/omx_ws
git add src/omx_perception/setup.py
git commit -m "refactor(perception): remove Python box_cup_world_pose_node (ported to C++)

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 8: launch 변경

**Files:**
- Modify: `src/omx_perception/launch/perception.launch.py`

- [ ] **Step 1: world pose Node 의 package/executable 변경**

`src/omx_perception/launch/perception.launch.py` 의 `box_cup_world_pose` Node 정의에서 `package` 와 `executable` 두 줄을 교체한다.

기존:

```python
    box_cup_world_pose = Node(
        package="omx_perception",
        executable="box_cup_world_pose_node",
        name="box_cup_world_pose",
```

변경 후:

```python
    box_cup_world_pose = Node(
        package="omx_perception_cpp",
        executable="box_cup_world_pose_node",
        name="box_cup_world_pose",
```

`parameters` 블록은 그대로 둔다 (파라미터 이름/타입이 C++ 노드와 1:1 동일).

- [ ] **Step 2: launch 파일 문법 확인**

Run: `cd /home/kjhz/omx_ws && python3 -c "import ast; ast.parse(open('src/omx_perception/launch/perception.launch.py').read()); print('launch syntax OK')"`
Expected: `launch syntax OK`

- [ ] **Step 3: Commit**

```bash
cd /home/kjhz/omx_ws
git add src/omx_perception/launch/perception.launch.py
git commit -m "feat(perception): point world pose node at omx_perception_cpp

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

### Task 9: 전체 빌드 + 테스트 검증

**Files:** 없음 (검증만).

- [ ] **Step 1: 두 패키지 전체 빌드**

Run: `cd /home/kjhz/omx_ws && colcon build --symlink-install --packages-select omx_interfaces omx_perception omx_perception_cpp`
Expected: 세 패키지 모두 `Finished <<<`.

- [ ] **Step 2: omx_perception_cpp colcon test**

Run: `cd /home/kjhz/omx_ws && source install/setup.bash && colcon test --packages-select omx_perception_cpp && colcon test-result --verbose`
Expected: gtest `test_pnp_geometry` 전체 통과, lint(`ament_lint_auto`) 통과, 실패 0건.

- [ ] **Step 3: 노드 기동 스모크 테스트 (intrinsics 경로 직접 지정)**

Run:
```bash
cd /home/kjhz/omx_ws && source install/setup.bash && \
timeout 5 ros2 run omx_perception_cpp box_cup_world_pose_node --ros-args \
  -p camera_intrinsics_path:=src/omx_perception/config/camera_intrinsics.yaml ; \
echo "exit: $?"
```
Expected: `box_cup world pose service ready ...` 로그가 출력되고 5초 후 `timeout` 으로 종료(`exit: 124`). intrinsics 파일이 없으면 `node failed to start` 후 `exit: 1` — 이 경우 경로를 실제 파일로 맞춘다.

- [ ] **Step 4: 최종 커밋 (필요 시)**

Step 1-3 에서 소스 변경이 없었다면 커밋할 것은 없다. 검증 중 수정이 생겼다면:

```bash
cd /home/kjhz/omx_ws
git add -A
git commit -m "fix(perception_cpp): address build/test verification findings

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Self-Review

**Spec coverage:** spec 의 모든 항목이 task 로 매핑됨 — 새 ament_cmake 패키지(Task 1), pnp_geometry 전 함수(Task 2-5), 노드 + 동시성 설계(Task 6), Python 자산 제거(Task 7), launch 변경(Task 8), 빌드/테스트 검증(Task 9). `camera_frame_yaw` 미이식·`pnp_geometry.py` 유지도 spec 대로 반영(이식 대상에서 제외).

**Placeholder scan:** TBD/TODO/"적절히 처리" 류 없음. 모든 코드 단계에 전체 코드 수록.

**Type consistency:** `PnpResult{ok,rvec,tvec}`, `Intrinsics{camera_matrix,dist_coeffs}` 가 헤더 정의(Task 1)와 노드 사용처(Task 6)에서 일치. `objectPointsForClass`/`solvePnpPlanarDisambiguated`/`wrapYawToPm45`/`quaternionToRotationMatrix`/`quaternionFromYaw`/`boxYawWorld`/`loadIntrinsics` 시그니처가 헤더·구현·테스트·노드 전반에서 동일. `keypoints` 는 `float32[12]` 고정 배열, `class_id` 는 `uint8` 로 메시지 정의와 일치.

**주의:** ROS2 배포판에 따라 `tf2_ros/buffer.h` 가 `.hpp` 로 바뀌었을 수 있다. 빌드 시 헤더 not found 면 `.hpp` 변형으로 교체한다. `yaml-cpp` 타깃명이 `yaml-cpp::yaml-cpp` 인 환경이면 `target_link_libraries` 를 그에 맞춰 조정한다.
