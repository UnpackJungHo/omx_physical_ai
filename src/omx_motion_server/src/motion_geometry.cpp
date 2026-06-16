// omx_motion_server — motion_geometry.cpp
//
// motion_geometry.hpp 에 선언된 순수 기하 함수의 정의.
// 의존성은 <cmath> 뿐이며, ROS/MoveIt 헤더를 포함하지 않는다.

#include "omx_motion_server/motion_geometry.hpp"

#include <cmath>

namespace omx_motion_server
{

double jaw_axis_yaw_from_quaternion(double qx, double qy, double qz, double qw)
{
  const double axis_x = 2.0 * (qx * qy - qw * qz);
  const double axis_y = 1.0 - 2.0 * (qx * qx + qz * qz);
  return std::atan2(axis_y, axis_x);
}

double wrap_to_pm45(double angle_rad)
{
  constexpr double quarter = M_PI / 2.0;
  double wrapped = std::fmod(angle_rad, quarter);
  if (wrapped > M_PI / 4.0)
    wrapped -= quarter;
  else if (wrapped <= -M_PI / 4.0)
    wrapped += quarter;
  return wrapped;
}

bool position_in_workspace_box(
    double x, double y, double z,
    double x_abs_max, double y_abs_max, double z_min, double z_max)
{
  return std::abs(x) <= x_abs_max &&
         std::abs(y) <= y_abs_max &&
         z >= z_min &&
         z <= z_max;
}

}  // namespace omx_motion_server
