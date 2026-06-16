// omx_motion_server — motion_geometry.hpp
//
// yaw 정렬 기하의 순수 함수 단일 소스.
//   - 외부(ROS/MoveIt/TF) 의존이 없는 결정적 계산만 모은다.
//   - motion_server 의 compute_align_yaw service 가 이 함수들을 사용하고,
//     로봇 없이도 ament_add_gtest 로 단위 검증할 수 있다.
//
// 과거 skill_executor(pick_place_geometry.py) 에 중복 구현돼 있던 jaw heading /
// 90deg wrap 계산을 motion_server 로 단일화했으며, 그 핵심 수식을 다시 이 헤더로
// 분리해 테스트 가능하게 만들었다. Python worker 는 omx/compute_align_yaw service
// 로 위임한다.

#ifndef OMX_MOTION_SERVER__MOTION_GEOMETRY_HPP_
#define OMX_MOTION_SERVER__MOTION_GEOMETRY_HPP_

namespace omx_motion_server
{

// end_effector_link +y 축의 world XY heading (rad). gimbal-lock 없음.
// quaternion (qx, qy, qz, qw) 은 정규화돼 있다고 가정한다.
double jaw_axis_yaw_from_quaternion(double qx, double qy, double qz, double qw);

// 박스 90° 대칭을 이용해 각도를 (-π/4, π/4] 범위로 접는다.
double wrap_to_pm45(double angle_rad);

// EE 목표 위치가 workspace 박스 안에 있는지 검사한다(world/planning frame 기준).
// |x|<=x_abs_max, |y|<=y_abs_max, z_min<=z<=z_max 이면 true.
// MoveToPose 의 goal 사전 거부와 execute 직전 검사가 이 한 함수를 공유한다.
bool position_in_workspace_box(
    double x, double y, double z,
    double x_abs_max, double y_abs_max, double z_min, double z_max);

}  // namespace omx_motion_server

#endif  // OMX_MOTION_SERVER__MOTION_GEOMETRY_HPP_
