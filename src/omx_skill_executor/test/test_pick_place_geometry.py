"""Pure-function unit tests for pick_place_geometry helpers."""
from __future__ import annotations

import math

from omx_skill_executor.pick_place_geometry import (
    is_box_in_cup,
    jaw_axis_yaw_from_quaternion,
    joint5_target,
    point_in_polygon_xy,
    wrap_to_pm45,
    wrap_yaw_zero_pi_over_2,
    yaw_from_quaternion,
)


def _angles_close(a: float, b: float, abs_tol: float = 1e-6) -> bool:
    """Compare two angles modulo 2*pi."""
    diff = math.atan2(math.sin(a - b), math.cos(a - b))
    return abs(diff) <= abs_tol


def _quat_mul(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Hamilton product of two (x, y, z, w) quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def test_yaw_from_quaternion_identity():
    assert math.isclose(yaw_from_quaternion(0.0, 0.0, 0.0, 1.0), 0.0, abs_tol=1e-9)


def test_yaw_from_quaternion_z_90deg():
    qz = math.sin(math.pi / 4)
    qw = math.cos(math.pi / 4)
    assert math.isclose(
        yaw_from_quaternion(0.0, 0.0, qz, qw), math.radians(90.0), abs_tol=1e-6
    )


def test_wrap_to_pm45_within_range_unchanged():
    assert math.isclose(wrap_to_pm45(math.radians(20.0)), math.radians(20.0), abs_tol=1e-9)


def test_wrap_to_pm45_folds_60deg_to_minus30():
    # 60° 는 90° 대칭이므로 -30° 와 같은 정렬.
    assert math.isclose(wrap_to_pm45(math.radians(60.0)), math.radians(-30.0), abs_tol=1e-6)


def test_wrap_to_pm45_folds_minus80_to_10():
    assert math.isclose(wrap_to_pm45(math.radians(-80.0)), math.radians(10.0), abs_tol=1e-6)


def test_wrap_to_pm45_folds_full_circle():
    assert math.isclose(wrap_to_pm45(math.radians(360.0)), 0.0, abs_tol=1e-6)


def test_wrap_yaw_zero_pi_over_2_within_range_unchanged():
    for deg in (0.0, 15.0, 45.0, 89.9):
        assert math.isclose(
            wrap_yaw_zero_pi_over_2(math.radians(deg)),
            math.radians(deg),
            abs_tol=1e-9,
        )


def test_wrap_yaw_zero_pi_over_2_folds_90_to_0():
    # 90° 회전 대칭이므로 90° 는 0° 와 같은 정렬.
    assert math.isclose(
        wrap_yaw_zero_pi_over_2(math.radians(90.0)), 0.0, abs_tol=1e-6
    )


def test_wrap_yaw_zero_pi_over_2_folds_negative_into_range():
    # -30° 는 60° 와 같은 정렬 (mod 90°).
    assert math.isclose(
        wrap_yaw_zero_pi_over_2(math.radians(-30.0)),
        math.radians(60.0),
        abs_tol=1e-6,
    )


def test_wrap_yaw_zero_pi_over_2_folds_200_to_20():
    assert math.isclose(
        wrap_yaw_zero_pi_over_2(math.radians(200.0)),
        math.radians(20.0),
        abs_tol=1e-6,
    )


def test_joint5_target_positive_sign():
    # box_yaw 30°, gripper_yaw 0°, joint5 현재 0° → delta +30°.
    result = joint5_target(0.0, math.radians(30.0), 0.0, 1.0)
    assert math.isclose(result, math.radians(30.0), abs_tol=1e-6)


def test_joint5_target_negative_sign_flips_delta():
    result = joint5_target(0.0, math.radians(30.0), 0.0, -1.0)
    assert math.isclose(result, math.radians(-30.0), abs_tol=1e-6)


def test_joint5_target_uses_90deg_symmetry():
    # box_yaw 100° 는 10° 와 동일 정렬 → delta +10°.
    result = joint5_target(math.radians(5.0), math.radians(100.0), 0.0, 1.0)
    assert math.isclose(result, math.radians(15.0), abs_tol=1e-6)


def test_jaw_axis_yaw_identity_points_along_world_y():
    # end_effector_link +y 축이 world +y 와 일치 → heading 90°.
    assert math.isclose(
        jaw_axis_yaw_from_quaternion(0.0, 0.0, 0.0, 1.0),
        math.radians(90.0),
        abs_tol=1e-9,
    )


def test_jaw_axis_yaw_tracks_z_rotation():
    # z축으로 theta 회전하면 jaw 축 heading 도 theta 만큼 증가한다.
    for deg in (-90.0, 30.0, 120.0):
        theta = math.radians(deg)
        qz, qw = math.sin(theta / 2.0), math.cos(theta / 2.0)
        assert _angles_close(
            jaw_axis_yaw_from_quaternion(0.0, 0.0, qz, qw),
            math.radians(90.0) + theta,
        )


def test_jaw_axis_yaw_well_defined_when_gripper_points_down():
    # end_effector +x 축이 world -z 를 향하는(아래로 향한 그리퍼) Ry(90°):
    # ZYX yaw 추출은 gimbal lock 이지만 jaw(y)축은 수평이라 명확하다.
    s = math.sqrt(0.5)
    q_down = (0.0, s, 0.0, s)  # Ry(90°)
    assert math.isclose(
        jaw_axis_yaw_from_quaternion(*q_down), math.radians(90.0), abs_tol=1e-6
    )

    # 아래를 향한 채 approach 축(=world -z) 둘레로 phi 만큼 roll → heading = 90° - phi.
    for deg in (-60.0, 25.0, 100.0):
        phi = math.radians(deg)
        q_roll = (0.0, 0.0, math.sin(-phi / 2.0), math.cos(-phi / 2.0))  # Rz(-phi)
        q_total = _quat_mul(q_roll, q_down)
        assert math.isclose(
            jaw_axis_yaw_from_quaternion(*q_total),
            math.radians(90.0) - phi,
            abs_tol=1e-6,
        )


# ---------- point_in_polygon_xy / is_box_in_cup ----------

# 0.1 m × 0.1 m 정사각형 cup, 중심 (0.3, 0.0). 모서리 시계 순서 무관.
_CUP_SQUARE = [
    (0.25, -0.05),
    (0.35, -0.05),
    (0.35, 0.05),
    (0.25, 0.05),
]


def test_point_in_polygon_center_inside():
    assert point_in_polygon_xy(0.30, 0.00, _CUP_SQUARE) is True


def test_point_in_polygon_outside_x():
    assert point_in_polygon_xy(0.50, 0.00, _CUP_SQUARE) is False


def test_point_in_polygon_outside_y():
    assert point_in_polygon_xy(0.30, 0.10, _CUP_SQUARE) is False


def test_point_in_polygon_returns_false_for_degenerate():
    assert point_in_polygon_xy(0.30, 0.00, []) is False
    assert point_in_polygon_xy(0.30, 0.00, [(0.0, 0.0), (1.0, 1.0)]) is False


def test_is_box_in_cup_inside():
    assert is_box_in_cup((0.30, 0.00), _CUP_SQUARE) is True


def test_is_box_in_cup_outside():
    assert is_box_in_cup((0.40, 0.00), _CUP_SQUARE) is False


def test_is_box_in_cup_empty_polygon_returns_false():
    # cup polygon 미가용 시 보수적으로 '바깥' 판정. (잡으러 가도록)
    assert is_box_in_cup((0.30, 0.00), []) is False


def test_point_in_polygon_rotated_square():
    # 45° 회전한 사각형(다이아몬드): 꼭짓점 4 개.
    diamond = [(0.30, -0.07), (0.37, 0.00), (0.30, 0.07), (0.23, 0.00)]
    assert point_in_polygon_xy(0.30, 0.00, diamond) is True
    assert point_in_polygon_xy(0.36, 0.05, diamond) is False
