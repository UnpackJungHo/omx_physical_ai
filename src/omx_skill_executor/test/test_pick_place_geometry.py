"""Pure-function unit tests for pick_place_geometry helpers.

NOTE: jaw heading / 90deg wrap / joint5 목표 계산은 motion_server(C++) 의
omx/compute_align_yaw service 로 단일화했다. 과거 여기서 검증하던
jaw_axis_yaw_from_quaternion / wrap_to_pm45 / wrap_yaw_zero_pi_over_2 /
joint5_target 테스트는 C++ 쪽으로 이전 대상이며 본 파일에서는 제거했다.
이 모듈에는 box yaw 추출과 cup polygon 판정만 남는다.
"""
from __future__ import annotations

import math

from omx_skill_executor.pick_place_geometry import (
    is_box_in_cup,
    point_in_polygon_xy,
    yaw_from_quaternion,
)


def test_yaw_from_quaternion_identity():
    assert math.isclose(yaw_from_quaternion(0.0, 0.0, 0.0, 1.0), 0.0, abs_tol=1e-9)


def test_yaw_from_quaternion_z_90deg():
    qz = math.sin(math.pi / 4)
    qw = math.cos(math.pi / 4)
    assert math.isclose(
        yaw_from_quaternion(0.0, 0.0, qz, qw), math.radians(90.0), abs_tol=1e-6
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
