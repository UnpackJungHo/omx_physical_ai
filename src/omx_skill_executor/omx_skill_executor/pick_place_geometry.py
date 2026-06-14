"""PickPlace 스킬의 순수 기하 helper.

ROS 의존이 없는 함수만 모아 단위 테스트가 쉽도록 분리한다.
"""
from __future__ import annotations

import math


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """단위 quaternion (x, y, z, w) 에서 world z축 기준 yaw 를 추출한다.

    입력이 정규화된 단위 quaternion 임을 전제한다.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# NOTE: jaw heading / 90deg wrap / joint5 목표 계산은 motion_server(C++) 의
# omx/compute_align_yaw service 로 단일화했다. 과거 여기 있던 중복 기하
# (jaw_axis_yaw_from_quaternion, wrap_yaw_zero_pi_over_2, wrap_to_pm45,
# joint5_target) 는 제거했다. 이 모듈은 ROS 의존 없는 cup polygon 판정과
# box yaw 추출만 남긴다.


def point_in_polygon_xy(
    px: float,
    py: float,
    polygon_xy: list[tuple[float, float]],
) -> bool:
    """XY 평면에서 점 (px, py) 가 polygon 내부인지 판정 (ray casting).

    polygon_xy 는 시계/반시계 어느 방향이든 무방하다. 경계 위 점은
    구현 안정성에 의존하지만 '안' 으로 간주한다. 3점 미만이면 항상 False.
    """
    n = len(polygon_xy)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon_xy[i]
        xj, yj = polygon_xy[j]
        # ray = +X 방향 무한선과 edge (i,j) 의 교차 여부.
        # 경계와의 교차 처리는 horizontal edge 를 skip 하기 위해
        # (yi > py) != (yj > py) 사용.
        if ((yi > py) != (yj > py)):
            denom = (yj - yi)
            if abs(denom) < 1e-12:
                j = i
                continue
            x_intersect = (xj - xi) * (py - yi) / denom + xi
            if px < x_intersect:
                inside = not inside
        j = i
    return inside


def is_box_in_cup(
    box_xy: tuple[float, float],
    cup_polygon_xy: list[tuple[float, float]],
) -> bool:
    """박스 중심 XY 가 cup polygon (4 corners, world XY) 내부에 있는지.

    cup_polygon_xy 가 비어 있거나 3점 미만이면 False (보수적으로 '바깥' 으로
    판정해 박스를 잡으러 가도록 둔다). cup 4 점 순서는 perception 의
    keypoint_order 를 그대로 따르며, 사각형이면 ray casting 으로 충분하다.
    """
    return point_in_polygon_xy(box_xy[0], box_xy[1], cup_polygon_xy)
