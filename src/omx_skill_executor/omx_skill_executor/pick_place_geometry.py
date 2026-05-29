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


def jaw_axis_yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """그리퍼 jaw 폐합축(end_effector_link +y축)의 world heading(rad).

    end_effector_link 의 +y 축을 world 로 회전시킨 뒤 xy 평면 heading 을
    반환한다. 두 그리퍼 손가락은 link5 의 y축 방향으로 벌어져 있어, 이
    축이 jaw 가 박스를 누르는 방향이다. 수직 approach 축(end_effector +x)
    에 직교해 항상 수평이므로, ZYX yaw 추출과 달리 그리퍼가 아래를 향할
    때도 gimbal lock 이 없다.

    입력이 정규화된 단위 quaternion 임을 전제한다.
    """
    # R @ [0, 1, 0] = 회전행렬의 2번째 열.
    axis_x = 2.0 * (x * y - w * z)
    axis_y = 1.0 - 2.0 * (x * x + z * z)
    return math.atan2(axis_y, axis_x)


def wrap_yaw_zero_pi_over_2(angle_rad: float) -> float:
    """각도를 box 90° 회전 대칭을 이용해 [0, pi/2) 범위로 접는다.

    perception (omx_perception_world_pose) 가 box yaw 를 동일한 구간으로
    wrap 해서 publish 하므로, skill 측에서 gripper jaw heading 을 같은
    구간으로 정규화해 두면 두 yaw 가 같은 표현이 되어 delta 계산이
    일관된다.
    """
    period = math.pi / 2.0
    wrapped = math.fmod(angle_rad, period)
    if wrapped < 0.0:
        wrapped += period
    return wrapped


def wrap_to_pm45(angle_rad: float) -> float:
    """각도를 box 90° 대칭을 이용해 (-pi/4, pi/4] 범위로 접는다.

    -pi/4 부근에서 출력이 +pi/4 로 점프하는 불연속이 존재한다(90° 대칭상
    -45° 와 +45° 는 같은 정렬). box_yaw-gripper_yaw 가 이 경계 근방에서
    노이즈를 타면 호출부에서 joint 명령이 튈 수 있으므로, 필요 시 상위
    레이어에서 hysteresis 를 고려한다.
    """
    quarter = math.pi / 2.0
    wrapped = math.fmod(angle_rad, quarter)
    if wrapped > math.pi / 4.0:
        wrapped -= quarter
    elif wrapped <= -math.pi / 4.0:
        wrapped += quarter
    return wrapped


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


def joint5_target(
    joint5_current: float,
    box_yaw: float,
    gripper_yaw: float,
    yaw_sign: float,
) -> float:
    """box 면과 그리퍼를 평행하게 맞추는 joint5 목표각(rad).

    perception 은 box_yaw 를 [0, pi/2) 로 wrap 해서 보낸다. gripper_yaw 도
    동일 구간으로 정규화한 뒤 차이를 (-pi/4, pi/4] 로 접어 최단 회전을
    구한다. joint5 는 수직 approach 축 기준 roll 이라 world yaw 변화량과
    1:1 이고, yaw_sign 은 자세에 따라 고정된 부호(+1/-1)로 1회 캘리브레이션한다.
    """
    box_yaw_norm = wrap_yaw_zero_pi_over_2(box_yaw)
    gripper_yaw_norm = wrap_yaw_zero_pi_over_2(gripper_yaw)
    delta_world = wrap_to_pm45(box_yaw_norm - gripper_yaw_norm)
    return joint5_current + yaw_sign * delta_world
