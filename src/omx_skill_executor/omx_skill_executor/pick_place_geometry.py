"""PickPlace 스킬의 순수 기하 helper.

ROS 의존이 없는 함수만 모아 단위 테스트가 쉽도록 분리한다.
"""
from __future__ import annotations

import math


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """quaternion 에서 world z축 기준 yaw 를 추출한다."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pm45(angle_rad: float) -> float:
    """각도를 box 90° 대칭을 이용해 (-pi/4, pi/4] 범위로 접는다."""
    quarter = math.pi / 2.0
    wrapped = math.fmod(angle_rad, quarter)
    if wrapped > math.pi / 4.0:
        wrapped -= quarter
    elif wrapped <= -math.pi / 4.0:
        wrapped += quarter
    return wrapped


def joint5_target(
    joint5_current: float,
    box_yaw: float,
    gripper_yaw: float,
    yaw_sign: float,
) -> float:
    """box 면과 그리퍼를 평행하게 맞추는 joint5 목표각(rad).

    joint5 는 수직 approach 축 기준 roll 이라 world yaw 변화량과 1:1.
    yaw_sign 은 자세에 따라 고정된 부호(+1/-1)로, 1회 캘리브레이션한다.
    """
    delta_world = wrap_to_pm45(box_yaw - gripper_yaw)
    return joint5_current + yaw_sign * delta_world
