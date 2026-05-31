"""rotate_base 상대 회전 -> 절대 joint1 목표(radian) 변환 (순수, ROS 무의존).

부호(left/right -> ±)는 config(SIGN)로 주입해 하드웨어 검증 결과를 코드 수정 없이
반영한다. 결과는 joint limit 으로 clamp 한다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class RotateConfig:
    sign: dict           # {"left": +1.0, "right": -1.0} (하드웨어 검증값)
    joint_lower: float   # radian
    joint_upper: float


def resolve_rotate_target(current: float, direction: str, angle_deg: int,
                          cfg: RotateConfig) -> float:
    delta = cfg.sign[direction] * math.radians(angle_deg)
    target = current + delta
    return max(cfg.joint_lower, min(target, cfg.joint_upper))
