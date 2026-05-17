"""Pure-function unit tests for pick_place_geometry helpers."""
from __future__ import annotations

import math

from omx_skill_executor.pick_place_geometry import (
    joint5_target,
    wrap_to_pm45,
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


def test_wrap_to_pm45_within_range_unchanged():
    assert math.isclose(wrap_to_pm45(math.radians(20.0)), math.radians(20.0), abs_tol=1e-9)


def test_wrap_to_pm45_folds_60deg_to_minus30():
    # 60° 는 90° 대칭이므로 -30° 와 같은 정렬.
    assert math.isclose(wrap_to_pm45(math.radians(60.0)), math.radians(-30.0), abs_tol=1e-6)


def test_wrap_to_pm45_folds_minus80_to_10():
    assert math.isclose(wrap_to_pm45(math.radians(-80.0)), math.radians(10.0), abs_tol=1e-6)


def test_wrap_to_pm45_folds_full_circle():
    assert math.isclose(wrap_to_pm45(math.radians(360.0)), 0.0, abs_tol=1e-6)


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
