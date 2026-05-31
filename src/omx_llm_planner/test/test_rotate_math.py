import math
import pytest
from omx_llm_planner.rotate_math import resolve_rotate_target, RotateConfig


CFG = RotateConfig(sign={"left": 1.0, "right": -1.0},
                   joint_lower=-2.8, joint_upper=2.8)


def test_left_adds_positive_delta():
    t = resolve_rotate_target(current=0.0, direction="left", angle_deg=90, cfg=CFG)
    assert t == pytest.approx(math.radians(90))


def test_right_subtracts():
    t = resolve_rotate_target(current=0.0, direction="right", angle_deg=90, cfg=CFG)
    assert t == pytest.approx(-math.radians(90))


def test_clamped_to_upper_limit():
    t = resolve_rotate_target(current=2.7, direction="left", angle_deg=90, cfg=CFG)
    assert t == pytest.approx(2.8)


def test_unknown_direction_raises():
    with pytest.raises(KeyError):
        resolve_rotate_target(current=0.0, direction="up", angle_deg=10, cfg=CFG)
