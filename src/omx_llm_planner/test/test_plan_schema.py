"""Pure unit tests for plan_schema — LLM plan 검증/정규화."""
from __future__ import annotations

import pytest

from omx_llm_planner.plan_schema import (
    ANGLE_DEG_HI,
    GRIPPER_STATES,
    NAMED_POSES,
    PICK_COLORS,
    ROTATE_DIRS,
    Plan,
    PlanError,
    PlanStep,
    build_plan,
)


def test_single_pick_place():
    plan = build_plan('{"steps": [{"action": "pick_place", "args": {"object_color": "red"}}]}')
    assert plan == Plan(steps=[PlanStep("pick_place", {"object_color": "red"})])


def test_multi_step_order_preserved():
    raw = (
        '{"steps": ['
        '{"action": "pick_place", "args": {"object_color": "red"}},'
        '{"action": "pick_place", "args": {"object_color": "blue"}}]}'
    )
    plan = build_plan(raw)
    assert [s.args["object_color"] for s in plan.steps] == ["red", "blue"]


def test_accepts_dict_input():
    plan = build_plan({"steps": [{"action": "move_to_named", "args": {"name": "home"}}]})
    assert plan.steps[0] == PlanStep("move_to_named", {"name": "home"})


def test_pick_place_all_clamps_and_defaults():
    plan = build_plan('{"steps": [{"action": "pick_place_all", "args": {"max_boxes": 99}}]}')
    assert plan.steps[0].args == {"max_boxes": 10, "retry_on_fail": False}


def test_pick_place_all_nonpositive_uses_default():
    plan = build_plan('{"steps": [{"action": "pick_place_all", "args": {"max_boxes": 0}}]}')
    assert plan.steps[0].args["max_boxes"] == 10


def test_pick_place_all_missing_args_defaults():
    plan = build_plan('{"steps": [{"action": "pick_place_all"}]}')
    assert plan.steps[0].args == {"max_boxes": 10, "retry_on_fail": False}


def test_empty_steps_raises():
    with pytest.raises(PlanError):
        build_plan('{"steps": []}')


def test_unknown_action_raises():
    with pytest.raises(PlanError):
        build_plan('{"steps": [{"action": "unknown"}]}')


def test_invalid_color_raises():
    with pytest.raises(PlanError):
        build_plan('{"steps": [{"action": "pick_place", "args": {"object_color": "purple"}}]}')


def test_invalid_named_pose_raises():
    with pytest.raises(PlanError):
        build_plan('{"steps": [{"action": "move_to_named", "args": {"name": "kitchen"}}]}')


def test_malformed_json_raises():
    with pytest.raises(PlanError):
        build_plan("not json at all")


def test_missing_steps_key_raises():
    with pytest.raises(PlanError):
        build_plan('{"plan": []}')


def test_enums_are_exposed():
    assert PICK_COLORS == ("red", "blue", "green")
    assert NAMED_POSES == ("home", "init")


def test_gripper_open_close_ok():
    p = build_plan({"steps": [{"action": "gripper", "args": {"state": "open"}}]})
    assert p.steps[0] == PlanStep("gripper", {"state": "open"})


def test_gripper_invalid_state_raises():
    with pytest.raises(PlanError):
        build_plan({"steps": [{"action": "gripper", "args": {"state": "half"}}]})


def test_rotate_base_ok_and_enums():
    p = build_plan({"steps": [{"action": "rotate_base",
                               "args": {"direction": "left", "angle_deg": 10}}]})
    assert p.steps[0] == PlanStep("rotate_base", {"direction": "left", "angle_deg": 10})
    assert GRIPPER_STATES == ("open", "close")
    assert ROTATE_DIRS == ("left", "right")


def test_rotate_base_clamps_angle():
    p = build_plan({"steps": [{"action": "rotate_base",
                               "args": {"direction": "right", "angle_deg": 999}}]})
    assert p.steps[0].args["angle_deg"] == ANGLE_DEG_HI


def test_rotate_base_invalid_direction_raises():
    with pytest.raises(PlanError):
        build_plan({"steps": [{"action": "rotate_base",
                               "args": {"direction": "up", "angle_deg": 10}}]})
