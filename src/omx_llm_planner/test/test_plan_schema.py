"""Pure unit tests for plan_schema — LLM plan 검증/정규화."""
from __future__ import annotations

import pytest

from omx_llm_planner.plan_schema import (
    NAMED_POSES,
    PICK_COLORS,
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
    assert NAMED_POSES == ("home", "ready", "pre_grasp", "stow")
