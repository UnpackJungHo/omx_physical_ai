"""MockLLMClient unit tests (network 무의존)."""
from __future__ import annotations

import pytest

from omx_llm_planner.llm_client import LLMUnavailable, MockLLMClient
from omx_llm_planner.plan_schema import PlanError


def test_mock_returns_parsed_plan():
    client = MockLLMClient({
        "빨간 박스 컵에 넣어": '{"steps": [{"action": "pick_place", "args": {"object_color": "red"}}]}',
    })
    plan = client.generate_plan("빨간 박스 컵에 넣어")
    assert plan.steps[0].args == {"object_color": "red"}


def test_mock_unknown_command_raises_unavailable():
    client = MockLLMClient({})
    with pytest.raises(LLMUnavailable):
        client.generate_plan("처음 보는 명령")


def test_mock_propagates_plan_error_for_bad_json():
    client = MockLLMClient({"x": "not json"})
    with pytest.raises(PlanError):
        client.generate_plan("x")
