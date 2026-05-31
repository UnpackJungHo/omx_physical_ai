"""gripper/rotate_base goal 빌드 + 변환 단위 테스트.

rclpy Node 를 띄우지 않고 SkillDispatcher 를 최소 구성으로 생성한다
(joint cache·action client 는 가짜 객체 주입). goal 빌드/변환 로직만 검증한다.
"""
import math
import threading
import pytest

from omx_llm_planner.rotate_math import RotateConfig
from omx_llm_planner.skill_clients import DispatcherConfig, SkillDispatcher


class _FakeCache:
    def __init__(self, value):
        self._value = value

    def get(self, joint):
        return self._value


class _FakeClient:
    def wait_for_server(self, timeout_sec=None):
        return True


def _make_config():
    return DispatcherConfig(
        pick_place_action="pp",
        pick_place_all_action="ppa",
        move_to_named_action="mtn",
        server_wait_timeout_sec=1.0,
        goal_response_timeout_sec=1.0,
        result_timeout_sec=1.0,
        gripper_action="grip",
        move_to_joints_action="mtj",
        rotate_joint_name="joint1",
        rotate_velocity_scale=0.3,
        rotate=RotateConfig(sign={"left": 1.0, "right": -1.0},
                            joint_lower=-2.8, joint_upper=2.8),
        gripper_open_position=1.0,
        gripper_close_position=0.0,
    )


def _make_dispatcher(joint_value):
    d = object.__new__(SkillDispatcher)
    d._config = _make_config()
    d._joint_cache = _FakeCache(joint_value)
    d._clients = {"gripper": _FakeClient(), "rotate_base": _FakeClient()}
    return d


@pytest.fixture
def dispatcher():
    return _make_dispatcher(0.0)


@pytest.fixture
def dispatcher_with_joint():
    def _factory(current):
        return _make_dispatcher(current)
    return _factory


def test_build_gripper_goal_open(dispatcher):
    goal = dispatcher._build_goal("gripper", {"state": "open"})
    assert goal.position == pytest.approx(1.0)


def test_build_gripper_goal_close(dispatcher):
    goal = dispatcher._build_goal("gripper", {"state": "close"})
    assert goal.position == pytest.approx(0.0)


def test_rotate_uses_current_joint_and_sign(dispatcher_with_joint):
    d = dispatcher_with_joint(current=0.0)
    goal = d._build_rotate_goal({"direction": "left", "angle_deg": 90})
    assert goal.joint_names == ["joint1"]
    assert goal.positions[0] == pytest.approx(math.radians(90))


def test_rotate_fails_when_joint_state_missing(dispatcher_with_joint):
    d = dispatcher_with_joint(current=None)   # stale/없음
    res = d.execute_step("rotate_base", {"direction": "left", "angle_deg": 10}, threading.Event())
    assert res.success is False and "joint1" in res.message
