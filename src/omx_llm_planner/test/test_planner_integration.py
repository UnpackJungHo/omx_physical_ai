"""planner_node 통합 테스트 — fake skill action server + MockLLMClient.

GPU/실제 모델/실제 로봇 없이 시퀀싱/실패/취소/dry_run 경로를 검증한다.
"""
from __future__ import annotations

import threading
import time

import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from omx_interfaces.action import ExecuteCommand, MoveToNamed, PickPlace, PickPlaceAll
from omx_llm_planner.llm_client import MockLLMClient
from omx_llm_planner.planner_node import PlannerNode
from rclpy.node import Node


class FakeSkillServers(Node):
    """pick_place / pick_place_all / move_to_named 를 흉내내는 fake 서버.

    fail_colors 에 든 색은 success=False 를 반환한다.
    """

    def __init__(self, fail_colors=()):
        super().__init__("fake_skill_servers")
        self._fail_colors = set(fail_colors)
        self.calls: list[str] = []
        cb = ReentrantCallbackGroup()
        self._pp = ActionServer(
            self, PickPlace, "/omx/pick_place", self._pick_place, callback_group=cb)
        self._ppa = ActionServer(
            self, PickPlaceAll, "/omx/pick_place_all", self._pick_place_all, callback_group=cb)
        self._mtn = ActionServer(
            self, MoveToNamed, "/omx/move_to_named", self._move_to_named, callback_group=cb)

    def _pick_place(self, gh):
        color = gh.request.object_color
        self.calls.append(f"pick_place:{color}")
        ok = color not in self._fail_colors
        res = PickPlace.Result(success=ok, message="fake", attempts=1)
        gh.succeed() if ok else gh.abort()
        return res

    def _pick_place_all(self, gh):
        self.calls.append("pick_place_all")
        gh.succeed()
        return PickPlaceAll.Result(success=True, message="fake", placed_count=1,
                                   attempted_count=1, max_boxes=gh.request.max_boxes)

    def _move_to_named(self, gh):
        self.calls.append(f"move_to_named:{gh.request.name}")
        gh.succeed()
        return MoveToNamed.Result(success=True, message="fake")


@pytest.fixture
def ros_env():
    rclpy.init()
    yield
    rclpy.shutdown()


def _spin(executor):
    try:
        executor.spin()
    except Exception:
        pass


def _send_command(command: str, dry_run: bool = False, cancel_after_sec: float | None = None):
    """planner + fake server 를 띄우고 ExecuteCommand 를 보낸 뒤 result 를 반환."""
    responses = {
        "빨간거 집어": '{"steps": [{"action": "pick_place", "args": {"object_color": "red"}}]}',
        "빨간거랑 파란거 집어": (
            '{"steps": ['
            '{"action": "pick_place", "args": {"object_color": "red"}},'
            '{"action": "pick_place", "args": {"object_color": "blue"}}]}'
        ),
        "다 정리해": '{"steps": [{"action": "pick_place_all", "args": {"max_boxes": 5}}]}',
        "집에 가": '{"steps": [{"action": "move_to_named", "args": {"name": "home"}}]}',
    }
    fail_colors = ("red",) if command == "__fail_red__" else ()
    if command == "__fail_red__":
        command = "빨간거랑 파란거 집어"

    planner = PlannerNode(llm_client=MockLLMClient(responses))
    fakes = FakeSkillServers(fail_colors=fail_colors)
    executor = MultiThreadedExecutor()
    executor.add_node(planner)
    executor.add_node(fakes)
    spin_thread = threading.Thread(target=_spin, args=(executor,), daemon=True)
    spin_thread.start()

    client_node = Node("test_client")
    executor.add_node(client_node)
    ac = ActionClient(client_node, ExecuteCommand, "/omx/execute_command")
    assert ac.wait_for_server(timeout_sec=10.0)

    goal = ExecuteCommand.Goal(command=command, dry_run=dry_run)
    send_future = ac.send_goal_async(goal)
    while not send_future.done():
        time.sleep(0.02)
    gh = send_future.result()
    assert gh.accepted

    result_future = gh.get_result_async()
    if cancel_after_sec is not None:
        time.sleep(cancel_after_sec)
        gh.cancel_goal_async()

    deadline = time.time() + 15.0
    while not result_future.done() and time.time() < deadline:
        time.sleep(0.02)
    assert result_future.done(), "result timeout"
    result = result_future.result().result

    executor.shutdown()
    planner.destroy_node()
    fakes.destroy_node()
    client_node.destroy_node()
    return result, fakes.calls


def test_single_step_success(ros_env):
    result, calls = _send_command("빨간거 집어")
    assert result.success is True
    assert result.steps_total == 1 and result.steps_completed == 1
    assert calls == ["pick_place:red"]


def test_multi_step_order(ros_env):
    result, calls = _send_command("빨간거랑 파란거 집어")
    assert result.success is True
    assert result.steps_completed == 2
    assert calls == ["pick_place:red", "pick_place:blue"]


def test_pick_place_all_clamped(ros_env):
    result, calls = _send_command("다 정리해")
    assert result.success is True
    assert calls == ["pick_place_all"]


def test_step_failure_stops_sequence(ros_env):
    # red 가 실패하도록 -> 두번째 step(blue) 은 실행되면 안 됨
    result, calls = _send_command("__fail_red__")
    assert result.success is False
    assert "step 1/2 실패" in result.message
    assert calls == ["pick_place:red"]


def test_dry_run_does_not_execute(ros_env):
    result, calls = _send_command("빨간거랑 파란거 집어", dry_run=True)
    assert result.success is True
    assert "dry_run" in result.message
    assert result.steps_total == 2
    assert calls == []
