"""PlanStep 1개를 해당 ROS2 action 으로 실행하는 dispatcher.

각 action(/omx/pick_place, /omx/pick_place_all, /omx/move_to_named,
/omx/gripper_command, /omx/move_to_joints) 에 대해
ActionClient 를 만들고, goal 전송 -> result 대기를 timeout 과 cancel 이벤트로
제어한다. blocking busy-wait 없이 future + Event 로 대기하며, 모든 timeout 은
주입된 config 로만 결정한다 (하드코딩 금지).

전제: 노드가 MultiThreadedExecutor + ReentrantCallbackGroup 으로 spin 되어야
future 콜백이 execute_callback 스레드와 병행 처리된다.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass

from rclpy.action import ActionClient
from rclpy.node import Node

from omx_interfaces.action import (
    GripperCommand,
    MoveToJoints,
    MoveToNamed,
    PickPlace,
    PickPlaceAll,
)
from omx_llm_planner.joint_state_cache import JointStateCache
from omx_llm_planner.rotate_math import RotateConfig, resolve_rotate_target


@dataclass
class StepResult:
    success: bool
    message: str


@dataclass
class DispatcherConfig:
    pick_place_action: str
    pick_place_all_action: str
    move_to_named_action: str
    server_wait_timeout_sec: float
    goal_response_timeout_sec: float
    result_timeout_sec: float
    gripper_action: str
    move_to_joints_action: str
    rotate_joint_name: str
    rotate_velocity_scale: float
    rotate: RotateConfig
    gripper_open_position: float
    gripper_close_position: float
    gripper_max_effort: float = 0.0


class SkillDispatcher:
    def __init__(
        self,
        node: Node,
        cb_group,
        config: DispatcherConfig,
        joint_cache: JointStateCache | None = None,
    ) -> None:
        self._node = node
        self._config = config
        self._joint_cache = joint_cache
        self._clients = {
            "pick_place": ActionClient(
                node, PickPlace, config.pick_place_action, callback_group=cb_group),
            "pick_place_all": ActionClient(
                node, PickPlaceAll, config.pick_place_all_action, callback_group=cb_group),
            "move_to_named": ActionClient(
                node, MoveToNamed, config.move_to_named_action, callback_group=cb_group),
            "gripper": ActionClient(
                node, GripperCommand, config.gripper_action, callback_group=cb_group),
            "rotate_base": ActionClient(
                node, MoveToJoints, config.move_to_joints_action, callback_group=cb_group),
        }

    def execute_step(self, action: str, args: dict, cancel_event: threading.Event) -> StepResult:
        client = self._clients.get(action)
        if client is None:
            return StepResult(False, f"미지원 action: {action}")
        try:
            goal = self._build_goal(action, args)
        except (KeyError, ValueError) as exc:
            return StepResult(False, str(exc))

        if not client.wait_for_server(timeout_sec=self._config.server_wait_timeout_sec):
            return StepResult(False, f"action server '{action}' 미연결")

        send_future = client.send_goal_async(goal)
        if not self._wait_future(send_future, self._config.goal_response_timeout_sec):
            return StepResult(False, f"'{action}' goal 응답 timeout")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return StepResult(False, f"'{action}' goal 거부됨")

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, self._config.result_timeout_sec, cancel_event):
            if cancel_event.is_set():
                goal_handle.cancel_goal_async()
                return StepResult(False, f"'{action}' 취소됨")
            return StepResult(False, f"'{action}' result timeout")

        wrapped = result_future.result()
        result = wrapped.result
        success = bool(getattr(result, "success", False))
        message = getattr(result, "message", "")
        return StepResult(success, message)

    def _build_goal(self, action: str, args: dict):
        if action == "pick_place":
            return PickPlace.Goal(object_color=args["object_color"], retry_on_fail=False)
        if action == "pick_place_all":
            return PickPlaceAll.Goal(
                max_boxes=args["max_boxes"], retry_on_fail=args["retry_on_fail"])
        if action == "move_to_named":
            return MoveToNamed.Goal(name=args["name"])
        if action == "gripper":
            position = {
                "open": self._config.gripper_open_position,
                "close": self._config.gripper_close_position,
            }[args["state"]]
            return GripperCommand.Goal(
                position=float(position),
                max_effort=float(self._config.gripper_max_effort),
            )
        if action == "rotate_base":
            return self._build_rotate_goal(args)
        raise ValueError(f"미지원 action: {action}")

    def _build_rotate_goal(self, args: dict) -> MoveToJoints.Goal:
        joint_name = self._config.rotate_joint_name
        if self._joint_cache is None:
            raise ValueError(f"{joint_name} joint state cache 가 없습니다")
        current = self._joint_cache.get(joint_name)
        if current is None:
            raise ValueError(f"{joint_name} joint state 가 없거나 오래되었습니다")
        target = resolve_rotate_target(
            current=current,
            direction=args["direction"],
            angle_deg=args["angle_deg"],
            cfg=self._config.rotate,
        )
        return MoveToJoints.Goal(
            joint_names=[joint_name],
            positions=[target],
            velocity_scale=float(self._config.rotate_velocity_scale),
        )

    def _wait_future(self, future, timeout_sec: float, cancel_event: threading.Event | None = None) -> bool:
        """future 완료를 event 로 대기. cancel_event 가 set 되면 조기 반환(False)."""
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        deadline_step = 0.05
        waited = 0.0
        while waited < timeout_sec:
            if done.wait(timeout=deadline_step):
                return True
            if cancel_event is not None and cancel_event.is_set():
                return False
            waited += deadline_step
        return done.is_set()
