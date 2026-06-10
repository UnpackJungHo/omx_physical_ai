"""ExecuteCommand action server — 자연어 명령 -> plan -> 순차 실행.

흐름:
  planning   : llm_client.generate_plan(command) 로 정규화된 Plan 획득
  validating : (build_plan 단계에서 검증 완료) plan_json/steps_total 확정
  executing  : 각 step 을 SkillDispatcher 로 순차 실행
  done       : Result(success, plan_json, steps_completed)

실패는 모두 명시적으로 abort. clarify/실패 복구는 모델 재추론 없이 코드가
결정론적으로 처리한다(중단 후 보고). continue_on_fail 파라미터로 step 실패 시
중단/계속을 선택한다.

LLM 클라이언트는 주입(런타임 OllamaLLMClient, 테스트 MockLLMClient)한다.
"""
from __future__ import annotations

import json
import threading
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState

from omx_interfaces.action import ExecuteCommand

from omx_llm_planner.joint_state_cache import JointStateCache
from omx_llm_planner.llm_client import LLMClient, LLMUnavailable, OllamaLLMClient
from omx_llm_planner.plan_schema import Plan, PlanError
from omx_llm_planner.rotate_math import RotateConfig
from omx_llm_planner.skill_clients import DispatcherConfig, SkillDispatcher

DEFAULT_SYSTEM_PROMPT = (
    "너는 로봇 명령 파서다. 사용자의 한국어 명령을 다음 JSON 형식의 plan 으로만 "
    "응답한다: {\"steps\":[{\"action\":..., \"args\":{...}}]}. "
    "action 은 pick_place(args.object_color: red|blue|green), "
    "pick_place_all(args.max_boxes:1-10, args.retry_on_fail:bool), "
    "move_to_named(args.name: home|init), "
    "gripper(args.state: open|close), "
    "rotate_base(args.direction: left|right, args.angle_deg:1-180) 만 사용한다. "
    "이해할 수 없으면 {\"steps\":[]} 로 응답한다."
)


class PlannerNode(Node):
    def __init__(self, llm_client: Optional[LLMClient] = None) -> None:
        super().__init__("llm_planner")
        self._cb_group = ReentrantCallbackGroup()
        self._declare_params()
        self._llm_client = llm_client or self._build_ollama_client()
        self._joint_cache = JointStateCache(
            max_age_sec=self.get_parameter("joint_state_max_age_sec").value,
            clock_now=lambda: self.get_clock().now().nanoseconds * 1e-9)
        self._joint_sub = self.create_subscription(
            JointState, self.get_parameter("joint_states_topic").value,
            self._on_joint_state, 10, callback_group=self._cb_group)
        self._dispatcher = SkillDispatcher(
            self, self._cb_group, self._dispatcher_config(), self._joint_cache)
        self._continue_on_fail = self.get_parameter("continue_on_fail").value

        self._busy_lock = threading.Lock()
        self._cancel_event = threading.Event()

        self._server = ActionServer(
            self,
            ExecuteCommand,
            "omx/execute_command",
            execute_callback=self._execute_callback,
            callback_group=self._cb_group,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info("PlannerNode ready (omx/execute_command)")

    # ── parameters ────────────────────────────────────────────────
    def _declare_params(self) -> None:
        self.declare_parameter("llm_endpoint", "http://127.0.0.1:11434")
        self.declare_parameter("llm_model_name", "qwen3-4b-omx")
        self.declare_parameter("llm_system_prompt", DEFAULT_SYSTEM_PROMPT)
        self.declare_parameter("llm_request_timeout_sec", 20.0)
        self.declare_parameter("llm_max_retries", 1)
        self.declare_parameter("continue_on_fail", False)
        self.declare_parameter("pick_place_action", "omx/pick_place")
        self.declare_parameter("pick_place_all_action", "omx/pick_place_all")
        self.declare_parameter("move_to_named_action", "omx/move_to_named")
        self.declare_parameter("server_wait_timeout_sec", 5.0)
        self.declare_parameter("goal_response_timeout_sec", 5.0)
        self.declare_parameter("result_timeout_sec", 120.0)
        self.declare_parameter("gripper_action", "omx/gripper_command")
        self.declare_parameter("move_to_joints_action", "omx/move_to_joints")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("rotate_joint_name", "joint1")
        self.declare_parameter("rotate_velocity_scale", 0.3)
        self.declare_parameter("joint_state_max_age_sec", 1.0)
        self.declare_parameter("rotate_sign_left", 1.0)    # 하드웨어 검증 후 확정
        self.declare_parameter("rotate_sign_right", -1.0)
        self.declare_parameter("joint1_lower", -2.8)       # URDF limit 으로 교체
        self.declare_parameter("joint1_upper", 2.8)
        self.declare_parameter("gripper_open_position", 1.0)
        self.declare_parameter("gripper_close_position", 0.0)

    # 초기 세팅
    def _build_ollama_client(self) -> OllamaLLMClient:
        return OllamaLLMClient(
            endpoint=self.get_parameter("llm_endpoint").value,
            model_name=self.get_parameter("llm_model_name").value,
            system_prompt=self.get_parameter("llm_system_prompt").value,
            request_timeout_sec=self.get_parameter("llm_request_timeout_sec").value,
            max_retries=self.get_parameter("llm_max_retries").value,
        )

    def _on_joint_state(self, msg) -> None:
        self._joint_cache.update(msg, stamp=self.get_clock().now().nanoseconds * 1e-9)

    def _dispatcher_config(self) -> DispatcherConfig:
        return DispatcherConfig(
            pick_place_action=self.get_parameter("pick_place_action").value,
            pick_place_all_action=self.get_parameter("pick_place_all_action").value,
            move_to_named_action=self.get_parameter("move_to_named_action").value,
            server_wait_timeout_sec=self.get_parameter("server_wait_timeout_sec").value,
            goal_response_timeout_sec=self.get_parameter("goal_response_timeout_sec").value,
            result_timeout_sec=self.get_parameter("result_timeout_sec").value,
            gripper_action=self.get_parameter("gripper_action").value,
            move_to_joints_action=self.get_parameter("move_to_joints_action").value,
            rotate_joint_name=self.get_parameter("rotate_joint_name").value,
            rotate_velocity_scale=self.get_parameter("rotate_velocity_scale").value,
            rotate=RotateConfig(
                sign={"left": self.get_parameter("rotate_sign_left").value,
                      "right": self.get_parameter("rotate_sign_right").value},
                joint_lower=self.get_parameter("joint1_lower").value,
                joint_upper=self.get_parameter("joint1_upper").value),
            gripper_open_position=self.get_parameter("gripper_open_position").value,
            gripper_close_position=self.get_parameter("gripper_close_position").value,
        )

    # ── action callbacks ──────────────────────────────────────────
    def _goal_callback(self, _goal_request):
        if not self._busy_lock.acquire(blocking=False):
            self.get_logger().warn("execute_command: already running, reject goal")
            return GoalResponse.REJECT
        self._cancel_event.clear()
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        self.get_logger().warn("execute_command: cancel requested")
        self._cancel_event.set()
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle) -> ExecuteCommand.Result:
        try:
            return self._run(goal_handle)
        finally:
            if self._busy_lock.locked():
                self._busy_lock.release()

    # ── orchestration ─────────────────────────────────────────────
    def _run(self, goal_handle: ServerGoalHandle) -> ExecuteCommand.Result:
        result = ExecuteCommand.Result(
            success=False, message="", plan_json="", steps_total=0, steps_completed=0)
        command = goal_handle.request.command
        dry_run = goal_handle.request.dry_run

        # planning + validating
        self._feedback(goal_handle, "planning", 0, 0, command)
        try:
            plan = self._llm_client.generate_plan(command)
        except LLMUnavailable as exc:
            return self._abort(goal_handle, result, f"LLM 연결 실패: {exc}")
        except PlanError as exc:
            return self._abort(goal_handle, result, f"명령 해석 실패: {exc}")

        result.plan_json = _plan_to_json(plan)
        result.steps_total = len(plan.steps)
        self._feedback(goal_handle, "validating", 0, result.steps_total, command)

        if dry_run:
            result.success = True
            result.message = f"dry_run: {result.steps_total} step plan 검증 완료"
            goal_handle.succeed()
            return result

        # executing
        for i, step in enumerate(plan.steps, start=1):
            if self._cancel_event.is_set():
                return self._cancel(goal_handle, result, "실행 전 취소됨")
            desc = f"{step.action} {step.args}"
            self._feedback(goal_handle, "executing", i, result.steps_total, desc)
            step_result = self._dispatcher.execute_step(step.action, step.args, self._cancel_event)
            if step_result.success:
                result.steps_completed = i
                continue
            if self._cancel_event.is_set():
                return self._cancel(goal_handle, result, f"step {i} 취소됨")
            msg = f"step {i}/{result.steps_total} 실패: {step_result.message}"
            if not self._continue_on_fail:
                return self._abort(goal_handle, result, msg)
            self.get_logger().warn(f"{msg} (continue_on_fail)")

        result.success = result.steps_completed == result.steps_total
        result.message = (
            f"{result.steps_completed}/{result.steps_total} step 완료"
        )
        self._feedback(goal_handle, "done", result.steps_total, result.steps_total, result.message)
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    # ── helpers ───────────────────────────────────────────────────
    def _feedback(self, goal_handle, phase: str, step_index: int, steps_total: int, desc: str) -> None:
        fb = ExecuteCommand.Feedback(
            phase=phase, step_index=step_index, steps_total=steps_total, step_desc=desc)
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f"[{phase}] {step_index}/{steps_total} {desc}")

    def _abort(self, goal_handle, result, message: str):
        result.success = False
        result.message = message
        self.get_logger().error(f"execute_command abort: {message}")
        if goal_handle.is_active:
            goal_handle.abort()
        return result

    def _cancel(self, goal_handle, result, message: str):
        result.success = False
        result.message = message
        self.get_logger().warn(f"execute_command canceled: {message}")
        if goal_handle.is_active:
            goal_handle.canceled()
        return result


def _plan_to_json(plan: Plan) -> str:
    return json.dumps(
        {"steps": [{"action": s.action, "args": s.args} for s in plan.steps]},
        ensure_ascii=False,
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlannerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
