"""PickPlace 스킬 서버 — 박스 한 개를 종이컵에 넣는다.

흐름은 pick_place_worker.PickPlaceWorker 가 캡슐화한다. 본 서버는
ActionServer 콜백, feedback 발행, busy-lock 만 책임진다.

cup 내부에 이미 들어가 있는 박스는 worker 가 자동으로 제외한다 (polygon 필터).
컵 외부에 박스가 없으면 'no box outside cup' 으로 실패 반환된다 (스윕까지 한 뒤).
"""
from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from omx_interfaces.action import PickPlace

from omx_skill_executor.pick_place_worker import (
    PickPlaceWorker,
    build_worker_config_from_node,
)


class PickPlaceServer(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_server")
        self._cb_group = ReentrantCallbackGroup()
        config = build_worker_config_from_node(self)
        self._worker = PickPlaceWorker(self, config, self._cb_group)

        self._server = ActionServer(
            self,
            PickPlace,
            "/omx/pick_place",
            execute_callback=self._execute_callback,
            callback_group=self._cb_group,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self._busy_lock = threading.Lock()
        self._active_goal_handle: Optional[ServerGoalHandle] = None
        self.get_logger().info("PickPlaceServer ready")

    # ────────────────────────────────────────────────────────────────
    # Action server callbacks
    # ────────────────────────────────────────────────────────────────

    def _goal_callback(self, _goal_request):
        if not self._busy_lock.acquire(blocking=False):
            self.get_logger().warn("PickPlace: already running, reject goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        self.get_logger().warn("PickPlace: cancel requested")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle) -> PickPlace.Result:
        try:
            return self._run(goal_handle)
        finally:
            self._active_goal_handle = None
            self._worker.end_goal()
            if self._busy_lock.locked():
                self._busy_lock.release()

    def _run(self, goal_handle: ServerGoalHandle) -> PickPlace.Result:
        result = PickPlace.Result(success=False, message="", attempts=0)
        self._active_goal_handle = goal_handle
        self._worker.begin_goal(goal_handle)
        
        target_color = goal_handle.request.object_color.strip().lower()

        def phase_cb(phase: str, status: str) -> None:
            fb = PickPlace.Feedback(phase=phase, status=status)
            goal_handle.publish_feedback(fb)
            self.get_logger().info(f"[{phase}] {status}")

        pick = self._worker.pick_one_box(phase_cb, target_color=target_color)
        result.attempts = pick.attempts
        if not pick.success:
            return self._fail(goal_handle, result, pick.error)

        assert pick.target_box is not None and pick.cup is not None

        ok, err = self._worker.place_in_cup(pick.cup, phase_cb)
        if not ok:
            return self._fail(goal_handle, result, err)

        ok, err = self._worker.return_home_close(phase_cb)
        if not ok:
            return self._fail(goal_handle, result, err)

        result.success = True
        result.message = f"placed '{pick.target_box.color}' box into cup"
        goal_handle.succeed()
        self.get_logger().info(result.message)
        return result

    def _fail(
        self,
        goal_handle: ServerGoalHandle,
        result: PickPlace.Result,
        message: str,
    ) -> PickPlace.Result:
        result.success = False
        detail = self._worker.last_action_error
        result.message = f"{message} | {detail}" if detail else message
        if self._worker.canceled:
            self.get_logger().warn(f"PickPlace canceled: {result.message}")
            if goal_handle.is_active:
                goal_handle.canceled()
        else:
            self.get_logger().error(f"PickPlace failed: {result.message}")
            if goal_handle.is_active:
                goal_handle.abort()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickPlaceServer()
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
