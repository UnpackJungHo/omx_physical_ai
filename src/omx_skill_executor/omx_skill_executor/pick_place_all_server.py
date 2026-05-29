"""PickPlaceAll 스킬 서버 — 컵 외부의 모든 박스를 컵에 넣는다.

각 iter 마다 새로 scan/detect 한 뒤, cup polygon 내부에 이미 들어 있는 박스를
제외하고 그 외부에 박스가 있는 동안 pick&place 를 반복한다.

종료 조건:
1. 스캔/필터 결과 cup 외부 박스가 없음 (스윕까지 했는데도 없음) → 정상 종료
2. attempted_count >= max_boxes (실패도 카운트하여 무한 루프 방지)
3. cancel → canceled
4. 시스템 레벨 실패 (motion/TF/액션서버 미연결 등) → abort

cup 위치는 매 iter detect 마다 갱신된다. 한 박스 pick 이 실패해도 다음 iter
에서 다시 scan 으로 돌아가 외부 박스가 더 있으면 계속 시도한다.
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

from omx_interfaces.action import PickPlaceAll

from omx_skill_executor.pick_place_worker import (
    PickPlaceWorker,
    build_worker_config_from_node,
)


# goal.max_boxes 가 0/음수일 때 적용할 기본값.
DEFAULT_MAX_BOXES = 10
# 안전 상한 (무한 루프 방지용 절대 cap). 사용자 요구 범위 1~10.
HARD_MAX_BOXES = 10


class PickPlaceAllServer(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_all_server")
        self._cb_group = ReentrantCallbackGroup()
        config = build_worker_config_from_node(self)
        self._worker = PickPlaceWorker(self, config, self._cb_group)

        self._server = ActionServer(
            self,
            PickPlaceAll,
            "/omx/pick_place_all",
            execute_callback=self._execute_callback,
            callback_group=self._cb_group,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self._busy_lock = threading.Lock()
        self._active_goal_handle: Optional[ServerGoalHandle] = None
        # phase_cb 가 참조하는 진행 상태. self 에 두어 한 iter 중간 갱신이
        # 다음 feedback 에 즉시 반영되도록 한다.
        self._iteration = 0
        self._placed = 0
        self._max_boxes = 0
        self.get_logger().info("PickPlaceAllServer ready")

    # ────────────────────────────────────────────────────────────────
    # Action server callbacks
    # ────────────────────────────────────────────────────────────────

    def _goal_callback(self, _goal_request):
        if not self._busy_lock.acquire(blocking=False):
            self.get_logger().warn(
                "PickPlaceAll: already running, reject goal"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        self.get_logger().warn("PickPlaceAll: cancel requested")
        return CancelResponse.ACCEPT

    def _execute_callback(
        self, goal_handle: ServerGoalHandle,
    ) -> PickPlaceAll.Result:
        try:
            return self._run(goal_handle)
        finally:
            self._active_goal_handle = None
            self._worker.end_goal()
            if self._busy_lock.locked():
                self._busy_lock.release()

    def _run(
        self, goal_handle: ServerGoalHandle,
    ) -> PickPlaceAll.Result:
        goal = goal_handle.request
        self._max_boxes = self._clamp_max_boxes(int(goal.max_boxes))
        self._iteration = 0
        self._placed = 0
        result = PickPlaceAll.Result(
            success=False, message="",
            placed_count=0, attempted_count=0, max_boxes=self._max_boxes,
        )
        self._active_goal_handle = goal_handle
        self._worker.begin_goal(goal_handle)

        last_error = ""
        pick_attempts = 0      # pick 시도한 iter 수 (no-box 종료는 제외).

        def phase_cb(phase: str, status: str) -> None:
            fb = PickPlaceAll.Feedback(
                phase=phase, status=status,
                iteration=self._iteration,
                placed_so_far=self._placed,
            )
            goal_handle.publish_feedback(fb)
            self.get_logger().info(
                f"[iter {self._iteration}/{self._max_boxes}]"
                f"[{phase}] {status}"
            )

        for iteration in range(1, self._max_boxes + 1):
            self._iteration = iteration
            phase_cb(
                "iter_start",
                f"starting iter {iteration}/{self._max_boxes} "
                f"(placed={self._placed})",
            )

            pick = self._worker.pick_one_box(phase_cb)

            if pick.no_boxes_outside_cup:
                # 정상 종료: cup 외부 박스 없음. 이 iter 는 'pick 시도' 가
                # 아니라 종료 조건 평가였으므로 pick_attempts 에 포함하지 않음.
                self.get_logger().info(
                    f"PickPlaceAll: no boxes outside cup, "
                    f"completed after {self._placed} placement(s)"
                )
                break

            pick_attempts += 1
            result.attempted_count = pick_attempts
            result.placed_count = self._placed

            if pick.success:
                assert pick.target_box is not None and pick.cup is not None
                ok, err = self._worker.place_in_cup(pick.cup, phase_cb)
                if ok:
                    self._placed += 1
                    result.placed_count = self._placed
                    self.get_logger().info(
                        f"PickPlaceAll: placed {self._placed} "
                        f"(iter {iteration})"
                    )
                else:
                    last_error = err
                    self.get_logger().warn(
                        f"PickPlaceAll: place failed at iter {iteration}: "
                        f"{err}"
                    )
                    if self._worker.canceled:
                        return self._cancel_or_abort(
                            goal_handle, result, last_error
                        )
                    # place 실패: 다음 iter 로 진행 (재탐색).
            else:
                last_error = pick.error
                self.get_logger().warn(
                    f"PickPlaceAll: pick failed at iter {iteration}: "
                    f"{pick.error}"
                )
                if self._worker.canceled:
                    return self._cancel_or_abort(
                        goal_handle, result, last_error
                    )
                # 실패해도 다음 iter 로 (재탐색).

        result.attempted_count = pick_attempts
        result.placed_count = self._placed

        # 루프 종료 후 home 복귀 + 그리퍼 닫기.
        ok, err = self._worker.return_home_close(phase_cb)
        if not ok:
            return self._cancel_or_abort(goal_handle, result, err)

        result.success = True
        if self._placed == 0 and pick_attempts == 0:
            result.message = "no boxes outside cup at start"
        elif self._placed == 0:
            result.message = (
                f"placed 0/{pick_attempts} attempts (last_error: {last_error})"
                if last_error else f"placed 0/{pick_attempts} attempts"
            )
        else:
            result.message = (
                f"placed {self._placed} box(es) into cup "
                f"({pick_attempts} iter, cap={self._max_boxes})"
            )
        goal_handle.succeed()
        self.get_logger().info(result.message)
        return result

    # ────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────

    @staticmethod
    def _clamp_max_boxes(requested: int) -> int:
        if requested <= 0:
            return DEFAULT_MAX_BOXES
        return min(requested, HARD_MAX_BOXES)

    def _cancel_or_abort(
        self,
        goal_handle: ServerGoalHandle,
        result: PickPlaceAll.Result,
        error: str,
    ) -> PickPlaceAll.Result:
        result.success = False
        result.placed_count = self._placed
        detail = self._worker.last_action_error
        result.message = f"{error} | {detail}" if detail else error
        if self._worker.canceled:
            self.get_logger().warn(
                f"PickPlaceAll canceled: {result.message} "
                f"(placed={self._placed}/{result.attempted_count})"
            )
            if goal_handle.is_active:
                goal_handle.canceled()
        else:
            self.get_logger().error(
                f"PickPlaceAll failed: {result.message} "
                f"(placed={self._placed}/{result.attempted_count})"
            )
            if goal_handle.is_active:
                goal_handle.abort()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickPlaceAllServer()
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
