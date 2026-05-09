"""PickDetected 스킬 서버.

0. 스킬 goal 수신
1. 스캔 포즈로 이동 (MoveToJoints)
2. 그리퍼 열기 (GripperCommand position=open)
3. /omx/get_block_poses 로 블록 탐지.
   탐지 실패 시 joint1 을 절대각 기준 -10° 씩 -90° 까지,
   그래도 실패면 0° 로 복귀 후 +10° 씩 +90° 까지 스윕.
   끝까지 실패면 home 복귀 후 실패 반환.
4. 탐지 성공 시 CLI 로 red/green/blue 선택 (goal.object_color 가 채워져 있으면 생략).
5. 선택된 블록의 grasp_pose 를 받아
   (a) hover: (x, y, z=hover_z), orientation 그대로 → MoveToPose
   (b) descent: (x, y, z=grasp_z) → MoveToPose
6. 그리퍼 닫기
7. 스캔 포즈로 복귀
8. 그리퍼 열기 → 성공 반환
9. 그리퍼 닫기
10. home 복귀
"""

from __future__ import annotations

import math
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Header

from omx_interfaces.action import (
    GripperCommand,
    MoveToJoints,
    MoveToNamed,
    MoveToPose,
    PickDetected,
)
from omx_interfaces.msg import BlockPose
from omx_interfaces.srv import GetBlockPoses


VALID_COLORS = ("red", "green", "blue")


@dataclass
class SkillConfig:
    scan_joint_positions_deg: tuple[float, ...]
    arm_joint_names: tuple[str, ...]
    sweep_joint_index: int
    sweep_step_deg: float
    sweep_positive_max_deg: float
    sweep_negative_min_deg: float
    detect_wait_after_motion_sec: float
    get_block_poses_timeout_sec: float
    approach_frame_id: str
    hover_z: float
    grasp_z: float
    move_velocity_scale: float
    hover_settle_sec: float
    gripper_open_position: float
    gripper_closed_position: float
    gripper_max_effort: float
    action_goal_timeout_sec: float
    action_result_timeout_sec: float


class PickDetectedServer(Node):
    def __init__(self) -> None:
        super().__init__("pick_detected_server")
        self._cb_group = ReentrantCallbackGroup()
        self._config = self._load_config()

        # 클라이언트
        self._move_joints_cli = ActionClient(
            self, MoveToJoints, "/omx/move_to_joints",
            callback_group=self._cb_group,
        )
        self._move_pose_cli = ActionClient(
            self, MoveToPose, "/omx/move_to_pose",
            callback_group=self._cb_group,
        )
        self._move_named_cli = ActionClient(
            self, MoveToNamed, "/omx/move_to_named",
            callback_group=self._cb_group,
        )
        self._gripper_cli = ActionClient(
            self, GripperCommand, "/omx/gripper_command",
            callback_group=self._cb_group,
        )
        self._blocks_cli = self.create_client(
            GetBlockPoses, "/omx/get_block_poses",
            callback_group=self._cb_group,
        )

        # 서버
        self._server = ActionServer(
            self,
            PickDetected,
            "/omx/pick_detected",
            execute_callback=self._execute_callback,
            callback_group=self._cb_group,
            goal_callback=self._goal_callback,
        )

        self._busy_lock = threading.Lock()
        self._last_action_error = ""
        self.get_logger().info("PickDetectedServer ready")

    # ────────────────────────────────────────────────────────────────
    # Config
    # ────────────────────────────────────────────────────────────────

    def _load_config(self) -> SkillConfig:
        p = self.declare_parameter
        return SkillConfig(
            scan_joint_positions_deg=tuple(
                p("scan_joint_positions_deg", [0.0, -25.0, -63.0, 107.0, 0.0]).value
            ),
            arm_joint_names=tuple(
                p("arm_joint_names",
                  ["joint1", "joint2", "joint3", "joint4", "joint5"]).value
            ),
            sweep_joint_index=int(p("sweep_joint_index", 0).value),
            sweep_step_deg=float(p("sweep_step_deg", 10.0).value),
            sweep_positive_max_deg=float(p("sweep_positive_max_deg", 90.0).value),
            sweep_negative_min_deg=float(p("sweep_negative_min_deg", -90.0).value),
            detect_wait_after_motion_sec=float(
                p("detect_wait_after_motion_sec", 0.8).value
            ),
            get_block_poses_timeout_sec=float(
                p("get_block_poses_timeout_sec", 2.0).value
            ),
            approach_frame_id=str(p("approach_frame_id", "world").value),
            hover_z=float(p("hover_z", 0.15).value),
            grasp_z=float(p("grasp_z", 0.07).value),
            move_velocity_scale=float(p("move_velocity_scale", 0.2).value),
            hover_settle_sec=float(p("hover_settle_sec", 0.5).value),
            gripper_open_position=float(p("gripper_open_position", 1.0).value),
            gripper_closed_position=float(p("gripper_closed_position", 0.0).value),
            gripper_max_effort=float(p("gripper_max_effort", 0.0).value),
            action_goal_timeout_sec=float(p("action_goal_timeout_sec", 5.0).value),
            action_result_timeout_sec=float(
                p("action_result_timeout_sec", 60.0).value
            ),
        )

    # ────────────────────────────────────────────────────────────────
    # Action server callbacks
    # ────────────────────────────────────────────────────────────────

    def _goal_callback(self, _goal_request):
        if not self._busy_lock.acquire(blocking=False):
            self.get_logger().warn("PickDetected: already running, reject goal")
            from rclpy.action import GoalResponse
            return GoalResponse.REJECT
        from rclpy.action import GoalResponse
        return GoalResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle) -> PickDetected.Result:
        try:
            return self._run_pick(goal_handle)
        finally:
            if self._busy_lock.locked():
                self._busy_lock.release()

    def _run_pick(self, goal_handle: ServerGoalHandle) -> PickDetected.Result:
        goal = goal_handle.request
        result = PickDetected.Result(success=False, message="", attempts=0)

        self._publish_phase(goal_handle, "detecting", "moving to scan pose")
        if not self._move_to_scan():
            return self._fail(goal_handle, result, "move to scan pose failed")

        self._publish_phase(goal_handle, "detecting", "opening gripper")
        if not self._send_gripper(self._config.gripper_open_position):
            return self._fail(goal_handle, result, "open gripper failed")

        self._publish_phase(goal_handle, "detecting", "initial detection")
        blocks = self._detect_blocks()
        result.attempts = 1

        if not blocks:
            blocks, result.attempts = self._sweep_for_blocks(goal_handle, result.attempts)

        if not blocks:
            self._publish_phase(goal_handle, "detecting", "no blocks, returning home")
            self._move_to_named("home")
            return self._fail(goal_handle, result, "no blocks detected after sweep")

        # 색 선택
        target_color = (goal.object_color or "").strip().lower()
        target_block = self._select_block(blocks, target_color)
        if target_block is None:
            if target_color:
                msg = f"requested color '{target_color}' not in detected blocks"
            else:
                msg = "color selection cancelled or invalid"
            return self._fail(goal_handle, result, msg)

        self.get_logger().info(
            f"Selected block color='{target_block.color}' "
            f"pos=({target_block.grasp_pose.pose.position.x:.3f}, "
            f"{target_block.grasp_pose.pose.position.y:.3f}, "
            f"{target_block.grasp_pose.pose.position.z:.3f}) "
            f"conf={target_block.confidence:.2f}"
        )

        # hover
        self._publish_phase(goal_handle, "approaching", "hover above target")
        hover_pose = self._build_pose(target_block.grasp_pose, self._config.hover_z)
        if not self._send_move_to_pose(hover_pose):
            return self._fail(goal_handle, result, "hover motion failed")

        # hover 종료 후 controller 가 완전히 정지하도록 짧게 대기.
        # 이게 없으면 plan 시 start state 로 잡힌 값과 execute 시점의 current
        # joint state 가 살짝 벗어나 CONTROL_FAILED 로 abort 되는 경우가 있음.
        if self._config.hover_settle_sec > 0.0:
            time.sleep(self._config.hover_settle_sec)

        # descent
        self._publish_phase(goal_handle, "approaching", "descending to grasp")
        grasp_pose = self._build_pose(target_block.grasp_pose, self._config.grasp_z)
        if not self._send_move_to_pose(grasp_pose):
            return self._fail(goal_handle, result, "descent motion failed")

        # close gripper
        self._publish_phase(goal_handle, "grasping", "closing gripper")
        if not self._send_gripper(self._config.gripper_closed_position):
            return self._fail(goal_handle, result, "close gripper failed")

        # back to scan pose
        self._publish_phase(goal_handle, "returning", "returning to scan pose")
        if not self._move_to_scan():
            return self._fail(goal_handle, result, "return to scan pose failed")

        # release
        self._publish_phase(goal_handle, "returning", "releasing")
        if not self._send_gripper(self._config.gripper_open_position):
            return self._fail(goal_handle, result, "release gripper failed")

        # 후처리: 그리퍼 닫기 → home 복귀 (다음 스킬 받을 준비)
        self._publish_phase(goal_handle, "returning", "closing gripper after release")
        if not self._send_gripper(self._config.gripper_closed_position):
            return self._fail(goal_handle, result, "post-release close gripper failed")

        self._publish_phase(goal_handle, "returning", "returning to home")
        if not self._move_to_named("home"):
            return self._fail(goal_handle, result, "return to home failed")

        result.success = True
        result.message = f"picked '{target_block.color}' block"
        goal_handle.succeed()
        self.get_logger().info(result.message)
        return result

    # ────────────────────────────────────────────────────────────────
    # High-level steps
    # ────────────────────────────────────────────────────────────────

    def _move_to_scan(self) -> bool:
        positions_rad = [math.radians(v) for v in self._config.scan_joint_positions_deg]
        return self._send_move_to_joints(positions_rad)

    def _sweep_for_blocks(
        self,
        goal_handle: ServerGoalHandle,
        attempts_so_far: int,
    ) -> tuple[list[BlockPose], int]:
        """joint1 절대각 스윕. 탐지 성공 시 즉시 반환."""
        cfg = self._config
        scan_deg = list(cfg.scan_joint_positions_deg)
        idx = cfg.sweep_joint_index
        attempts = attempts_so_far

        def angles_to_try() -> list[float]:
            seq: list[float] = []
            # - 방향: -step_deg, -2*step_deg, ..., negative_min_deg
            n_neg = int(abs(cfg.sweep_negative_min_deg) / cfg.sweep_step_deg)
            for i in range(1, n_neg + 1):
                seq.append(-i * cfg.sweep_step_deg)
            # 0 으로 리셋 (초기 포즈 복귀)
            seq.append(0.0)
            # + 방향
            n_pos = int(cfg.sweep_positive_max_deg / cfg.sweep_step_deg)
            for i in range(1, n_pos + 1):
                seq.append(i * cfg.sweep_step_deg)
            return seq

        for angle_deg in angles_to_try():
            scan_deg[idx] = angle_deg
            self._publish_phase(
                goal_handle, "detecting",
                f"sweep joint{idx + 1}={angle_deg:+.1f}°",
            )
            positions_rad = [math.radians(v) for v in scan_deg]
            if not self._send_move_to_joints(positions_rad):
                self.get_logger().warn(
                    f"sweep motion failed at joint1={angle_deg:+.1f}°, continuing"
                )
                continue
            attempts += 1
            blocks = self._detect_blocks()
            if blocks:
                return blocks, attempts
        return [], attempts

    def _detect_blocks(self) -> list[BlockPose]:
        time.sleep(self._config.detect_wait_after_motion_sec)
        if not self._blocks_cli.wait_for_service(
            timeout_sec=self._config.get_block_poses_timeout_sec
        ):
            self.get_logger().warn("/omx/get_block_poses service unavailable")
            return []
        request = GetBlockPoses.Request(color="")
        future = self._blocks_cli.call_async(request)
        response = self._wait_future(future, self._config.get_block_poses_timeout_sec)
        if response is None:
            self.get_logger().warn("GetBlockPoses timed out")
            return []
        return list(response.blocks)

    def _select_block(
        self,
        blocks: list[BlockPose],
        preselected_color: str,
    ) -> Optional[BlockPose]:
        available = sorted({b.color for b in blocks if b.color in VALID_COLORS})
        if preselected_color:
            color = preselected_color
        else:
            color = self._prompt_color(available)
        if not color:
            return None
        candidates = [b for b in blocks if b.color == color]
        if not candidates:
            self.get_logger().warn(f"color '{color}' not in detected set {available}")
            return None
        # 신뢰도 최고 블록 선택
        return max(candidates, key=lambda b: b.confidence)

    def _prompt_color(self, available: list[str]) -> str:
        print(f"\n[PickDetected] Detected colors: {available}", flush=True)
        attempts = 0
        while attempts < 2:
            print("Select color (red / green / blue): ", end="", flush=True)
            try:
                choice = sys.stdin.readline().strip().lower()
            except Exception:  # noqa: BLE001
                return ""
            if choice in available:
                return choice
            if choice in VALID_COLORS:
                print(f"'{choice}' not detected. Retry.", flush=True)
            else:
                print("Invalid input. Use red / green / blue.", flush=True)
            attempts += 1
        print("Color selection failed.", flush=True)
        return ""

    def _build_pose(self, reference: PoseStamped, z: float) -> PoseStamped:
        ref = reference.pose
        return PoseStamped(
            header=Header(frame_id=self._config.approach_frame_id),
            pose=Pose(
                position=Point(x=ref.position.x, y=ref.position.y, z=z),
                orientation=ref.orientation,
            ),
        )

    # ────────────────────────────────────────────────────────────────
    # Action client wrappers
    # ────────────────────────────────────────────────────────────────

    def _send_move_to_joints(self, positions_rad: list[float]) -> bool:
        goal = MoveToJoints.Goal(
            joint_names=list(self._config.arm_joint_names),
            positions=list(positions_rad),
            velocity_scale=float(self._config.move_velocity_scale),
        )
        return self._run_action_goal(self._move_joints_cli, goal, "MoveToJoints")

    def _send_move_to_pose(self, pose: PoseStamped) -> bool:
        goal = MoveToPose.Goal(
            target_pose=pose,
            velocity_scale=float(self._config.move_velocity_scale),
            plan_only=False,
        )
        return self._run_action_goal(self._move_pose_cli, goal, "MoveToPose")

    def _send_gripper(self, position: float) -> bool:
        goal = GripperCommand.Goal(
            position=float(position),
            max_effort=float(self._config.gripper_max_effort),
        )
        return self._run_action_goal(self._gripper_cli, goal, "GripperCommand")

    def _move_to_named(self, name: str) -> bool:
        goal = MoveToNamed.Goal(name=name)
        return self._run_action_goal(self._move_named_cli, goal, f"MoveToNamed({name})")

    def _run_action_goal(self, client: ActionClient, goal, label: str) -> bool:
        # 직전 실패 사유를 보존해 skill result.message 에 그대로 전파할 수 있게 한다.
        self._last_action_error = ""
        if not client.wait_for_server(timeout_sec=self._config.action_goal_timeout_sec):
            self._last_action_error = f"{label}: action server unavailable"
            self.get_logger().error(self._last_action_error)
            return False
        send_future = client.send_goal_async(goal)
        gh = self._wait_future(send_future, self._config.action_goal_timeout_sec)
        if gh is None:
            self._last_action_error = f"{label}: send_goal timed out"
            self.get_logger().error(self._last_action_error)
            return False
        if not gh.accepted:
            self._last_action_error = f"{label}: goal rejected"
            self.get_logger().error(self._last_action_error)
            return False
        result_future = gh.get_result_async()
        wrapped = self._wait_future(result_future, self._config.action_result_timeout_sec)
        if wrapped is None:
            self._last_action_error = f"{label}: result timed out"
            self.get_logger().error(self._last_action_error)
            return False
        result = wrapped.result
        success = bool(getattr(result, "success", False))
        if not success:
            msg = getattr(result, "message", "")
            self._last_action_error = f"{label} failed: {msg}"
            self.get_logger().error(self._last_action_error)
        return success

    @staticmethod
    def _wait_future(future, timeout_sec: float):
        """외부 MultiThreadedExecutor 가 spin 중이라는 가정하에 폴링 대기."""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.02)
        if not future.done():
            return None
        return future.result()

    # ────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────

    def _publish_phase(
        self,
        goal_handle: ServerGoalHandle,
        phase: str,
        status: str,
    ) -> None:
        fb = PickDetected.Feedback(phase=phase, status=status)
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f"[{phase}] {status}")

    def _fail(
        self,
        goal_handle: ServerGoalHandle,
        result: PickDetected.Result,
        message: str,
    ) -> PickDetected.Result:
        result.success = False
        # 하위 action 에서 보존한 구체적 사유(MoveItErrorCode 포함)를 덧붙여
        # 클라이언트 측 `ros2 action send_goal` 출력에서도 원인이 보이게 한다.
        detail = self._last_action_error
        result.message = f"{message} | {detail}" if detail else message
        self.get_logger().error(f"PickDetected failed: {result.message}")
        if goal_handle.is_active:
            goal_handle.abort()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickDetectedServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
