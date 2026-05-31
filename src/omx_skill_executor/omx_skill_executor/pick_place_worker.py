"""PickPlace / PickPlaceAll 공통 worker.

ROS2 노드(Node) 를 외부에서 받아 액션 클라이언트, 서비스 클라이언트,
TF buffer, joint state 구독을 구성한다. action server / feedback 발행만
호출부(서버) 가 책임지며, 본 모듈은 한 박스에 대한 pick&place 절차와
재탐지·sweep·cup-내부 필터 같은 공통 동작을 제공한다.

사용 흐름:
    worker = PickPlaceWorker(node, config)
    worker.begin_goal(goal_handle)
    try:
        result = worker.pick_one_box(phase_cb)
        if result.success:
            worker.place_in_cup(result.cup, phase_cb)
            worker.return_home_close(phase_cb)
    finally:
        worker.end_goal()

phase_cb 는 (phase: str, status: str) -> None. server 가 자신의 feedback
메시지로 발행한다.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.action import ActionClient
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import CallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from omx_interfaces.action import (
    GripperCommand,
    MoveToJoints,
    MoveToNamed,
    MoveToPose,
)
from omx_interfaces.msg import BlockPose
from omx_interfaces.srv import CheckGrasp, GetBlockPoses

from omx_skill_executor.pick_place_geometry import (
    is_box_in_cup,
    jaw_axis_yaw_from_quaternion,
    wrap_to_pm45,
    wrap_yaw_zero_pi_over_2,
    yaw_from_quaternion,
)

BOX_COLOR = "box"
CUP_COLOR = "cup"

PhaseCallback = Callable[[str, str], None]


@dataclass
class WorkerConfig:
    """worker 가 사용하는 파라미터 묶음. 서버가 ROS 파라미터에서 읽어 채워준다."""
    scan_joint_positions_deg: tuple[float, ...]
    arm_joint_names: tuple[str, ...]
    sweep_joint_index: int
    sweep_step_deg: float
    sweep_positive_max_deg: float
    sweep_negative_min_deg: float
    detect_wait_after_motion_sec: float
    service_call_timeout_sec: float
    world_poses_service_name: str
    approach_frame_id: str
    hover_z: float
    grasp_z: float
    move_velocity_scale: float
    hover_settle_sec: float
    refine_at_hover: bool
    refine_match_tolerance_m: float
    drop_clearance_m: float
    world_frame: str
    gripper_link: str
    joint_states_topic: str
    yaw_min_corner_confidence: float
    joint5_yaw_sign: float
    require_yaw: bool
    gripper_open_position: float
    gripper_closed_position: float
    gripper_max_effort: float
    action_goal_timeout_sec: float
    action_result_timeout_sec: float
    max_grasp_retries: int
    check_grasp_service_name: str
    grasp_settle_sec: float
    yaw_align_max_iterations: int
    yaw_align_tolerance_rad: float
    yaw_align_max_total_correction_rad: float
    yaw_align_iter_settle_sec: float


@dataclass
class PickResult:
    """한 박스 pick 시도 결과.

    success=True 면 박스를 잡아 scan 자세까지 복귀 + grasp verify 통과한 상태.
    이때 target_box, cup 도 유효하다.

    success=False 일 때 no_boxes_outside_cup=True 면 "스캔/필터 결과 컵 외부에
    잡을 박스가 없다" 라는 정상 종료 신호 (pick_place_all 의 종료 조건).
    """
    success: bool
    error: str = ""
    target_box: Optional[BlockPose] = None
    cup: Optional[BlockPose] = None
    attempts: int = 0
    no_boxes_outside_cup: bool = False


class PickPlaceWorker:
    """단일 박스 pick&place 절차를 캡슐화한 worker.

    Note:
        server 가 새 goal 을 받기 전 begin_goal(gh) 를 호출해 cancel 처리
        대상 핸들을 등록해야 한다. cancel 후에는 end_goal() 로 정리한다.
    """

    def __init__(
        self,
        node: Node,
        config: WorkerConfig,
        cb_group: CallbackGroup,
    ) -> None:
        self._node = node
        self._config = config
        self._cb_group = cb_group

        # 액션 클라이언트
        self._move_joints_cli = ActionClient(
            node, MoveToJoints, "omx/move_to_joints",
            callback_group=cb_group,
        )
        self._move_pose_cli = ActionClient(
            node, MoveToPose, "omx/move_to_pose",
            callback_group=cb_group,
        )
        self._move_named_cli = ActionClient(
            node, MoveToNamed, "omx/move_to_named",
            callback_group=cb_group,
        )
        self._gripper_cli = ActionClient(
            node, GripperCommand, "omx/gripper_command",
            callback_group=cb_group,
        )
        self._blocks_cli = node.create_client(
            GetBlockPoses, config.world_poses_service_name,
            callback_group=cb_group,
        )
        self._check_grasp_cli = node.create_client(
            CheckGrasp, config.check_grasp_service_name,
            callback_group=cb_group,
        )

        # joint state / TF
        self._joint_state_lock = threading.Lock()
        self._latest_joint_state: Optional[JointState] = None
        node.create_subscription(
            JointState, config.joint_states_topic,
            self._on_joint_state, 10, callback_group=cb_group,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        self._active_goal_handle: Optional[ServerGoalHandle] = None
        self._canceled = False
        self._last_action_error = ""

    # ────────────────────────────────────────────────────────────────
    # Goal lifecycle (server 가 호출)
    # ────────────────────────────────────────────────────────────────

    def begin_goal(self, goal_handle: ServerGoalHandle) -> None:
        self._active_goal_handle = goal_handle
        self._canceled = False
        self._last_action_error = ""

    def end_goal(self) -> None:
        self._active_goal_handle = None

    @property
    def canceled(self) -> bool:
        return self._canceled

    @property
    def last_action_error(self) -> str:
        return self._last_action_error

    # ────────────────────────────────────────────────────────────────
    # High-level orchestrations
    # ────────────────────────────────────────────────────────────────

    def pick_one_box(self, phase_cb: PhaseCallback, target_color: str = "") -> PickResult:
        """detect → filter → (sweep) → pick → verify 의 한 box 사이클.

        성공 시 박스를 잡고 scan 자세까지 복귀한 상태. cup, target_box 도 유효.
        '컵 외부 박스 없음' 으로 끝나면 no_boxes_outside_cup=True (정상 종료 신호).
        """
        result = PickResult(success=False)
        cfg = self._config

        phase_cb("detecting", "moving to scan pose")
        if not self._move_to_scan():
            result.error = "move to scan pose failed"
            return result

        phase_cb("detecting", "opening gripper")
        if not self._send_gripper(cfg.gripper_open_position):
            result.error = "open gripper failed"
            return result

        max_attempts = cfg.max_grasp_retries + 1
        target_block: Optional[BlockPose] = None
        cup: Optional[BlockPose] = None

        for grasp_attempt in range(1, max_attempts + 1):
            phase_cb(
                "detecting",
                f"grasp attempt {grasp_attempt}/{max_attempts}",
            )

            if grasp_attempt > 1:
                if not self._send_gripper(cfg.gripper_open_position):
                    result.error = "reopen gripper failed"
                    return result
                if not self._move_to_scan():
                    result.error = "re-scan pose move failed"
                    return result

            boxes, cup_local = self._detect_box_cup()
            result.attempts += 1

            if target_color:
                boxes = [b for b in boxes if b.color == target_color]

            # cup 내부에 있는 박스는 잡으러 가지 않는다 (이미 처리된 박스).
            filtered_boxes = self._filter_boxes_outside_cup(boxes, cup_local)

            if not (filtered_boxes and cup_local):
                filtered_boxes, cup_local, result.attempts = self._sweep_for_box_cup(
                    phase_cb, result.attempts, target_color
                )

            if not (filtered_boxes and cup_local):
                # 스윕까지 했는데도 cup 외부에 박스가 없음 → 정상 종료 후보.
                phase_cb("detecting", "no box outside cup, returning home")
                self._move_to_named("home")
                result.no_boxes_outside_cup = True
                result.error = "no box outside cup after sweep"
                return result

            cup = cup_local
            target_block = self._select_closest_box(filtered_boxes)
            if target_block is None:
                result.error = "no valid box detected"
                return result

            self._node.get_logger().info(
                f"Selected box color='{target_block.color}' "
                f"pos=({target_block.grasp_pose.pose.position.x:.3f}, "
                f"{target_block.grasp_pose.pose.position.y:.3f}) "
                f"conf={target_block.confidence:.2f} "
                f"yaw_conf={target_block.yaw_confidence:.2f}"
            )

            # hover
            phase_cb("approaching", "hover above target box")
            hover_pose = self._build_pose(target_block.grasp_pose, cfg.hover_z)
            if not self._send_move_to_pose(hover_pose):
                result.error = "hover motion failed"
                return result
            if cfg.hover_settle_sec > 0.0:
                if not self._sleep_with_cancel(
                    cfg.hover_settle_sec, "hover settle"
                ):
                    result.error = "hover settle canceled"
                    return result

            if cfg.refine_at_hover:
                phase_cb("approaching", "re-measuring target at hover")
                target_block = self._refine_target_at_hover(target_block)

            # descent
            phase_cb("approaching", "descending to grasp")
            grasp_pose = self._build_pose(target_block.grasp_pose, cfg.grasp_z)
            if not self._send_move_to_pose(grasp_pose):
                result.error = "descent motion failed"
                return result

            # yaw align
            phase_cb("aligning", "aligning gripper yaw to box")
            if not self._align_yaw_at_grasp(target_block):
                result.error = "yaw alignment failed"
                return result

            # close gripper
            phase_cb("grasping", "closing gripper")
            if not self._send_gripper(cfg.gripper_closed_position):
                result.error = "close gripper failed"
                return result

            # scan 복귀
            phase_cb("returning", "returning to scan pose")
            if not self._move_to_scan():
                result.error = "return to scan pose failed"
                return result

            # verify
            phase_cb(
                "grasping",
                f"verifying grasp ({grasp_attempt}/{max_attempts})",
            )
            verified, verify_detail = self._verify_grasp()
            if verified:
                self._node.get_logger().info(f"grasp verified: {verify_detail}")
                result.success = True
                result.target_box = target_block
                result.cup = cup
                return result

            self._node.get_logger().warn(
                f"grasp verify failed "
                f"({grasp_attempt}/{max_attempts}): {verify_detail}"
            )
            if self._canceled:
                self._last_action_error = (
                    f"grasp verify canceled: {verify_detail}"
                )
                result.error = "grasp verify canceled"
                return result
            if grasp_attempt >= max_attempts:
                self._last_action_error = verify_detail
                self._move_to_named("home")
                result.error = (
                    f"grasp not detected after {max_attempts} attempts"
                )
                return result

        result.error = "grasp verify loop exited unexpectedly"
        return result

    def place_in_cup(
        self, cup: BlockPose, phase_cb: PhaseCallback,
    ) -> tuple[bool, str]:
        """잡은 박스를 cup 위로 옮겨 낙하 → scan 자세 복귀.

        반환: (success, error_message). 실패 사유는 호출부 result.message 용.
        """
        cfg = self._config

        phase_cb("placing", "moving above cup")
        drop_z = cup.pose.pose.position.z + cfg.drop_clearance_m
        place_pose = self._build_pose(cup.pose, drop_z)
        if not self._send_move_to_pose(place_pose):
            return False, "move above cup failed"

        phase_cb("placing", "releasing box into cup")
        if not self._send_gripper(cfg.gripper_open_position):
            return False, "release gripper failed"

        phase_cb("returning", "returning to scan pose")
        if not self._move_to_scan():
            return False, "return to scan pose failed"

        return True, ""

    def return_home_close(self, phase_cb: PhaseCallback) -> tuple[bool, str]:
        """home 으로 복귀하고 그리퍼 닫음 (절차 마무리)."""
        phase_cb("returning", "returning to home")
        if not self._move_to_named("home"):
            return False, "return to home failed"
        phase_cb("returning", "closing gripper")
        if not self._send_gripper(self._config.gripper_closed_position):
            return False, "post-place close gripper failed"
        return True, ""

    # ────────────────────────────────────────────────────────────────
    # cup-내부 박스 필터
    # ────────────────────────────────────────────────────────────────

    def _filter_boxes_outside_cup(
        self, boxes: list[BlockPose], cup: Optional[BlockPose],
    ) -> list[BlockPose]:
        """cup polygon 내부에 중심이 있는 박스를 제거한다.

        cup 또는 polygon 이 비어 있으면 보수적으로 전부 통과 (잡으러 가도록).
        polygon 이 있으면 박스 중심 XY 로 점-in-polygon 검사.
        """
        if cup is None or not cup.polygon:
            return list(boxes)
        polygon_xy = [(p.x, p.y) for p in cup.polygon]
        outside: list[BlockPose] = []
        for b in boxes:
            bx = b.pose.pose.position.x
            by = b.pose.pose.position.y
            if is_box_in_cup((bx, by), polygon_xy):
                self._node.get_logger().info(
                    f"skip box color='{b.color}' "
                    f"at ({bx:.3f},{by:.3f}) — inside cup polygon"
                )
                continue
            outside.append(b)
        return outside

    # ────────────────────────────────────────────────────────────────
    # detect / sweep / select / refine
    # ────────────────────────────────────────────────────────────────

    def _move_to_scan(self) -> bool:
        positions_rad = [
            math.radians(v) for v in self._config.scan_joint_positions_deg
        ]
        return self._send_move_to_joints(positions_rad)

    def _sweep_for_box_cup(
        self,
        phase_cb: PhaseCallback,
        attempts_so_far: int,
        target_color: str = "",
    ) -> tuple[list[BlockPose], Optional[BlockPose], int]:
        """joint1 절대각 스윕. 매번 cup-필터 적용한 결과로 판단."""
        cfg = self._config
        scan_deg = list(cfg.scan_joint_positions_deg)
        idx = cfg.sweep_joint_index
        attempts = attempts_so_far

        def angles_to_try() -> list[float]:
            seq: list[float] = []
            n_neg = int(abs(cfg.sweep_negative_min_deg) / cfg.sweep_step_deg)
            for i in range(1, n_neg + 1):
                seq.append(-i * cfg.sweep_step_deg)
            seq.append(0.0)
            n_pos = int(cfg.sweep_positive_max_deg / cfg.sweep_step_deg)
            for i in range(1, n_pos + 1):
                seq.append(i * cfg.sweep_step_deg)
            return seq

        for angle_deg in angles_to_try():
            if self._cancel_requested("sweep"):
                return [], None, attempts
            scan_deg[idx] = angle_deg
            phase_cb(
                "detecting",
                f"sweep joint{idx + 1}={angle_deg:+.1f}°",
            )
            positions_rad = [math.radians(v) for v in scan_deg]
            if not self._send_move_to_joints(positions_rad):
                self._node.get_logger().warn(
                    f"sweep motion failed at joint1={angle_deg:+.1f}°, continuing"
                )
                continue
            attempts += 1
            boxes, cup = self._detect_box_cup()
            if target_color:
                boxes = [b for b in boxes if b.color == target_color]
            outside = self._filter_boxes_outside_cup(boxes, cup)
            if outside and cup:
                return outside, cup, attempts
        return [], None, attempts

    def _detect_box_cup(self) -> tuple[list[BlockPose], Optional[BlockPose]]:
        blocks = self._detect_blocks()
        boxes = [b for b in blocks if b.color != CUP_COLOR]
        cups = [b for b in blocks if b.color == CUP_COLOR]
        cup = max(cups, key=lambda b: b.confidence) if cups else None
        return boxes, cup

    def _detect_blocks(self) -> list[BlockPose]:
        if not self._sleep_with_cancel(
            self._config.detect_wait_after_motion_sec,
            "detect settle",
        ):
            return []
        if not self._wait_for_service(
            self._blocks_cli,
            self._config.world_poses_service_name,
        ):
            self._node.get_logger().warn(
                f"{self._config.world_poses_service_name} unavailable (canceled)"
            )
            return []
        request = GetBlockPoses.Request()
        future = self._blocks_cli.call_async(request)
        response = self._wait_future(future, "GetBlockPoses")
        if response is None:
            return []
        return list(response.blocks)

    def _select_closest_box(
        self, boxes: list[BlockPose],
    ) -> Optional[BlockPose]:
        if not boxes:
            return None
        return min(
            boxes,
            key=lambda b: (
                b.grasp_pose.pose.position.x ** 2
                + b.grasp_pose.pose.position.y ** 2
            ),
        )

    def _refine_target_at_hover(
        self, target_block: BlockPose,
    ) -> BlockPose:
        """hover 자세에서 fresh 측정으로 yaw 만 갱신 (silent fallback)."""
        fresh = self._detect_blocks()
        if not fresh:
            self._node.get_logger().warn(
                "hover refine: no blocks detected, keeping initial yaw"
            )
            return target_block

        candidates = [b for b in fresh if b.color == target_block.color]
        if not candidates:
            self._node.get_logger().warn(
                f"hover refine: no '{target_block.color}' block in fresh "
                f"measurement, keeping initial yaw"
            )
            return target_block

        ox = target_block.pose.pose.position.x
        oy = target_block.pose.pose.position.y
        best = min(
            candidates,
            key=lambda b: (
                (b.pose.pose.position.x - ox) ** 2
                + (b.pose.pose.position.y - oy) ** 2
            ),
        )
        dx = best.pose.pose.position.x - ox
        dy = best.pose.pose.position.y - oy
        dist = math.hypot(dx, dy)
        if dist > self._config.refine_match_tolerance_m:
            self._node.get_logger().warn(
                f"hover refine: closest '{target_block.color}' block "
                f"{dist*1000:.0f}mm away "
                f"(> {self._config.refine_match_tolerance_m*1000:.0f}mm), "
                f"keeping initial yaw"
            )
            return target_block

        o_old = target_block.pose.pose.orientation
        o_new = best.pose.pose.orientation
        yaw_old = yaw_from_quaternion(o_old.x, o_old.y, o_old.z, o_old.w)
        yaw_new = yaw_from_quaternion(o_new.x, o_new.y, o_new.z, o_new.w)
        self._node.get_logger().info(
            f"hover refine (yaw only): "
            f"yaw {math.degrees(yaw_old):.1f}°→{math.degrees(yaw_new):.1f}°, "
            f"yaw_conf {target_block.yaw_confidence:.2f}→{best.yaw_confidence:.2f} "
            f"(hover pos shift was {dist*1000:.1f}mm, not applied)"
        )
        refined = BlockPose()
        refined.header = target_block.header
        refined.pose.header = target_block.pose.header
        refined.pose.pose.position = target_block.pose.pose.position
        refined.pose.pose.orientation = o_new
        refined.grasp_pose.header = target_block.grasp_pose.header
        refined.grasp_pose.pose.position = target_block.grasp_pose.pose.position
        refined.grasp_pose.pose.orientation = o_new
        refined.color = target_block.color
        refined.confidence = target_block.confidence
        refined.yaw_confidence = best.yaw_confidence
        # polygon 은 box 에서는 빈 채로 유지된다.
        return refined

    # ────────────────────────────────────────────────────────────────
    # grasp verify
    # ────────────────────────────────────────────────────────────────

    def _verify_grasp(self) -> tuple[bool, str]:
        cfg = self._config
        if cfg.grasp_settle_sec > 0.0:
            if not self._sleep_with_cancel(cfg.grasp_settle_sec, "grasp settle"):
                return False, "canceled during grasp settle"
        if not self._wait_for_service(
            self._check_grasp_cli,
            cfg.check_grasp_service_name,
        ):
            return False, "canceled while waiting for check_grasp service"
        future = self._check_grasp_cli.call_async(CheckGrasp.Request())
        response = self._wait_future(future, "CheckGrasp")
        if response is None:
            return False, "CheckGrasp canceled"
        summary = (
            f"is_grasping={response.is_grasping} "
            f"reason={response.reason} "
            f"current={response.current_ma:.1f}mA"
        )
        return bool(response.is_grasping), summary

    # ────────────────────────────────────────────────────────────────
    # yaw closed-loop align
    # ────────────────────────────────────────────────────────────────

    def _joint5_move_positions(self, joints: dict, j5_target: float) -> list[float]:
        return [
            joints["joint1"], joints["joint2"], joints["joint3"],
            joints["joint4"], j5_target,
        ]

    def _align_yaw_at_grasp(self, target_block: BlockPose) -> bool:
        cfg = self._config

        def _skip_or_fail(reason: str) -> bool:
            if cfg.require_yaw:
                self._last_action_error = f"yaw align: {reason}"
                self._node.get_logger().error(self._last_action_error)
                return False
            self._node.get_logger().warn(f"yaw align skipped: {reason}")
            return True

        if target_block.yaw_confidence < cfg.yaw_min_corner_confidence:
            return _skip_or_fail(
                f"yaw_confidence {target_block.yaw_confidence:.2f} "
                f"< {cfg.yaw_min_corner_confidence:.2f}"
            )

        o = target_block.pose.pose.orientation
        box_yaw = yaw_from_quaternion(o.x, o.y, o.z, o.w)
        box_yaw_norm = wrap_yaw_zero_pi_over_2(box_yaw)

        initial_joint5: Optional[float] = None
        last_delta_world: Optional[float] = None
        last_logged_residual_rad: Optional[float] = None

        for iteration in range(1, cfg.yaw_align_max_iterations + 1):
            joints = self._latest_joint_positions()
            if joints is None:
                return _skip_or_fail("joint states unavailable")

            jaw_yaw = self._lookup_jaw_yaw()
            if jaw_yaw is None:
                return _skip_or_fail("gripper TF unavailable")

            tilt_deg = self._lookup_approach_tilt_deg()
            tilt_str = f"{tilt_deg:.1f}°" if tilt_deg is not None else "n/a"

            gripper_yaw_norm = wrap_yaw_zero_pi_over_2(jaw_yaw)
            delta_world = wrap_to_pm45(box_yaw_norm - gripper_yaw_norm)
            residual_rad = abs(delta_world)

            j5_cur = joints["joint5"]
            if initial_joint5 is None:
                initial_joint5 = j5_cur

            gain_str = "n/a"
            if last_delta_world is not None and abs(last_delta_world) > 1e-9:
                gain = delta_world / last_delta_world
                gain_str = f"{gain:+.3f}"

            self._node.get_logger().info(
                f"yaw align iter {iteration}/{cfg.yaw_align_max_iterations}: "
                f"box_yaw={math.degrees(box_yaw):+.2f}° "
                f"jaw_yaw={math.degrees(jaw_yaw):+.2f}° "
                f"box_norm={math.degrees(box_yaw_norm):.2f}° "
                f"jaw_norm={math.degrees(gripper_yaw_norm):.2f}° "
                f"delta={math.degrees(delta_world):+.2f}° "
                f"residual={math.degrees(residual_rad):.2f}° "
                f"joint5_cur={math.degrees(j5_cur):+.2f}° "
                f"approach_tilt={tilt_str} "
                f"iter_gain={gain_str} "
                f"total_corr={math.degrees(j5_cur - initial_joint5):+.2f}°"
            )
            last_logged_residual_rad = residual_rad

            if residual_rad <= cfg.yaw_align_tolerance_rad:
                self._node.get_logger().info(
                    f"yaw align CONVERGED in {iteration} iter: "
                    f"residual {math.degrees(residual_rad):.2f}° "
                    f"<= tol {math.degrees(cfg.yaw_align_tolerance_rad):.2f}° "
                    f"(total_corr={math.degrees(j5_cur - initial_joint5):+.2f}°)"
                )
                return True

            last_delta_world = delta_world

            j5_tgt = j5_cur + cfg.joint5_yaw_sign * delta_world
            projected_total = abs(j5_tgt - initial_joint5)
            if projected_total > cfg.yaw_align_max_total_correction_rad:
                msg = (
                    f"iter {iteration}: projected total correction "
                    f"{math.degrees(projected_total):.2f}° exceeds cap "
                    f"{math.degrees(cfg.yaw_align_max_total_correction_rad):.2f}°"
                )
                return _skip_or_fail(msg)

            self._node.get_logger().info(
                f"yaw align iter {iteration}: cmd joint5 "
                f"{math.degrees(j5_cur):+.2f}° → {math.degrees(j5_tgt):+.2f}° "
                f"(Δ={math.degrees(j5_tgt - j5_cur):+.2f}°)"
            )

            if not self._send_move_to_joints(
                self._joint5_move_positions(joints, j5_tgt)
            ):
                return False

            if cfg.yaw_align_iter_settle_sec > 0.0:
                if not self._sleep_with_cancel(
                    cfg.yaw_align_iter_settle_sec,
                    f"yaw align iter {iteration} settle",
                ):
                    return False

        final_jaw_yaw = self._lookup_jaw_yaw()
        if final_jaw_yaw is not None:
            final_residual = abs(wrap_to_pm45(
                box_yaw_norm - wrap_yaw_zero_pi_over_2(final_jaw_yaw)
            ))
            self._node.get_logger().warn(
                f"yaw align did NOT converge after "
                f"{cfg.yaw_align_max_iterations} iter: "
                f"final residual {math.degrees(final_residual):.2f}° "
                f"(tol {math.degrees(cfg.yaw_align_tolerance_rad):.2f}°, "
                f"last_pre_move_residual="
                f"{math.degrees(last_logged_residual_rad):.2f}°)"
            )
        else:
            self._node.get_logger().warn(
                f"yaw align did NOT converge after "
                f"{cfg.yaw_align_max_iterations} iter, final TF unavailable"
            )
        return _skip_or_fail(
            f"did not converge after {cfg.yaw_align_max_iterations} iterations"
        )

    # ────────────────────────────────────────────────────────────────
    # Pose / TF / joint helpers
    # ────────────────────────────────────────────────────────────────

    def _build_pose(self, reference: PoseStamped, z: float) -> PoseStamped:
        ref = reference.pose
        return PoseStamped(
            header=Header(frame_id=self._config.approach_frame_id),
            pose=Pose(
                position=Point(x=ref.position.x, y=ref.position.y, z=z),
                orientation=ref.orientation,
            ),
        )

    def _on_joint_state(self, msg: JointState) -> None:
        with self._joint_state_lock:
            self._latest_joint_state = msg

    def _latest_joint_positions(self) -> Optional[dict]:
        with self._joint_state_lock:
            js = self._latest_joint_state
        if js is None:
            return None
        names = list(js.name)
        positions = list(js.position)
        out: dict[str, float] = {}
        for jn in self._config.arm_joint_names:
            if jn not in names:
                return None
            idx = names.index(jn)
            if idx >= len(positions):
                return None
            out[jn] = float(positions[idx])
        return out

    def _lookup_jaw_yaw(self) -> Optional[float]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._config.world_frame,
                self._config.gripper_link,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as exc:
            self._node.get_logger().warn(f"gripper TF lookup failed: {exc}")
            return None
        q = tf.transform.rotation
        return jaw_axis_yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def _lookup_approach_tilt_deg(self) -> Optional[float]:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._config.world_frame,
                self._config.gripper_link,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as exc:
            self._node.get_logger().debug(f"approach tilt TF lookup failed: {exc}")
            return None
        q = tf.transform.rotation
        ax_z = 2.0 * (q.x * q.z - q.y * q.w)
        cos_tilt = max(-1.0, min(1.0, -ax_z))
        return math.degrees(math.acos(cos_tilt))

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
        self._last_action_error = ""
        if self._cancel_requested(label):
            return False
        if not self._wait_for_action_server(
            client,
            self._config.action_goal_timeout_sec,
            label,
        ):
            if self._canceled:
                return False
            self._last_action_error = f"{label}: action server unavailable"
            self._node.get_logger().error(self._last_action_error)
            return False
        send_future = client.send_goal_async(goal)
        gh = self._wait_future(
            send_future, label,
            timeout_sec=self._config.action_goal_timeout_sec,
        )
        if gh is None:
            if not self._canceled:
                self._last_action_error = f"{label}: send_goal timed out"
                self._node.get_logger().error(self._last_action_error)
            return False
        if not gh.accepted:
            self._last_action_error = f"{label}: goal rejected"
            self._node.get_logger().error(self._last_action_error)
            return False
        result_future = gh.get_result_async()
        wrapped = self._wait_for_result(result_future, gh, label)
        if wrapped is None:
            return False
        result = wrapped.result
        success = bool(getattr(result, "success", False))
        if not success:
            msg = getattr(result, "message", "")
            self._last_action_error = f"{label} failed: {msg}"
            self._node.get_logger().error(self._last_action_error)
        return success

    def _wait_for_action_server(
        self,
        client: ActionClient,
        timeout_sec: float,
        label: str,
    ) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok():
            if self._cancel_requested(label):
                return False
            if client.wait_for_server(timeout_sec=0.05):
                return True
            if time.monotonic() > deadline:
                return False
        return False

    def _wait_for_service(self, client, label: str) -> bool:
        deadline = time.monotonic() + max(
            0.0, self._config.service_call_timeout_sec
        )
        while rclpy.ok():
            if self._cancel_requested(label):
                return False
            if client.wait_for_service(timeout_sec=0.05):
                return True
            if time.monotonic() > deadline:
                self._node.get_logger().warn(
                    f"{label}: service wait timeout "
                    f"(>{self._config.service_call_timeout_sec:.1f}s), "
                    f"proceeding to next step"
                )
                return False
        return False

    def _wait_for_result(self, future, sub_goal_handle, label: str):
        deadline = time.monotonic() + max(
            0.0, self._config.action_result_timeout_sec
        )
        while rclpy.ok() and not future.done():
            if self._cancel_requested(label):
                sub_goal_handle.cancel_goal_async()
                return None
            if time.monotonic() > deadline:
                self._last_action_error = f"{label}: result timed out"
                self._node.get_logger().error(self._last_action_error)
                sub_goal_handle.cancel_goal_async()
                return None
            time.sleep(0.02)
        if not future.done():
            return None
        return future.result()

    def _wait_future(
        self, future, label: str = "", timeout_sec: Optional[float] = None,
    ):
        effective_timeout = (
            self._config.service_call_timeout_sec
            if timeout_sec is None else timeout_sec
        )
        deadline = time.monotonic() + max(0.0, effective_timeout)
        while rclpy.ok() and not future.done():
            if label and self._cancel_requested(label):
                return None
            if time.monotonic() > deadline:
                self._node.get_logger().warn(
                    f"{label}: response timeout "
                    f"(>{effective_timeout:.1f}s), proceeding to next step"
                )
                return None
            time.sleep(0.02)
        if not future.done():
            return None
        return future.result()

    def _sleep_with_cancel(self, duration_sec: float, label: str) -> bool:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            if self._cancel_requested(label):
                return False
            time.sleep(min(0.02, deadline - time.monotonic()))
        return not self._cancel_requested(label)

    def _cancel_requested(self, label: str) -> bool:
        gh = self._active_goal_handle
        if gh is not None and gh.is_cancel_requested:
            if not self._canceled:
                self._last_action_error = f"{label}: canceled by client"
                self._node.get_logger().warn(self._last_action_error)
            self._canceled = True
            return True
        if self._canceled:
            if not self._last_action_error:
                self._last_action_error = f"{label}: canceled by client"
                self._node.get_logger().warn(self._last_action_error)
            self._canceled = True
            return True
        return False


def build_worker_config_from_node(node: Node) -> WorkerConfig:
    """Node 의 ROS 파라미터에서 WorkerConfig 를 채워 반환한다.

    PickPlace / PickPlaceAll 서버가 동일 파라미터 스키마를 공유하므로
    공통 로더를 제공한다. (서버별 추가 파라미터는 서버에서 따로 declare)
    """
    p = node.declare_parameter
    config = WorkerConfig(
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
        service_call_timeout_sec=float(
            p("service_call_timeout_sec", 15.0).value
        ),
        world_poses_service_name=str(
            p("world_poses_service_name",
              "perception/get_box_cup_world_poses").value
        ),
        approach_frame_id=str(p("approach_frame_id", "world").value),
        hover_z=float(p("hover_z", 0.15).value),
        grasp_z=float(p("grasp_z", 0.055).value),
        move_velocity_scale=float(p("move_velocity_scale", 0.2).value),
        hover_settle_sec=float(p("hover_settle_sec", 0.5).value),
        refine_at_hover=bool(p("refine_at_hover", True).value),
        refine_match_tolerance_m=float(
            p("refine_match_tolerance_m", 0.05).value
        ),
        drop_clearance_m=float(p("drop_clearance_m", 0.10).value),
        world_frame=str(p("world_frame", "world").value),
        gripper_link=str(p("gripper_link", "end_effector_link").value),
        joint_states_topic=str(p("joint_states_topic", "joint_states").value),
        yaw_min_corner_confidence=float(
            p("yaw_min_corner_confidence", 0.30).value
        ),
        joint5_yaw_sign=float(p("joint5_yaw_sign", 1.0).value),
        require_yaw=bool(p("require_yaw", False).value),
        gripper_open_position=float(p("gripper_open_position", 1.0).value),
        gripper_closed_position=float(p("gripper_closed_position", 0.1).value),
        gripper_max_effort=float(p("gripper_max_effort", 0.0).value),
        action_goal_timeout_sec=float(p("action_goal_timeout_sec", 5.0).value),
        action_result_timeout_sec=float(
            p("action_result_timeout_sec", 60.0).value
        ),
        max_grasp_retries=int(p("max_grasp_retries", 2).value),
        check_grasp_service_name=str(
            p("check_grasp_service_name", "gripper/check_grasp").value
        ),
        grasp_settle_sec=float(p("grasp_settle_sec", 0.3).value),
        yaw_align_max_iterations=int(
            p("yaw_align_max_iterations", 4).value
        ),
        yaw_align_tolerance_rad=math.radians(float(
            p("yaw_align_tolerance_deg", 0.5).value
        )),
        yaw_align_max_total_correction_rad=math.radians(float(
            p("yaw_align_max_total_correction_deg", 60.0).value
        )),
        yaw_align_iter_settle_sec=float(
            p("yaw_align_iter_settle_sec", 0.1).value
        ),
    )
    if config.sweep_step_deg <= 0.0:
        raise ValueError("sweep_step_deg must be positive")
    if config.max_grasp_retries < 0:
        raise ValueError("max_grasp_retries must be >= 0")
    if config.yaw_align_max_iterations < 1:
        raise ValueError("yaw_align_max_iterations must be >= 1")
    if config.yaw_align_tolerance_rad <= 0.0:
        raise ValueError("yaw_align_tolerance_deg must be positive")
    if config.yaw_align_max_total_correction_rad <= 0.0:
        raise ValueError("yaw_align_max_total_correction_deg must be positive")
    return config
