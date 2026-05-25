"""PickPlace 스킬 서버 — box 를 종이컵에 넣는다.

0. 스킬 goal 수신
1. 스캔 포즈로 이동 (MoveToJoints)
2. 그리퍼 열기 (GripperCommand open)
3. ros2 service call /perception/get_box_cup_world_poses omx_interfaces/srv/GetBlockPoses "{}" 로 box+cup 탐지.
   box 와 cup 이 동시에 잡힐 때까지 joint1 절대각 스윕.
   끝까지 실패면 home 복귀 후 실패 반환.
4. 탐지된 box 중 로봇 베이스에서 가장 가까운 box 선택.
5. 선택 box 위 hover 로 이동 (MoveToPose, position only).
6. grasp_z 로 하강 (MoveToPose, position only).
7. grasp 위치에서 yaw 정렬: TF 로 현재 그리퍼 yaw 조회 → joint5 회전 명령.
   box 의 yaw_confidence 가 낮으면 생략. 회전량이 한계를 넘으면 실패.
8. 그리퍼 닫기 → 스캔 포즈 복귀.
9. /gripper/check_grasp 서비스로 grasp 검증.
   실패 시 그리퍼 열고 단계 3 부터 재진입 (최대 max_grasp_retries 회).
   max_grasp_retries 까지 실패하면 home 복귀 후 실패 반환.
10. 저장한 cup 위치 위 (cup_z + drop_clearance) 로 이동 → 그리퍼 열기 (낙하).
11. 스캔 포즈 복귀.
12. home 복귀 → 그리퍼 닫기.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
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
    PickPlace,
)
from omx_interfaces.msg import BlockPose
from omx_interfaces.srv import CheckGrasp, GetBlockPoses

from omx_skill_executor.pick_place_geometry import (
    jaw_axis_yaw_from_quaternion,
    joint5_target,
    yaw_from_quaternion,
)

BOX_COLOR = "box"
CUP_COLOR = "cup"


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
    world_poses_service_name: str
    approach_frame_id: str
    hover_z: float
    grasp_z: float
    move_velocity_scale: float
    hover_settle_sec: float
    drop_clearance_m: float
    world_frame: str
    gripper_link: str
    joint_states_topic: str
    yaw_min_corner_confidence: float
    joint5_yaw_sign: float
    joint5_correction_max_rad: float
    require_yaw: bool
    gripper_open_position: float
    gripper_closed_position: float
    gripper_max_effort: float
    action_goal_timeout_sec: float
    action_result_timeout_sec: float
    max_grasp_retries: int
    check_grasp_service_name: str
    check_grasp_timeout_sec: float
    grasp_settle_sec: float


class PickPlaceServer(Node):
    def __init__(self) -> None:
        super().__init__("pick_place_server")
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
            GetBlockPoses, self._config.world_poses_service_name,
            callback_group=self._cb_group,
        )
        self._check_grasp_cli = self.create_client(
            CheckGrasp, self._config.check_grasp_service_name,
            callback_group=self._cb_group,
        )

        # joint state / TF
        self._joint_state_lock = threading.Lock()
        self._latest_joint_state: Optional[JointState] = None
        self.create_subscription(
            JointState, self._config.joint_states_topic,
            self._on_joint_state, 10, callback_group=self._cb_group,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 서버
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
        self._last_action_error = ""
        self._canceled = False
        self._active_goal_handle: Optional[ServerGoalHandle] = None
        self.get_logger().info("PickPlaceServer ready")

    # ────────────────────────────────────────────────────────────────
    # Config
    # ────────────────────────────────────────────────────────────────

    def _load_config(self) -> SkillConfig:
        p = self.declare_parameter
        config = SkillConfig(
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
            world_poses_service_name=str(
                p("world_poses_service_name",
                  "/perception/get_box_cup_world_poses").value
            ),
            approach_frame_id=str(p("approach_frame_id", "world").value),
            hover_z=float(p("hover_z", 0.15).value),
            grasp_z=float(p("grasp_z", 0.055).value),
            move_velocity_scale=float(p("move_velocity_scale", 0.2).value),
            hover_settle_sec=float(p("hover_settle_sec", 0.5).value),
            drop_clearance_m=float(p("drop_clearance_m", 0.10).value),
            world_frame=str(p("world_frame", "world").value),
            gripper_link=str(p("gripper_link", "end_effector_link").value),
            joint_states_topic=str(p("joint_states_topic", "/joint_states").value),
            yaw_min_corner_confidence=float(
                p("yaw_min_corner_confidence", 0.30).value
            ),
            joint5_yaw_sign=float(p("joint5_yaw_sign", 1.0).value),
            joint5_correction_max_rad=float(
                p("joint5_correction_max_rad", 0.2).value
            ),
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
                p("check_grasp_service_name", "/gripper/check_grasp").value
            ),
            check_grasp_timeout_sec=float(
                p("check_grasp_timeout_sec", 2.0).value
            ),
            grasp_settle_sec=float(p("grasp_settle_sec", 0.3).value),
        )
        if config.sweep_step_deg <= 0.0:
            raise ValueError("sweep_step_deg must be positive")
        if config.max_grasp_retries < 0:
            raise ValueError("max_grasp_retries must be >= 0")
        return config

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
        self._canceled = True
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle: ServerGoalHandle) -> PickPlace.Result:
        try:
            return self._run(goal_handle)
        finally:
            self._active_goal_handle = None
            if self._busy_lock.locked():
                self._busy_lock.release()

    def _run(self, goal_handle: ServerGoalHandle) -> PickPlace.Result:
        goal = goal_handle.request
        result = PickPlace.Result(success=False, message="", attempts=0)
        self._canceled = False
        self._active_goal_handle = goal_handle

        self._publish_phase(goal_handle, "detecting", "moving to scan pose")
        if not self._move_to_scan():
            return self._fail(goal_handle, result, "move to scan pose failed")

        self._publish_phase(goal_handle, "detecting", "opening gripper")
        if not self._send_gripper(self._config.gripper_open_position):
            return self._fail(goal_handle, result, "open gripper failed")

        # detect → approach → grasp → verify 의 한 attempt 를 max_grasp_retries+1
        # 까지 반복한다. verify 실패만 다음 attempt 로 넘어가고, 그 외 motion
        # 실패는 즉시 fail (재시도가 같은 실패를 반복할 가능성이 크다).
        max_attempts = self._config.max_grasp_retries + 1
        target_block: Optional[BlockPose] = None
        cup: Optional[BlockPose] = None

        for grasp_attempt in range(1, max_attempts + 1):
            self._publish_phase(
                goal_handle, "detecting",
                f"grasp attempt {grasp_attempt}/{max_attempts}",
            )

            # 두 번째 이후 attempt 는 그리퍼 재-open + scan 자세 보장.
            if grasp_attempt > 1:
                if not self._send_gripper(self._config.gripper_open_position):
                    return self._fail(goal_handle, result, "reopen gripper failed")
                if not self._move_to_scan():
                    return self._fail(
                        goal_handle, result, "re-scan pose move failed"
                    )

            boxes, cup = self._detect_box_cup()
            result.attempts += 1

            if not (boxes and cup):
                boxes, cup, result.attempts = self._sweep_for_box_cup(
                    goal_handle, result.attempts
                )

            if not (boxes and cup):
                self._publish_phase(
                    goal_handle, "detecting", "box+cup not found, returning home"
                )
                self._move_to_named("home")
                return self._fail(
                    goal_handle, result,
                    "box and cup not both detected after sweep"
                )

            target_block = self._select_closest_box(boxes)
            if target_block is None:
                return self._fail(goal_handle, result, "no valid box detected")

            self.get_logger().info(
                f"Selected box color='{target_block.color}' "
                f"pos=({target_block.grasp_pose.pose.position.x:.3f}, "
                f"{target_block.grasp_pose.pose.position.y:.3f}) "
                f"conf={target_block.confidence:.2f} "
                f"yaw_conf={target_block.yaw_confidence:.2f}"
            )

            # hover
            self._publish_phase(
                goal_handle, "approaching", "hover above target box"
            )
            hover_pose = self._build_pose(
                target_block.grasp_pose, self._config.hover_z
            )
            if not self._send_move_to_pose(hover_pose):
                return self._fail(goal_handle, result, "hover motion failed")
            if self._config.hover_settle_sec > 0.0:
                if not self._sleep_with_cancel(
                    self._config.hover_settle_sec,
                    "hover settle",
                ):
                    return self._fail(
                        goal_handle, result, "hover settle canceled"
                    )

            # descent (yaw 정렬은 grasp 위치 도달 후에 수행한다)
            self._publish_phase(
                goal_handle, "approaching", "descending to grasp"
            )
            grasp_pose = self._build_pose(
                target_block.grasp_pose, self._config.grasp_z
            )
            if not self._send_move_to_pose(grasp_pose):
                return self._fail(goal_handle, result, "descent motion failed")

            # grasp 위치에서 joint5 yaw 정렬
            self._publish_phase(
                goal_handle, "aligning", "aligning gripper yaw to box"
            )
            if not self._align_yaw_at_grasp(target_block):
                return self._fail(goal_handle, result, "yaw alignment failed")

            # close gripper
            self._publish_phase(goal_handle, "grasping", "closing gripper")
            if not self._send_gripper(self._config.gripper_closed_position):
                return self._fail(goal_handle, result, "close gripper failed")

            # 스캔 포즈 복귀
            self._publish_phase(
                goal_handle, "returning", "returning to scan pose"
            )
            if not self._move_to_scan():
                return self._fail(
                    goal_handle, result, "return to scan pose failed"
                )

            # grasp 검증 — scan 자세에서 /gripper/check_grasp 평가.
            self._publish_phase(
                goal_handle, "grasping",
                f"verifying grasp ({grasp_attempt}/{max_attempts})",
            )
            verified, verify_detail = self._verify_grasp()
            if verified:
                self.get_logger().info(f"grasp verified: {verify_detail}")
                break
            self.get_logger().warn(
                f"grasp verify failed "
                f"({grasp_attempt}/{max_attempts}): {verify_detail}"
            )
            if self._canceled:
                self._last_action_error = (
                    f"grasp verify canceled: {verify_detail}"
                )
                return self._fail(goal_handle, result, "grasp verify canceled")
            if grasp_attempt >= max_attempts:
                self._last_action_error = verify_detail
                self._move_to_named("home")
                return self._fail(
                    goal_handle, result,
                    f"grasp not detected after {max_attempts} attempts"
                )
            # 다음 attempt 로
        else:
            # for-else: break 없이 루프 종료 = 모든 attempt 실패. 위의 cap
            # 분기에서 이미 fail return 되므로 도달하지 않지만 방어적으로
            # 처리한다.
            return self._fail(
                goal_handle, result, "grasp verify loop exited unexpectedly"
            )

        assert target_block is not None and cup is not None

        # cup 위로 이동
        self._publish_phase(goal_handle, "placing", "moving above cup")
        drop_z = cup.pose.pose.position.z + self._config.drop_clearance_m
        place_pose = self._build_pose(cup.pose, drop_z)
        if not self._send_move_to_pose(place_pose):
            return self._fail(goal_handle, result, "move above cup failed")

        # 낙하 (release)
        self._publish_phase(goal_handle, "placing", "releasing box into cup")
        if not self._send_gripper(self._config.gripper_open_position):
            return self._fail(goal_handle, result, "release gripper failed")

        # 스캔 포즈 복귀
        self._publish_phase(goal_handle, "returning", "returning to scan pose")
        if not self._move_to_scan():
            return self._fail(goal_handle, result, "return to scan pose failed")

        # home 복귀 + 그리퍼 닫기
        self._publish_phase(goal_handle, "returning", "returning to home")
        if not self._move_to_named("home"):
            return self._fail(goal_handle, result, "return to home failed")
        self._publish_phase(goal_handle, "returning", "closing gripper")
        if not self._send_gripper(self._config.gripper_closed_position):
            return self._fail(goal_handle, result, "post-place close gripper failed")

        result.success = True
        result.message = f"placed '{target_block.color}' box into cup"
        goal_handle.succeed()
        self.get_logger().info(result.message)
        return result

    # ────────────────────────────────────────────────────────────────
    # High-level steps
    # ────────────────────────────────────────────────────────────────

    def _move_to_scan(self) -> bool:
        positions_rad = [math.radians(v) for v in self._config.scan_joint_positions_deg]
        return self._send_move_to_joints(positions_rad)

    def _sweep_for_box_cup(
        self,
        goal_handle: ServerGoalHandle,
        attempts_so_far: int,
    ) -> tuple[list[BlockPose], Optional[BlockPose], int]:
        """joint1 절대각 스윕. box 와 cup 동시 인식 시 즉시 반환."""
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
            boxes, cup = self._detect_box_cup()
            if boxes and cup:
                return boxes, cup, attempts
        return [], None, attempts

    def _verify_grasp(self) -> tuple[bool, str]:
        """scan 자세에서 /gripper/check_grasp 로 grasp 여부를 확인한다.

        반환: (성공 여부, 사유/현재값 요약 문자열).
        cancel 이면 (False, "...canceled...") 로 반환하고 self._canceled 가
        True 인지로 상위가 구분한다.
        """
        cfg = self._config
        if cfg.grasp_settle_sec > 0.0:
            if not self._sleep_with_cancel(cfg.grasp_settle_sec, "grasp settle"):
                return False, "canceled during grasp settle"
        if not self._wait_for_service(
            self._check_grasp_cli,
            cfg.check_grasp_timeout_sec,
            cfg.check_grasp_service_name,
        ):
            if self._canceled:
                return False, "canceled while waiting for check_grasp service"
            return False, f"{cfg.check_grasp_service_name} unavailable"
        future = self._check_grasp_cli.call_async(CheckGrasp.Request())
        response = self._wait_future(
            future, cfg.check_grasp_timeout_sec, "CheckGrasp"
        )
        if response is None:
            if self._canceled:
                return False, "CheckGrasp canceled"
            return False, "CheckGrasp timed out"
        summary = (
            f"is_grasping={response.is_grasping} "
            f"reason={response.reason} "
            f"current={response.current_ma:.1f}mA"
        )
        return bool(response.is_grasping), summary

    def _detect_box_cup(self) -> tuple[list[BlockPose], Optional[BlockPose]]:
        blocks = self._detect_blocks()
        boxes = [b for b in blocks if b.color == BOX_COLOR]
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
            self._config.get_block_poses_timeout_sec,
            self._config.world_poses_service_name,
        ):
            self.get_logger().warn(
                f"{self._config.world_poses_service_name} service unavailable"
            )
            return []
        request = GetBlockPoses.Request()
        future = self._blocks_cli.call_async(request)
        response = self._wait_future(
            future,
            self._config.get_block_poses_timeout_sec,
            "GetBlockPoses",
        )
        if response is None:
            self.get_logger().warn("GetBlockPoses timed out")
            return []
        return list(response.blocks)

    def _select_closest_box(self, boxes: list[BlockPose]) -> Optional[BlockPose]:
        """감지된 박스 중 로봇 베이스(원점)에서 가장 가까운 박스를 반환한다."""
        if not boxes:
            return None
        return min(
            boxes,
            key=lambda b: (
                b.grasp_pose.pose.position.x ** 2
                + b.grasp_pose.pose.position.y ** 2
            ),
        )

    def _plan_joint5_alignment(
        self, target_block: BlockPose, label: str
    ) -> Optional[tuple[dict, float]]:
        """현재 joints/TF 로 joint5 yaw 정렬 목표각을 계산한다.

        실패 시 self._last_action_error 에 사유를 적고 None 을 반환한다.
        성공 시 (joints, joint5_target_rad) 을 반환한다.
        """
        joints = self._latest_joint_positions()
        if joints is None:
            self._last_action_error = "joint states unavailable"
            return None

        jaw_yaw = self._lookup_jaw_yaw()
        if jaw_yaw is None:
            self._last_action_error = "gripper TF unavailable"
            return None

        o = target_block.pose.pose.orientation
        box_yaw = yaw_from_quaternion(o.x, o.y, o.z, o.w)
        j5_cur = joints["joint5"]
        j5_tgt = joint5_target(
            j5_cur, box_yaw, jaw_yaw, self._config.joint5_yaw_sign
        )
        self.get_logger().info(
            f"yaw align ({label}): box_yaw={math.degrees(box_yaw):.1f}° "
            f"jaw_yaw={math.degrees(jaw_yaw):.1f}° "
            f"joint5 {math.degrees(j5_cur):.1f}°→{math.degrees(j5_tgt):.1f}°"
        )
        return joints, j5_tgt

    def _joint5_move_positions(self, joints: dict, j5_target: float) -> list[float]:
        """joints dict 에서 joint5 만 j5_target 으로 바꾼 arm 위치 리스트."""
        return [
            joints["joint1"], joints["joint2"], joints["joint3"],
            joints["joint4"], j5_target,
        ]

    def _align_yaw_at_grasp(self, target_block: BlockPose) -> bool:
        """grasp 높이에 도달한 뒤 box yaw 에 맞춰 joint5 를 회전한다.

        hover 단계에서는 정렬하지 않고 descent 후 한 번에 정렬한다. 이유:
        하강은 position-only IK 라 hover 에서 미리 맞춰둔 joint5 가 틀어져
        잔차 보정이 필요했는데, grasp 위치에서 단번에 정렬하면 그 두-단계
        오차를 없앨 수 있다.

        yaw_confidence 가 낮거나 joint/ TF 미가용 시: require_yaw 가 False
        면 정렬을 생략하고 True 반환, True 면 실패 반환. 회전 명령량이
        joint5_correction_max_rad 를 넘으면 grasp 높이에서의 박스 충돌
        위험으로 보고 실패 처리한다.
        """
        cfg = self._config

        def _skip_or_fail(reason: str) -> bool:
            if cfg.require_yaw:
                self._last_action_error = f"yaw align: {reason}"
                self.get_logger().error(self._last_action_error)
                return False
            self.get_logger().warn(f"yaw align skipped: {reason}")
            return True

        if target_block.yaw_confidence < cfg.yaw_min_corner_confidence:
            return _skip_or_fail(
                f"yaw_confidence {target_block.yaw_confidence:.2f} "
                f"< {cfg.yaw_min_corner_confidence:.2f}"
            )

        plan = self._plan_joint5_alignment(target_block, "grasp")
        if plan is None:
            return _skip_or_fail(self._last_action_error)
        joints, j5_target = plan

        correction = abs(j5_target - joints["joint5"])
        if correction > cfg.joint5_correction_max_rad:
            self._last_action_error = (
                f"joint5 rotation {math.degrees(correction):.1f}° exceeds "
                f"limit {math.degrees(cfg.joint5_correction_max_rad):.1f}°"
            )
            self.get_logger().error(self._last_action_error)
            return False

        # 회전량이 무시할 수준이면 불필요한 재계획/이동을 생략한다.
        if correction < math.radians(1.0):
            return True
        return self._send_move_to_joints(
            self._joint5_move_positions(joints, j5_target)
        )

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
    # joint state / TF
    # ────────────────────────────────────────────────────────────────

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
        """그리퍼 jaw 폐합축(end_effector_link +y축)의 world heading(rad)."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._config.world_frame,
                self._config.gripper_link,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as exc:
            self.get_logger().warn(f"gripper TF lookup failed: {exc}")
            return None
        q = tf.transform.rotation
        return jaw_axis_yaw_from_quaternion(q.x, q.y, q.z, q.w)

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
            self.get_logger().error(self._last_action_error)
            return False
        send_future = client.send_goal_async(goal)
        gh = self._wait_future(send_future, self._config.action_goal_timeout_sec, label)
        if gh is None:
            if not self._canceled:
                self._last_action_error = f"{label}: send_goal timed out"
                self.get_logger().error(self._last_action_error)
            return False
        if not gh.accepted:
            self._last_action_error = f"{label}: goal rejected"
            self.get_logger().error(self._last_action_error)
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
            self.get_logger().error(self._last_action_error)
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

    def _wait_for_service(self, client, timeout_sec: float, label: str) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok():
            if self._cancel_requested(label):
                return False
            if client.wait_for_service(timeout_sec=0.05):
                return True
            if time.monotonic() > deadline:
                return False
        return False

    def _wait_for_result(self, future, sub_goal_handle, label: str):
        """하위 action 결과를 폴링 대기한다.

        상위 PickPlace goal 에 cancel 이 요청되거나 result timeout 이 발생하면
        진행 중인 하위 goal 을 cancel 한 뒤 None 을 반환해 호출부가 실패/취소
        경로를 타게 한다.
        """
        deadline = time.monotonic() + max(0.0, self._config.action_result_timeout_sec)
        while rclpy.ok() and not future.done():
            if self._cancel_requested(label):
                sub_goal_handle.cancel_goal_async()
                return None
            if time.monotonic() > deadline:
                self._last_action_error = f"{label}: result timed out"
                self.get_logger().error(self._last_action_error)
                sub_goal_handle.cancel_goal_async()
                return None
            time.sleep(0.02)
        if not future.done():
            return None
        return future.result()

    def _wait_future(self, future, timeout_sec: float, label: str = ""):
        """외부 MultiThreadedExecutor 가 spin 중이라는 가정하에 폴링 대기."""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while rclpy.ok() and not future.done():
            if label and self._cancel_requested(label):
                return None
            if time.monotonic() > deadline:
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
                self.get_logger().warn(self._last_action_error)
            self._canceled = True
            return True
        if self._canceled:
            if not self._last_action_error:
                self._last_action_error = f"{label}: canceled by client"
                self.get_logger().warn(self._last_action_error)
            self._canceled = True
            return True
        return False

    # ────────────────────────────────────────────────────────────────
    # Helpers
    # ────────────────────────────────────────────────────────────────

    def _publish_phase(
        self,
        goal_handle: ServerGoalHandle,
        phase: str,
        status: str,
    ) -> None:
        fb = PickPlace.Feedback(phase=phase, status=status)
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f"[{phase}] {status}")

    def _fail(
        self,
        goal_handle: ServerGoalHandle,
        result: PickPlace.Result,
        message: str,
    ) -> PickPlace.Result:
        result.success = False
        detail = self._last_action_error
        result.message = f"{message} | {detail}" if detail else message
        if self._canceled:
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
