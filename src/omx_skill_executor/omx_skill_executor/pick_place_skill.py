"""omx_skill_executor — PickPlaceSkill

Action server  : /omx/pick_place   (omx_interfaces/action/PickPlace)
Service client : /omx/get_block_poses   (omx_interfaces/srv/GetBlockPoses)
Action clients :
  /omx/move_to_named    (omx_interfaces/action/MoveToNamed)
  /omx/move_to_pose     (omx_interfaces/action/MoveToPose)
  /omx/gripper_command  (omx_interfaces/action/GripperCommand)

Execution phases:
  detecting   — query omx_perception; filter by color; pick highest-confidence block
  approaching — move to pre-grasp pose (above block)
  grasping    — open gripper → descend → close → lift
  placing     — transport to target box → descend → release → return to ready

Recovery policy:
  Detection failure  → retry up to max_retries, then abort
  Any motion failure → safe_stop (home + open gripper), then abort

Prerequisites:
  A TF transform from the camera frame (block_detection_node "camera_frame"
  param, default "default_cam") to the planning frame must exist.
  Publish one with a static_transform_publisher or a calibration node.

Key parameters (see config/skill_executor.yaml):
  planning_frame        : MoveIt2 planning frame (default "world")
  approach_orientation  : end-effector quaternion for top-down grasp (tune per robot)
  left_box / right_box  : box positions in planning frame (meters)
  grasp_z_offset        : offset added to detected table point to reach grasp height
  pre_grasp_z_offset    : height above block for pre-grasp approach
  lift_z_offset         : height above grasp point after closing gripper
  place_z_offset        : height above box for pre-place approach
  velocity_scale        : normal motion speed (0.0–1.0)
"""

import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import geometry_msgs.msg as geo_msgs
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support

from omx_interfaces.action import GripperCommand, MoveToNamed, MoveToPose, PickPlace
from omx_interfaces.srv import GetBlockPoses

# ── Constants ─────────────────────────────────────────────────────────────────
_DETECT_TIMEOUT_SEC   = 5.0
_MOTION_TIMEOUT_SEC   = 30.0
_GRIPPER_TIMEOUT_SEC  = 10.0
_MAX_DETECT_RETRIES   = 3
_POST_CLOSE_WAIT_SEC  = 0.5   # settle time after gripper close

_GRIPPER_OPEN  = 1.0
_GRIPPER_CLOSE = 0.0


class PickPlaceSkill(Node):
    """Rule-based PickPlace action server (5단계: omx_skill_executor)."""

    def __init__(self) -> None:
        super().__init__("pick_place_skill")
        cbg = ReentrantCallbackGroup()

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("planning_frame", "world")

        # Box positions in planning frame (meters)
        self.declare_parameter("left_box.x",  0.20)
        self.declare_parameter("left_box.y",  0.20)
        self.declare_parameter("left_box.z",  0.05)
        self.declare_parameter("right_box.x", 0.20)
        self.declare_parameter("right_box.y", -0.20)
        self.declare_parameter("right_box.z", 0.05)

        # Geometry offsets
        self.declare_parameter("grasp_z_offset",     0.0)
        self.declare_parameter("pre_grasp_z_offset", 0.10)
        self.declare_parameter("lift_z_offset",      0.15)
        self.declare_parameter("place_z_offset",     0.08)

        # End-effector orientation for grasps (tune for your robot)
        # identity (w=1) = EEF frame aligned with world frame
        # Adjust x/y/z/w so the gripper faces downward toward the table.
        self.declare_parameter("approach_orientation.x", 0.0)
        self.declare_parameter("approach_orientation.y", 0.0)
        self.declare_parameter("approach_orientation.z", 0.0)
        self.declare_parameter("approach_orientation.w", 1.0)

        # Motion velocity (0.0–1.0)
        self.declare_parameter("velocity_scale", 0.3)

        self._planning_frame: str = self.get_parameter("planning_frame").value

        # ── TF2 ───────────────────────────────────────────────────────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Service client ─────────────────────────────────────────────
        self._get_block_poses = self.create_client(
            GetBlockPoses, "/omx/get_block_poses",
            callback_group=cbg,
        )

        # ── Action clients ─────────────────────────────────────────────
        self._move_to_named_client = ActionClient(
            self, MoveToNamed, "/omx/move_to_named",
            callback_group=cbg,
        )
        self._move_to_pose_client = ActionClient(
            self, MoveToPose, "/omx/move_to_pose",
            callback_group=cbg,
        )
        self._gripper_client = ActionClient(
            self, GripperCommand, "/omx/gripper_command",
            callback_group=cbg,
        )

        # ── Action server ──────────────────────────────────────────────
        self._server = ActionServer(
            self, PickPlace, "/omx/pick_place",
            execute_callback=self._execute_cb,
            callback_group=cbg,
        )

        self.get_logger().info(
            "PickPlaceSkill ready.\n"
            "  Action : /omx/pick_place\n"
            "  Calls  : /omx/get_block_poses | /omx/move_to_named"
            " | /omx/move_to_pose | /omx/gripper_command"
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Blocking call helpers
    # Uses threading.Event so the MultiThreadedExecutor can process response
    # callbacks in other threads while this thread waits.
    # ─────────────────────────────────────────────────────────────────────────

    def _call_service(self, client, request, timeout_sec: float = 5.0):
        """Send a service request and block until response. Returns response or None."""
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service /omx/get_block_poses unavailable")
            return None

        done = threading.Event()
        holder: list = [None]

        def _on_response(future):
            holder[0] = future.result()
            done.set()

        client.call_async(request).add_done_callback(_on_response)

        if not done.wait(timeout=timeout_sec):
            self.get_logger().error(
                f"Service call timed out after {timeout_sec:.1f}s"
            )
        return holder[0]

    def _call_action(self, client, goal_msg, timeout_sec: float = 30.0):
        """Send an action goal and block until result.

        Returns (accepted: bool, wrapped_result) or (False, None) on failure.
        """
        done     = threading.Event()
        accepted: list = [None]
        result:   list = [None]

        def _on_goal_response(future):
            handle = future.result()
            if not handle.accepted:
                accepted[0] = False
                done.set()
                return
            accepted[0] = True
            handle.get_result_async().add_done_callback(_on_result)

        def _on_result(future):
            result[0] = future.result()
            done.set()

        client.send_goal_async(goal_msg).add_done_callback(_on_goal_response)

        if not done.wait(timeout=timeout_sec):
            self.get_logger().warn(
                f"Action call timed out after {timeout_sec:.1f}s"
            )
            return False, None

        return bool(accepted[0]), result[0]

    # ─────────────────────────────────────────────────────────────────────────
    # TF helper
    # ─────────────────────────────────────────────────────────────────────────

    def _to_planning_frame(
        self, pose_stamped: geo_msgs.PoseStamped
    ) -> geo_msgs.PoseStamped | None:
        """Transform a PoseStamped into the planning frame. Returns None on failure."""
        if pose_stamped.header.frame_id == self._planning_frame:
            return pose_stamped
        try:
            return self._tf_buffer.transform(
                pose_stamped,
                self._planning_frame,
                timeout=Duration(seconds=1.0),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().error(
                f"TF '{pose_stamped.header.frame_id}' → "
                f"'{self._planning_frame}' failed: {exc}"
            )
            return None

    # ─────────────────────────────────────────────────────────────────────────
    # Pose builders
    # ─────────────────────────────────────────────────────────────────────────

    def _approach_orientation(self) -> geo_msgs.Quaternion:
        ori = geo_msgs.Quaternion()
        ori.x = float(self.get_parameter("approach_orientation.x").value)
        ori.y = float(self.get_parameter("approach_orientation.y").value)
        ori.z = float(self.get_parameter("approach_orientation.z").value)
        ori.w = float(self.get_parameter("approach_orientation.w").value)
        return ori

    def _make_pose(self, x: float, y: float, z: float) -> geo_msgs.PoseStamped:
        ps = geo_msgs.PoseStamped()
        ps.header.frame_id  = self._planning_frame
        ps.header.stamp     = self.get_clock().now().to_msg()
        ps.pose.position.x  = x
        ps.pose.position.y  = y
        ps.pose.position.z  = z
        ps.pose.orientation = self._approach_orientation()
        return ps

    def _box_pose(self, target_box: str, z_extra: float = 0.0) -> geo_msgs.PoseStamped:
        prefix = f"{target_box}_box"
        px = float(self.get_parameter(f"{prefix}.x").value)
        py = float(self.get_parameter(f"{prefix}.y").value)
        pz = float(self.get_parameter(f"{prefix}.z").value) + z_extra
        return self._make_pose(px, py, pz)

    # ─────────────────────────────────────────────────────────────────────────
    # Main execute callback
    # ─────────────────────────────────────────────────────────────────────────

    def _execute_cb(self, goal_handle):  # noqa: C901
        goal     = goal_handle.request
        feedback = PickPlace.Feedback()
        result   = PickPlace.Result()

        self.get_logger().info(
            f"PickPlace START  color='{goal.object_color}'"
            f"  target='{goal.target_box}'  retry={goal.retry_on_fail}"
        )

        v_scale      = float(self.get_parameter("velocity_scale").value)
        max_retries  = _MAX_DETECT_RETRIES if goal.retry_on_fail else 1
        grasp_z_off  = float(self.get_parameter("grasp_z_offset").value)
        pre_z_offset = float(self.get_parameter("pre_grasp_z_offset").value)
        lift_z       = float(self.get_parameter("lift_z_offset").value)
        place_z_off  = float(self.get_parameter("place_z_offset").value)

        # ── Helpers local to this goal ─────────────────────────────────

        def pub_fb(phase: str, status: str) -> None:
            feedback.phase  = phase
            feedback.status = status
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(f"  [{phase}] {status}")

        def abort(msg: str, attempts: int = 1) -> PickPlace.Result:
            result.success  = False
            result.message  = msg
            result.attempts = attempts
            goal_handle.abort(result)
            self.get_logger().warn(f"PickPlace ABORT: {msg}")
            return result

        def check_cancel() -> bool:
            """Return True and set canceled if a cancel was requested."""
            if goal_handle.is_cancel_requested:
                result.success  = False
                result.message  = "Cancelled"
                result.attempts = 1
                goal_handle.canceled(result)
                return True
            return False

        # ── Phase 1: detecting ─────────────────────────────────────────
        pub_fb("detecting", f"searching for '{goal.object_color}' block")

        block_in_planning: geo_msgs.PoseStamped | None = None

        for attempt in range(1, max_retries + 1):
            if check_cancel():
                return result

            req       = GetBlockPoses.Request()
            req.color = goal.object_color
            resp      = self._call_service(
                self._get_block_poses, req, timeout_sec=_DETECT_TIMEOUT_SEC
            )

            if resp is not None and len(resp.blocks) > 0:
                best = max(resp.blocks, key=lambda b: b.confidence)
                self.get_logger().info(
                    f"  [detect] block found: "
                    f"({best.pose.pose.position.x:.3f},"
                    f" {best.pose.pose.position.y:.3f},"
                    f" {best.pose.pose.position.z:.3f})"
                    f"  conf={best.confidence:.2f}"
                    f"  frame='{best.pose.header.frame_id}'"
                )
                block_in_planning = self._to_planning_frame(best.pose)
                if block_in_planning is not None:
                    break
                pub_fb("detecting", f"TF transform failed (attempt {attempt}/{max_retries})")
            else:
                reason = "no response" if resp is None else "no blocks detected"
                pub_fb("detecting", f"attempt {attempt}/{max_retries}: {reason}")
                if attempt < max_retries:
                    time.sleep(0.5)

        if block_in_planning is None:
            return abort(
                f"No '{goal.object_color}' block detected after {max_retries} attempt(s)",
                attempts=max_retries,
            )

        bx = block_in_planning.pose.position.x
        by = block_in_planning.pose.position.y
        bz = block_in_planning.pose.position.z
        grasp_z = bz + grasp_z_off

        # ── Phase 2: approaching ───────────────────────────────────────
        if check_cancel():
            return result
        pub_fb("approaching", "moving to pre-grasp pose")

        pre_grasp = self._make_pose(bx, by, grasp_z + pre_z_offset)
        if not self._move_to(pre_grasp, v_scale):
            self._safe_stop()
            return abort("Pre-grasp move failed")

        # ── Phase 3: grasping ──────────────────────────────────────────
        if check_cancel():
            return result

        pub_fb("grasping", "opening gripper")
        if not self._gripper_cmd(_GRIPPER_OPEN):
            self._safe_stop()
            return abort("Gripper open failed")

        pub_fb("grasping", "descending to grasp pose")
        grasp_pose = self._make_pose(bx, by, grasp_z)
        if not self._move_to(grasp_pose, v_scale * 0.5):
            self._safe_stop()
            return abort("Grasp descent failed")

        pub_fb("grasping", "closing gripper")
        if not self._gripper_cmd(_GRIPPER_CLOSE):
            self._safe_stop()
            return abort("Gripper close failed")

        time.sleep(_POST_CLOSE_WAIT_SEC)  # let the grasp settle

        # ── Phase 4: placing ───────────────────────────────────────────
        if check_cancel():
            self._safe_stop()
            return result

        pub_fb("placing", "lifting object")
        lift_pose = self._make_pose(bx, by, grasp_z + lift_z)
        if not self._move_to(lift_pose, v_scale * 0.5):
            self._safe_stop()
            return abort("Lift failed")

        pub_fb("placing", f"transporting to '{goal.target_box}' box")
        pre_place = self._box_pose(goal.target_box, z_extra=place_z_off)
        if not self._move_to(pre_place, v_scale):
            self._safe_stop()
            return abort("Transport to target box failed")

        pub_fb("placing", "descending to release height")
        place = self._box_pose(goal.target_box)
        if not self._move_to(place, v_scale * 0.5):
            self._safe_stop()
            return abort("Descent to place pose failed")

        pub_fb("placing", "releasing object")
        if not self._gripper_cmd(_GRIPPER_OPEN):
            self._safe_stop()
            return abort("Gripper release failed")

        # ── Return to ready ────────────────────────────────────────────
        pub_fb("placing", "returning to ready pose")
        self._move_to_named("ready")

        result.success  = True
        result.message  = (
            f"Placed '{goal.object_color}' block in '{goal.target_box}' box"
        )
        result.attempts = 1
        goal_handle.succeed(result)
        self.get_logger().info(f"PickPlace SUCCEEDED: {result.message}")
        return result

    # ─────────────────────────────────────────────────────────────────────────
    # Motion helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _move_to(self, pose: geo_msgs.PoseStamped, v_scale: float) -> bool:
        g              = MoveToPose.Goal()
        g.target_pose  = pose
        g.velocity_scale = float(max(0.01, min(v_scale, 1.0)))
        ok, res = self._call_action(
            self._move_to_pose_client, g, _MOTION_TIMEOUT_SEC
        )
        if not ok or res is None:
            self.get_logger().warn("MoveToPose: rejected or timed out")
            return False
        if not res.result.success:
            self.get_logger().warn(f"MoveToPose failed: {res.result.message}")
        return res.result.success

    def _move_to_named(self, name: str) -> bool:
        g      = MoveToNamed.Goal()
        g.name = name
        ok, res = self._call_action(
            self._move_to_named_client, g, _MOTION_TIMEOUT_SEC
        )
        if not ok or res is None:
            self.get_logger().warn(f"MoveToNamed '{name}': rejected or timed out")
            return False
        if not res.result.success:
            self.get_logger().warn(f"MoveToNamed '{name}' failed: {res.result.message}")
        return res.result.success

    def _gripper_cmd(self, position: float) -> bool:
        g             = GripperCommand.Goal()
        g.position    = float(position)
        g.max_effort  = 0.0
        ok, res = self._call_action(
            self._gripper_client, g, _GRIPPER_TIMEOUT_SEC
        )
        if not ok or res is None:
            self.get_logger().warn("GripperCommand: rejected or timed out")
            return False
        if not res.result.success:
            self.get_logger().warn(f"GripperCommand failed: {res.result.message}")
        return res.result.success

    def _safe_stop(self) -> None:
        """Recovery: move to home and open gripper regardless of current state."""
        self.get_logger().warn("PickPlace safe_stop — moving to home + opening gripper")
        self._move_to_named("home")
        self._gripper_cmd(_GRIPPER_OPEN)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickPlaceSkill()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
