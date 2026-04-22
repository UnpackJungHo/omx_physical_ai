from __future__ import annotations

import json
import math
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header, String

from omx_interfaces.msg import BlockPose
from omx_interfaces.srv import GetBlockPoses


def _canonicalize_cube_yaw(yaw_rad: float) -> float:
    """Fold yaw into the cube-symmetric range [-pi/4, pi/4]."""
    half_turn = math.pi / 2.0
    wrapped = (yaw_rad + math.pi / 4.0) % half_turn
    return wrapped - math.pi / 4.0


def _yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    half = 0.5 * yaw_rad
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(half),
        w=math.cos(half),
    )


def _x_axis_quaternion(angle_rad: float) -> Quaternion:
    half = 0.5 * angle_rad
    return Quaternion(
        x=math.sin(half),
        y=0.0,
        z=0.0,
        w=math.cos(half),
    )


def _y_axis_quaternion(angle_rad: float) -> Quaternion:
    half = 0.5 * angle_rad
    return Quaternion(
        x=0.0,
        y=math.sin(half),
        z=0.0,
        w=math.cos(half),
    )


def _normalize_quaternion(quat: Quaternion) -> Quaternion:
    norm = math.sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w)
    if norm == 0.0:
        return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    return Quaternion(
        x=quat.x / norm,
        y=quat.y / norm,
        z=quat.z / norm,
        w=quat.w / norm,
    )


def _quaternion_multiply(lhs: Quaternion, rhs: Quaternion) -> Quaternion:
    return Quaternion(
        x=lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y,
        y=lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
        z=lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
        w=lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z,
    )


def _omx_top_down_grasp_quaternion(yaw_rad: float, yaw_offset_rad: float = 0.0) -> Quaternion:
    """Build an OMX-friendly top-down grasp orientation.

    OMX's ``end_effector_link`` extends forward from ``link5`` along the local +X axis.
    A top-down grasp therefore needs a fixed +90 deg pitch so the tool +X axis points
    toward world -Z. The in-plane block yaw is then applied as a roll about the tool
    approach axis, which is what joint5 can realize once the wrist is pitched down.
    """

    top_down = _y_axis_quaternion(math.pi / 2.0)
    in_plane_roll = _x_axis_quaternion(yaw_rad + yaw_offset_rad)
    return _normalize_quaternion(_quaternion_multiply(top_down, in_plane_roll))


class GetBlockPosesServer(Node):
    def __init__(self) -> None:
        super().__init__("get_block_poses_server")

        self.declare_parameter("stale_threshold_sec", 2.0)
        self.declare_parameter("grasp_yaw_offset_deg", 0.0)

        self._latest: dict | None = None
        self._received_at: float = 0.0
        stream_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(
            String, "/omx/perception/tracked_blocks", self._on_blocks, stream_qos
        )
        self.create_service(GetBlockPoses, "/omx/get_block_poses", self._handle)

    def _on_blocks(self, msg: String) -> None:
        try:
            self._latest = json.loads(msg.data)
            self._received_at = time.monotonic()
        except json.JSONDecodeError:
            pass

    def _handle(
        self,
        request: GetBlockPoses.Request,
        response: GetBlockPoses.Response,
    ) -> GetBlockPoses.Response:
        stale_sec = float(self.get_parameter("stale_threshold_sec").value)

        if self._latest is None:
            return response

        if (time.monotonic() - self._received_at) > stale_sec:
            self.get_logger().warn("perception result is stale — returning empty")
            return response

        frame_id = self._latest.get("frame_id", "world")
        stamp_sec = int(self._latest.get("stamp_sec", 0))
        stamp_nanosec = int(self._latest.get("stamp_nanosec", 0))
        grasp_yaw_offset_rad = math.radians(float(self.get_parameter("grasp_yaw_offset_deg").value))

        for b in self._latest.get("blocks", []):
            color = b.get("color", "")
            if request.color and request.color != color:
                continue

            raw_yaw = float(b.get("yaw", 0.0))
            canonical_yaw = _canonicalize_cube_yaw(raw_yaw) if math.isfinite(raw_yaw) else 0.0

            header = Header(
                frame_id=frame_id,
                stamp=Time(sec=stamp_sec, nanosec=stamp_nanosec),
            )
            observed_pose = PoseStamped(
                header=header,
                pose=Pose(
                    position=Point(x=float(b["x"]), y=float(b["y"]), z=float(b["z"])),
                    orientation=_yaw_to_quaternion(canonical_yaw),
                ),
            )
            grasp_pose = PoseStamped(
                header=header,
                pose=Pose(
                    position=Point(x=float(b["x"]), y=float(b["y"]), z=float(b["z"])),
                    orientation=_omx_top_down_grasp_quaternion(canonical_yaw, grasp_yaw_offset_rad),
                ),
            )
            block_pose = BlockPose(
                header=header,
                color=color,
                pose=observed_pose,
                grasp_pose=grasp_pose,
                confidence=float(b.get("confidence", 0.7)),
            )
            response.blocks.append(block_pose)

        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GetBlockPosesServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
