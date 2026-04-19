from __future__ import annotations

import json
import time

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node
from std_msgs.msg import Header, String

from omx_interfaces.msg import BlockPose
from omx_interfaces.srv import GetBlockPoses


class GetBlockPosesServer(Node):
    def __init__(self) -> None:
        super().__init__("get_block_poses_server")

        self.declare_parameter("stale_threshold_sec", 2.0)

        self._latest: dict | None = None
        self._received_at: float = 0.0

        self.create_subscription(String, "/omx/perception/tracked_blocks", self._on_blocks, 10)
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

        for b in self._latest.get("blocks", []):
            color = b.get("color", "")
            if request.color and request.color != color:
                continue

            header = Header(
                frame_id=frame_id,
                stamp=Time(sec=stamp_sec, nanosec=stamp_nanosec),
            )
            pose_stamped = PoseStamped(
                header=header,
                pose=Pose(
                    position=Point(x=float(b["x"]), y=float(b["y"]), z=float(b["z"])),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            )
            block_pose = BlockPose(
                header=header,
                color=color,
                pose=pose_stamped,
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
        rclpy.shutdown()
