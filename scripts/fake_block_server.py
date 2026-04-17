#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from omx_interfaces.msg import BlockPose
from omx_interfaces.srv import GetBlockPoses


class FakeBlockServer(Node):
    def __init__(self) -> None:
        super().__init__("fake_block_server")
        self.create_service(GetBlockPoses, "/omx/get_block_poses", self._handle_get_block_poses)
        self.get_logger().info("Fake block server ready: /omx/get_block_poses")

    def _handle_get_block_poses(
        self, req: GetBlockPoses.Request, res: GetBlockPoses.Response
    ) -> GetBlockPoses.Response:
        color = (req.color or "red").strip().lower() or "red"
        if color not in ("red", "green", "blue"):
            self.get_logger().warn(f"Ignoring unsupported color request: '{req.color}'")
            return res

        now = self.get_clock().now().to_msg()
        block = BlockPose()
        block.header = Header(frame_id="world", stamp=now)
        block.color = color
        block.confidence = 0.95

        block.pose = PoseStamped()
        block.pose.header = Header(frame_id="world", stamp=now)
        block.pose.pose.position.x = 0.22
        block.pose.pose.position.y = 0.00
        block.pose.pose.position.z = 0.06
        block.pose.pose.orientation.w = 1.0

        res.blocks.append(block)
        return res


def main() -> None:
    rclpy.init()
    node = FakeBlockServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
