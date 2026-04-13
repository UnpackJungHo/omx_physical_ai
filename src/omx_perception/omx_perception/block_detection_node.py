"""omx_perception — BlockDetectionNode

Subscribes to /camera/image_raw and /camera/camera_info.
Serves /omx/get_block_poses (omx_interfaces/srv/GetBlockPoses).
Optionally publishes a debug image on /omx/perception/debug_image.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from omx_interfaces.msg import BlockPose
from omx_interfaces.srv import GetBlockPoses

from .color_detector import detect_blocks, draw_detections
from .pixel_to_3d import PixelTo3D


class BlockDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("block_detection_node")

        # Parameters
        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("table_z_in_camera", 0.30)
        self.declare_parameter("publish_debug_image", True)

        self._camera_frame: str = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )
        table_z: float = (
            self.get_parameter("table_z_in_camera").get_parameter_value().double_value
        )
        self._publish_debug: bool = (
            self.get_parameter("publish_debug_image").get_parameter_value().bool_value
        )

        self._bridge = CvBridge()
        self._pixel_to_3d = PixelTo3D(table_z_in_camera=table_z)

        self._latest_image: np.ndarray | None = None

        # QoS — sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Image, "/camera/image_raw", self._cb_image, sensor_qos)
        self.create_subscription(
            CameraInfo, "/camera/camera_info", self._cb_camera_info, sensor_qos
        )

        self._srv = self.create_service(
            GetBlockPoses, "/omx/get_block_poses", self._handle_get_block_poses
        )

        if self._publish_debug:
            self._debug_pub = self.create_publisher(Image, "/omx/perception/debug_image", 1)
        else:
            self._debug_pub = None

        self.get_logger().info(
            f"BlockDetectionNode ready. camera_frame={self._camera_frame}, "
            f"table_z={table_z:.3f}m"
        )

    # ------------------------------------------------------------------ #
    # Subscriptions
    # ------------------------------------------------------------------ #

    def _cb_image(self, msg: Image) -> None:
        try:
            self._latest_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"cv_bridge error: {e}", throttle_duration_sec=5.0)

    def _cb_camera_info(self, msg: CameraInfo) -> None:
        if not self._pixel_to_3d.is_ready:
            self._pixel_to_3d.update_camera_info(msg)
            self.get_logger().info("CameraInfo received — projection ready.")

    # ------------------------------------------------------------------ #
    # Service handler
    # ------------------------------------------------------------------ #

    def _handle_get_block_poses(
        self,
        request: GetBlockPoses.Request,
        response: GetBlockPoses.Response,
    ) -> GetBlockPoses.Response:
        if self._latest_image is None:
            self.get_logger().warn("No image received yet.")
            return response

        if not self._pixel_to_3d.is_ready:
            self.get_logger().warn(
                "CameraInfo not received yet — returning empty result."
            )
            return response

        target_color = request.color.strip().lower() if request.color else ""

        blocks = detect_blocks(self._latest_image, target_color or None)

        stamp = self.get_clock().now().to_msg()

        for b in blocks:
            try:
                x, y, z = self._pixel_to_3d.pixel_to_camera_point(b.cx, b.cy)
            except RuntimeError:
                continue

            pose_stamped = PoseStamped()
            pose_stamped.header = Header(frame_id=self._camera_frame, stamp=stamp)
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = z
            pose_stamped.pose.orientation.w = 1.0  # identity rotation

            block_pose = BlockPose()
            block_pose.header = Header(frame_id=self._camera_frame, stamp=stamp)
            block_pose.color = b.color
            block_pose.pose = pose_stamped
            block_pose.confidence = b.confidence

            response.blocks.append(block_pose)

        if self._debug_pub is not None:
            debug_img = draw_detections(self._latest_image, blocks)
            try:
                self._debug_pub.publish(
                    self._bridge.cv2_to_imgmsg(debug_img, "bgr8")
                )
            except CvBridgeError:
                pass

        self.get_logger().info(
            f"GetBlockPoses: color='{target_color or 'all'}' → {len(response.blocks)} blocks"
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlockDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
