from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

_MODEL_DEFAULT = "/home/kjhz/omx_ws/src/omx_perception/models/midas_v21_small_256.onnx"

# MiDaS v2.1-small normalization constants
_MEAN = (0.485, 0.456, 0.406)
_STD = (0.229, 0.224, 0.225)
_INPUT_SIZE = 256


class DepthNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_node")
        self.declare_parameter("model_path", _MODEL_DEFAULT)

        model_path = self.get_parameter("model_path").value
        if not Path(model_path).exists():
            self.get_logger().error(f"MiDaS model not found: {model_path}")
            raise FileNotFoundError(model_path)

        self._net = cv2.dnn.readNetFromONNX(model_path)
        self._bridge = CvBridge()

        self.create_subscription(Image, "/image/raw", self._on_image, 10)
        self._depth_pub = self.create_publisher(Image, "/omx/perception/depth", 10)
        self._color_pub = self.create_publisher(Image, "/omx/perception/depth_colorized", 10)
        self.get_logger().info("DepthNode ready")

    def _on_image(self, msg: Image) -> None:
        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = bgr.shape[:2]

        blob = cv2.dnn.blobFromImage(
            bgr,
            scalefactor=1.0 / 255.0,
            size=(_INPUT_SIZE, _INPUT_SIZE),
            mean=tuple(m * 255 for m in _MEAN),
            swapRB=True,
            crop=False,
        )
        # apply std normalization manually (blobFromImage only handles mean)
        blob[0, 0] /= _STD[0]
        blob[0, 1] /= _STD[1]
        blob[0, 2] /= _STD[2]

        self._net.setInput(blob)
        raw: np.ndarray = self._net.forward().squeeze()  # (256, 256)

        # normalize to [0, 1] float32 and resize back to original resolution
        depth_norm = cv2.normalize(raw, None, 0.0, 1.0, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        depth_resized = cv2.resize(depth_norm, (w, h), interpolation=cv2.INTER_LINEAR)

        # publish raw float32 depth
        depth_msg = self._bridge.cv2_to_imgmsg(depth_resized, encoding="32FC1")
        depth_msg.header = msg.header
        self._depth_pub.publish(depth_msg)

        # publish colorized for visualization
        depth_u8 = (depth_resized * 255).astype(np.uint8)
        colorized = cv2.applyColorMap(depth_u8, cv2.COLORMAP_INFERNO)
        color_msg = self._bridge.cv2_to_imgmsg(colorized, encoding="bgr8")
        color_msg.header = msg.header
        self._color_pub.publish(color_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
