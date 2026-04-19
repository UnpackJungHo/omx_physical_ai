#!/usr/bin/env python3
"""Image collection tool for omx_perception dataset.

Controls
--------
SPACE : save current frame
Q     : quit
"""
from __future__ import annotations

import os
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

SAVE_DIR = Path("/home/kjhz/omx_ws/datasets/block_detection/images/train")
SAVE_DIR.mkdir(parents=True, exist_ok=True)


class CollectNode(Node):
    def __init__(self) -> None:
        super().__init__("collect_dataset")
        self._bridge = CvBridge()
        self._frame: cv2.Mat | None = None
        self._count = len(list(SAVE_DIR.glob("*.jpg")))
        self.create_subscription(Image, "/image/raw", self._on_image, 10)
        self.get_logger().info(f"Save dir: {SAVE_DIR}  |  existing frames: {self._count}")
        self.get_logger().info("SPACE=save  Q=quit")

    def _on_image(self, msg: Image) -> None:
        self._frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def spin_with_ui(self) -> None:
        cv2.namedWindow("collect — SPACE:save  Q:quit", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("collect — SPACE:save  Q:quit", 960, 540)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.03)

            if self._frame is None:
                continue

            display = self._frame.copy()
            cv2.putText(
                display,
                f"saved: {self._count}  |  SPACE=save  Q=quit",
                (16, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 0),
                2,
            )
            cv2.imshow("collect — SPACE:save  Q:quit", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" "):
                ts = int(time.time() * 1000)
                path = SAVE_DIR / f"frame_{ts:016d}.jpg"
                cv2.imwrite(str(path), self._frame)
                self._count += 1
                self.get_logger().info(f"[{self._count}] saved {path.name}")

        cv2.destroyAllWindows()


def main() -> None:
    rclpy.init()
    node = CollectNode()
    try:
        node.spin_with_ui()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
