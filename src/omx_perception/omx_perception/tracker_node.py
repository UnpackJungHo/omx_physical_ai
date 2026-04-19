from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _blend_scalar(previous: float, current: float, alpha: float) -> float:
    return (1.0 - alpha) * previous + alpha * current


def _blend_angle(previous: float, current: float, alpha: float) -> float:
    delta = math.atan2(math.sin(current - previous), math.cos(current - previous))
    return previous + alpha * delta


class TrackerNode(Node):
    """Color-wise tracker with staleness filtering and simple temporal smoothing."""

    def __init__(self) -> None:
        super().__init__("tracker_node")

        self.declare_parameter("stale_threshold_sec", 1.0)
        self.declare_parameter("min_confidence", 0.3)
        self.declare_parameter("position_alpha", 0.35)
        self.declare_parameter("yaw_alpha", 0.25)
        self.declare_parameter("confidence_alpha", 0.4)

        self._frame_id = "world"
        self._stamp_sec = 0
        self._stamp_nanosec = 0
        self._tracks: dict[str, dict] = {}

        self.create_subscription(String, "/omx/perception/blocks", self._on_blocks, 10)
        self._pub = self.create_publisher(String, "/omx/perception/tracked_blocks", 10)
        self.create_timer(0.1, self._tick)

    def _on_blocks(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        self._frame_id = payload.get("frame_id", "world")
        self._stamp_sec = int(payload.get("stamp_sec", 0))
        self._stamp_nanosec = int(payload.get("stamp_nanosec", 0))

        min_conf = float(self.get_parameter("min_confidence").value)
        pos_alpha = float(self.get_parameter("position_alpha").value)
        yaw_alpha = float(self.get_parameter("yaw_alpha").value)
        conf_alpha = float(self.get_parameter("confidence_alpha").value)
        now = time.monotonic()

        best_by_color: dict[str, dict] = {}
        for block in payload.get("blocks", []):
            confidence = float(block.get("confidence", 0.0))
            if confidence < min_conf:
                continue

            color = str(block.get("color", ""))
            if not color:
                continue

            previous = best_by_color.get(color)
            if previous is None or confidence > float(previous.get("confidence", 0.0)):
                best_by_color[color] = dict(block)

        for color, block in best_by_color.items():
            prev = self._tracks.get(color)
            if prev is None:
                smoothed = dict(block)
            else:
                smoothed = dict(prev["block"])
                smoothed["x"] = _blend_scalar(float(prev["block"]["x"]), float(block["x"]), pos_alpha)
                smoothed["y"] = _blend_scalar(float(prev["block"]["y"]), float(block["y"]), pos_alpha)
                smoothed["z"] = _blend_scalar(float(prev["block"]["z"]), float(block["z"]), pos_alpha)
                smoothed["yaw"] = _blend_angle(
                    float(prev["block"].get("yaw", 0.0)),
                    float(block.get("yaw", 0.0)),
                    yaw_alpha,
                )
                smoothed["confidence"] = _blend_scalar(
                    float(prev["block"].get("confidence", 0.0)),
                    float(block.get("confidence", 0.0)),
                    conf_alpha,
                )
                smoothed["pixel_u"] = _blend_scalar(
                    float(prev["block"].get("pixel_u", 0.0)),
                    float(block.get("pixel_u", 0.0)),
                    pos_alpha,
                )
                smoothed["pixel_v"] = _blend_scalar(
                    float(prev["block"].get("pixel_v", 0.0)),
                    float(block.get("pixel_v", 0.0)),
                    pos_alpha,
                )
                for key in ("source", "reproj_err", "solidity", "fill_ratio"):
                    if key in block:
                        smoothed[key] = block[key]

            self._tracks[color] = {
                "block": smoothed,
                "updated_at": now,
            }

    def _tick(self) -> None:
        stale_sec = float(self.get_parameter("stale_threshold_sec").value)
        now = time.monotonic()

        blocks: list[dict] = []
        stale_colors: list[str] = []
        for color, track in self._tracks.items():
            if (now - float(track["updated_at"])) > stale_sec:
                stale_colors.append(color)
                continue
            blocks.append(dict(track["block"]))

        for color in stale_colors:
            self._tracks.pop(color, None)

        self._pub.publish(
            String(
                data=json.dumps(
                    {
                        "stamp_sec": self._stamp_sec,
                        "stamp_nanosec": self._stamp_nanosec,
                        "frame_id": self._frame_id,
                        "blocks": sorted(blocks, key=lambda b: b.get("color", "")),
                    }
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
