from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def _blend_scalar(previous: float, current: float, alpha: float) -> float:
    return (1.0 - alpha) * previous + alpha * current


def _blend_angle(previous: float, current: float, alpha: float) -> float:
    delta = math.atan2(math.sin(current - previous), math.cos(current - previous))
    return previous + alpha * delta


def _planar_distance(a: dict, b: dict) -> float:
    dx = float(a.get("x", 0.0)) - float(b.get("x", 0.0))
    dy = float(a.get("y", 0.0)) - float(b.get("y", 0.0))
    return math.hypot(dx, dy)


class TrackerNode(Node):
    """Flat track list with greedy nearest-neighbor association and EMA smoothing."""

    def __init__(self) -> None:
        super().__init__("tracker_node")

        self.declare_parameter("stale_threshold_sec", 1.5)
        self.declare_parameter("min_confidence", 0.25)
        self.declare_parameter("assoc_threshold_m", 0.025)
        self.declare_parameter("position_alpha", 0.35)
        self.declare_parameter("yaw_alpha", 0.25)
        self.declare_parameter("confidence_alpha", 0.4)

        self._frame_id = "world"
        self._stamp_sec = 0
        self._stamp_nanosec = 0
        self._tracks: list[dict] = []
        self._next_track_id = 0
        stream_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(String, "/omx/perception/blocks", self._on_blocks, stream_qos)
        self._pub = self.create_publisher(String, "/omx/perception/tracked_blocks", stream_qos)
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
        assoc_threshold = float(self.get_parameter("assoc_threshold_m").value)
        now = time.monotonic()

        detections: list[dict] = []
        for block in payload.get("blocks", []):
            if float(block.get("confidence", 0.0)) < min_conf:
                continue
            color = str(block.get("color", ""))
            if not color:
                continue
            detections.append(dict(block))

        matches = self._greedy_match(detections, self._tracks, assoc_threshold)

        matched_track_indices: set[int] = set()
        matched_detection_indices: set[int] = set()
        for det_idx, track_idx in matches:
            det = detections[det_idx]
            track = self._tracks[track_idx]
            self._update_track(track, det, pos_alpha, yaw_alpha, conf_alpha, now)
            matched_track_indices.add(track_idx)
            matched_detection_indices.add(det_idx)

        for idx, det in enumerate(detections):
            if idx in matched_detection_indices:
                continue
            self._tracks.append(self._new_track(det, now))

    def _greedy_match(
        self,
        detections: list[dict],
        tracks: list[dict],
        assoc_threshold: float,
    ) -> list[tuple[int, int]]:
        pairs: list[tuple[float, int, int]] = []
        for d_idx, det in enumerate(detections):
            for t_idx, track in enumerate(tracks):
                if det.get("color", "") != track["block"].get("color", ""):
                    continue
                distance = _planar_distance(det, track["block"])
                if distance > assoc_threshold:
                    continue
                pairs.append((distance, d_idx, t_idx))
        pairs.sort(key=lambda p: p[0])

        used_d: set[int] = set()
        used_t: set[int] = set()
        matches: list[tuple[int, int]] = []
        for _, d_idx, t_idx in pairs:
            if d_idx in used_d or t_idx in used_t:
                continue
            used_d.add(d_idx)
            used_t.add(t_idx)
            matches.append((d_idx, t_idx))
        return matches

    def _new_track(self, detection: dict, now: float) -> dict:
        self._next_track_id += 1
        block = dict(detection)
        block["track_id"] = self._next_track_id
        return {"block": block, "updated_at": now}

    def _update_track(
        self,
        track: dict,
        detection: dict,
        pos_alpha: float,
        yaw_alpha: float,
        conf_alpha: float,
        now: float,
    ) -> None:
        previous = track["block"]
        smoothed = dict(detection)
        smoothed["track_id"] = previous.get("track_id")
        smoothed["x"] = _blend_scalar(float(previous["x"]), float(detection["x"]), pos_alpha)
        smoothed["y"] = _blend_scalar(float(previous["y"]), float(detection["y"]), pos_alpha)
        smoothed["z"] = _blend_scalar(float(previous["z"]), float(detection["z"]), pos_alpha)
        smoothed["yaw"] = _blend_angle(
            float(previous.get("yaw", 0.0)),
            float(detection.get("yaw", 0.0)),
            yaw_alpha,
        )
        smoothed["confidence"] = _blend_scalar(
            float(previous.get("confidence", 0.0)),
            float(detection.get("confidence", 0.0)),
            conf_alpha,
        )
        smoothed["pixel_u"] = _blend_scalar(
            float(previous.get("pixel_u", 0.0)),
            float(detection.get("pixel_u", 0.0)),
            pos_alpha,
        )
        smoothed["pixel_v"] = _blend_scalar(
            float(previous.get("pixel_v", 0.0)),
            float(detection.get("pixel_v", 0.0)),
            pos_alpha,
        )
        track["block"] = smoothed
        track["updated_at"] = now

    def _tick(self) -> None:
        stale_sec = float(self.get_parameter("stale_threshold_sec").value)
        now = time.monotonic()

        fresh: list[dict] = []
        for track in self._tracks:
            if (now - float(track["updated_at"])) <= stale_sec:
                fresh.append(track)
        self._tracks = fresh

        blocks = [dict(track["block"]) for track in self._tracks]
        blocks.sort(key=lambda b: (b.get("color", ""), -float(b.get("confidence", 0.0))))

        self._pub.publish(
            String(
                data=json.dumps(
                    {
                        "stamp_sec": self._stamp_sec,
                        "stamp_nanosec": self._stamp_nanosec,
                        "frame_id": self._frame_id,
                        "blocks": blocks,
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
