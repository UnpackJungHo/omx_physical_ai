from __future__ import annotations

from datetime import datetime
from pathlib import Path
import select
import sys
import termios
import threading
import tty
from typing import Any

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


CLASS_NAMES = {
    0: "Box",
    1: "Cup",
}

CLASS_COLORS = {
    0: (0, 255, 255),
    1: (0, 165, 255),
}


class BoxCupPoseSnapshotNode(Node):
    def __init__(self) -> None:
        super().__init__("box_cup_pose_snapshot")

        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/image/raw")
        self.declare_parameter("output_dir", "tmp_kjh/box_cup_pose_image")
        self.declare_parameter(
            "extra_pythonpath",
            "/home/kjhz/miniconda3/envs/driving/lib/python3.12/site-packages",
        )
        self.declare_parameter("device", "0")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("max_det", 20)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_frame: np.ndarray | None = None
        self._latest_stamp = None
        self._stop_event = threading.Event()
        self._keyboard_thread: threading.Thread | None = None

        self._model_path = self._resolve_path(str(self.get_parameter("model_path").value))
        if not self._model_path.exists():
            raise FileNotFoundError(
                f"YOLO pose model not found: {self._model_path}. Set model_path to best.pt."
            )

        self._output_dir = self._resolve_path(str(self.get_parameter("output_dir").value))
        self._output_dir.mkdir(parents=True, exist_ok=True)

        YOLO = self._load_yolo_class()
        self._model = YOLO(str(self._model_path))
        self._device = str(self.get_parameter("device").value)
        self._imgsz = int(self.get_parameter("imgsz").value)
        self._conf = float(self.get_parameter("conf").value)
        self._max_det = int(self.get_parameter("max_det").value)

        image_topic = str(self.get_parameter("image_topic").value)
        self._image_sub = self.create_subscription(Image, image_topic, self._on_image, 10)
        self._start_keyboard_listener()

        self.get_logger().info(
            "box/cup YOLO snapshot ready: press 'p' in this launch terminal to save an annotated image "
            f"(model={self._model_path}, image_topic={image_topic}, output_dir={self._output_dir})"
        )

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._keyboard_thread is not None and self._keyboard_thread.is_alive():
            self._keyboard_thread.join(timeout=1.0)
        return super().destroy_node()

    def _resolve_path(self, value: str) -> Path:
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        return path.resolve()

    def _load_yolo_class(self):
        extra_pythonpath = Path(str(self.get_parameter("extra_pythonpath").value)).expanduser()
        try:
            from ultralytics import YOLO
            return YOLO
        except ModuleNotFoundError:
            if extra_pythonpath.exists() and str(extra_pythonpath) not in sys.path:
                sys.path.insert(0, str(extra_pythonpath))
                self.get_logger().info(
                    f"added extra_pythonpath for YOLO dependencies: {extra_pythonpath}"
                )

            try:
                from ultralytics import YOLO
                return YOLO
            except ModuleNotFoundError as exc:
                raise ModuleNotFoundError(
                    "Missing Python package: ultralytics. Install it for /usr/bin/python3 "
                    "or set extra_pythonpath to the site-packages directory that contains ultralytics and torch."
                ) from exc

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"failed to convert image: {exc}")
            return

        with self._lock:
            self._latest_frame = frame.copy()
            self._latest_stamp = msg.header.stamp

    def _start_keyboard_listener(self) -> None:
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()

    def _keyboard_loop(self) -> None:
        try:
            with open("/dev/tty", "r", encoding="utf-8") as tty_file:
                fd = tty_file.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setcbreak(fd)
                    while not self._stop_event.is_set() and rclpy.ok():
                        readable, _, _ = select.select([tty_file], [], [], 0.1)
                        if not readable:
                            continue
                        char = tty_file.read(1)
                        if char in ("p", "P"):
                            self._save_snapshot()
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except OSError as exc:
            self.get_logger().warning(f"keyboard listener disabled; cannot open /dev/tty: {exc}")

    def _save_snapshot(self) -> None:
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            stamp = self._latest_stamp

        if frame is None:
            self.get_logger().warning("cannot save YOLO snapshot: no /image/raw frame received yet")
            return

        result = self._model.predict(
            source=frame,
            imgsz=self._imgsz,
            conf=self._conf,
            max_det=self._max_det,
            device=self._device,
            verbose=False,
        )[0]
        detections = self._extract_detections(result)
        annotated = frame.copy()
        self._draw_detections(annotated, detections)

        filename = self._build_filename(stamp)
        output_path = self._output_dir / filename
        if not cv2.imwrite(str(output_path), annotated):
            self.get_logger().error(f"failed to write YOLO snapshot: {output_path}")
            return

        self.get_logger().info(
            f"saved YOLO snapshot: {output_path} ({len(detections)} detection(s))"
        )

    def _build_filename(self, stamp: Any) -> str:
        now = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        if stamp is None:
            return f"box_cup_pose_{now}.jpg"
        return f"box_cup_pose_{stamp.sec}_{stamp.nanosec}_{now}.jpg"

    def _extract_detections(self, result) -> list[dict[str, Any]]:
        if result.boxes is None or len(result.boxes) == 0 or result.keypoints is None:
            return []

        boxes_conf = result.boxes.conf.detach().cpu().numpy()
        boxes_cls = result.boxes.cls.detach().cpu().numpy().astype(int)
        keypoints_xy = result.keypoints.xy.detach().cpu().numpy()
        keypoints_conf = None
        if result.keypoints.conf is not None:
            keypoints_conf = result.keypoints.conf.detach().cpu().numpy()

        detections: list[dict[str, Any]] = []
        for detection_index in np.argsort(-boxes_conf):
            idx = int(detection_index)
            points = keypoints_xy[idx]
            if len(points) < 4:
                continue

            class_id = int(boxes_cls[idx])
            keypoints = []
            for keypoint_index in range(4):
                point = points[keypoint_index]
                confidence = 1.0
                if keypoints_conf is not None:
                    confidence = float(keypoints_conf[idx][keypoint_index])
                keypoints.append((float(point[0]), float(point[1]), confidence))

            detections.append(
                {
                    "class_id": class_id,
                    "class_name": CLASS_NAMES.get(class_id, f"class_{class_id}"),
                    "confidence": float(boxes_conf[idx]),
                    "keypoints": keypoints,
                }
            )

        return detections

    def _draw_detections(self, image: np.ndarray, detections: list[dict[str, Any]]) -> None:
        cv2.putText(
            image,
            f"YOLO box/cup pose: {len(detections)}",
            (16, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        for det_index, detection in enumerate(detections):
            class_id = int(detection["class_id"])
            class_name = str(detection["class_name"])
            color = CLASS_COLORS.get(class_id, (180, 180, 180))
            points: list[tuple[int, int]] = []

            for keypoint_index, (x_value, y_value, keypoint_conf) in enumerate(detection["keypoints"]):
                x = int(round(x_value))
                y = int(round(y_value))
                if x <= 0 and y <= 0:
                    continue

                points.append((x, y))
                cv2.circle(image, (x, y), 5, color, -1, lineType=cv2.LINE_AA)
                cv2.circle(image, (x, y), 7, (0, 0, 0), 1, lineType=cv2.LINE_AA)
                cv2.putText(
                    image,
                    f"{class_name}{det_index}.p{keypoint_index}:{keypoint_conf:.2f}",
                    (x + 7, y - 7),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    color,
                    1,
                    cv2.LINE_AA,
                )

            if len(points) == 4:
                for start, end in ((0, 1), (1, 2), (2, 3), (3, 0)):
                    cv2.line(image, points[start], points[end], color, 2, lineType=cv2.LINE_AA)

            if points:
                cv2.putText(
                    image,
                    f"{class_name} {det_index}: {float(detection['confidence']):.2f}",
                    (points[0][0], max(18, points[0][1] - 24)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                    cv2.LINE_AA,
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoxCupPoseSnapshotNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
