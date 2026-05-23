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
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image

import math

from omx_interfaces.msg import KeypointDetection
from omx_interfaces.srv import GetBlockPoses, GetKeypointDetections


CLASS_NAMES = {
    0: "Box",
    1: "Cup",
}

CLASS_COLORS = {
    0: (0, 255, 255),
    1: (0, 165, 255),
}


class BoxCupKeypointNode(Node):
    def __init__(self) -> None:
        super().__init__("box_cup_keypoint")

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
        self.declare_parameter("publish_debug", False)

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
        self._publish_debug = bool(self.get_parameter("publish_debug").value)

        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        image_topic = str(self.get_parameter("image_topic").value)
        self._image_sub = self.create_subscription(Image, image_topic, self._on_image, image_qos)

        self._debug_pub = None
        if self._publish_debug:
            self._debug_pub = self.create_publisher(Image, "/perception/debug_image", 1)

        cb_group = ReentrantCallbackGroup()
        self._srv = self.create_service(
            GetKeypointDetections,
            "/perception/get_box_cup_keypoints",
            self._handle_get_keypoints,
            callback_group=cb_group,
        )

        self._world_pose_client = self.create_client(
            GetBlockPoses,
            "/perception/get_box_cup_world_poses",
            callback_group=cb_group,
        )

        self._start_keyboard_listener()

        self.get_logger().info(
            "box/cup keypoint node ready: service=/perception/get_box_cup_keypoints, "
            "press 'p' to save snapshot "
            f"(model={self._model_path}, image_topic={image_topic})"
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

    def _handle_get_keypoints(
        self,
        request: GetKeypointDetections.Request,
        response: GetKeypointDetections.Response,
    ) -> GetKeypointDetections.Response:
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            stamp = self._latest_stamp

        if frame is None:
            response.success = False
            response.message = "no image frame received yet"
            return response

        try:
            result = self._model.predict(
                source=frame,
                imgsz=self._imgsz,
                conf=self._conf,
                max_det=self._max_det,
                device=self._device,
                verbose=False,
            )[0]
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"YOLO inference error: {exc}"
            return response

        raw_detections = self._extract_detections(result)

        if stamp is not None:
            from builtin_interfaces.msg import Time
            response.header.stamp = stamp
        response.header.frame_id = "default_cam"

        for det in raw_detections:
            kd = KeypointDetection()
            kd.class_id = int(det["class_id"])
            kd.class_name = str(det["class_name"]).lower()
            kd.detection_confidence = float(det["confidence"])
            kd.color = kd.class_name  # "box" | "cup"
            kd.color_confidence = 0.0
            flat: list[float] = []
            for kp_x, kp_y, kp_conf in det["keypoints"]:
                flat.extend([kp_x, kp_y, kp_conf])
            kd.keypoints = flat
            response.detections.append(kd)

        if request.publish_debug:
            annotated = frame.copy()
            self._draw_detections(annotated, raw_detections)
            if self._debug_pub is not None:
                try:
                    img_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                    if stamp is not None:
                        img_msg.header.stamp = stamp
                    img_msg.header.frame_id = "default_cam"
                    self._debug_pub.publish(img_msg)
                except CvBridgeError as exc:
                    self.get_logger().warning(f"failed to publish debug image: {exc}")
            else:
                self.get_logger().warning(
                    "publish_debug requested but publish_debug param is false; no publisher created"
                )

        response.success = True
        response.message = ""
        return response

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

        # world pose overlay (best-effort: world_pose node가 없으면 생략)
        block_poses = self._fetch_world_poses_sync()
        if block_poses is not None:
            self._draw_world_poses(annotated, detections, block_poses)

        filename = self._build_filename(stamp)
        output_path = self._output_dir / filename
        if not cv2.imwrite(str(output_path), annotated):
            self.get_logger().error(f"failed to write YOLO snapshot: {output_path}")
            return

        self.get_logger().info(
            f"saved YOLO snapshot: {output_path} "
            f"({len(detections)} detection(s), world_pose={'ok' if block_poses is not None else 'unavailable'})"
        )

    def _fetch_world_poses_sync(self) -> list | None:
        """키보드 스레드에서 world pose 서비스를 event 기반으로 호출한다."""
        if not self._world_pose_client.service_is_ready():
            return None

        req = GetBlockPoses.Request()
        req.color = ""

        future = self._world_pose_client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout=3.0):
            self.get_logger().warning("world pose service timed out during snapshot")
            return None

        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warning(f"world pose service error: {exc}")
            return None

        return resp.blocks

    def _draw_world_poses(
        self,
        image: np.ndarray,
        detections: list[dict[str, Any]],
        blocks: list,
    ) -> None:
        """solvePnP 결과 (x, y, yaw_deg)를 각 검출 centroid 근처에 그린다."""
        # blocks는 confidence 내림차순, detections도 동일 순서이므로 index로 매칭
        for idx, block in enumerate(blocks):
            color_name = block.color
            p = block.pose.pose.position
            q = block.pose.pose.orientation

            yaw_deg: float | None = None
            if block.yaw_confidence > 0.0:
                yaw_rad = 2.0 * math.atan2(q.z, q.w)
                yaw_deg = math.degrees(yaw_rad)

            lines = [
                f"x={p.x:.3f}m  y={p.y:.3f}m",
                f"z={p.z:.3f}m  [{color_name}]",
            ]
            if yaw_deg is not None:
                lines.append(f"yaw={yaw_deg:.1f}deg")

            # anchor: 해당 detection의 keypoint centroid; 없으면 이미지 상단 나열
            anchor_x, anchor_y = self._detection_centroid(detections, idx)

            bg_x = anchor_x
            bg_y = anchor_y + 12
            line_h = 16
            box_w = 180
            box_h = line_h * len(lines) + 6
            cv2.rectangle(
                image,
                (bg_x - 2, bg_y - line_h),
                (bg_x + box_w, bg_y + box_h - line_h),
                (20, 20, 20),
                -1,
            )
            for li, line in enumerate(lines):
                cv2.putText(
                    image,
                    line,
                    (bg_x, bg_y + li * line_h),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.40,
                    (220, 220, 220),
                    1,
                    cv2.LINE_AA,
                )

    def _detection_centroid(
        self,
        detections: list[dict[str, Any]],
        idx: int,
    ) -> tuple[int, int]:
        if idx < len(detections):
            kps = [
                (int(round(x)), int(round(y)))
                for x, y, _ in detections[idx]["keypoints"]
                if x > 0 or y > 0
            ]
            if kps:
                cx = int(sum(p[0] for p in kps) / len(kps))
                cy = int(sum(p[1] for p in kps) / len(kps))
                return cx, cy
        # fallback: 이미지 왼쪽 상단에 순서대로 나열
        return 10, 60 + idx * 60

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
    node = BoxCupKeypointNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
