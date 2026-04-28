from __future__ import annotations

from pathlib import Path
import sys
from threading import Lock

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from omx_interfaces.msg import Top4Box
from omx_interfaces.srv import GetTop4Keypoints
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class Top4KeypointsNode(Node):
    """Service-based YOLOv8-Pose detector for cube top-face keypoints."""

    def __init__(self) -> None:
        super().__init__("top4_keypoints")

        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/image/raw")
        self.declare_parameter("annotated_image_topic", "/image/raw/top4_pose")
        self.declare_parameter("service_name", "/perception/get_top4_keypoints")
        self.declare_parameter(
            "extra_pythonpath",
            "/home/kjhz/miniconda3/envs/driving/lib/python3.12/site-packages",
        )
        self.declare_parameter(
            "device",
            "0",
            ParameterDescriptor(
                description="Ultralytics inference device. Accepts 0, '0', or 'cpu'.",
                dynamic_typing=True,
            ),
        )
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("max_det", 20)

        self._bridge = CvBridge()
        self._lock = Lock()
        self._latest_frame: np.ndarray | None = None
        self._latest_header = None
        self._latest_boxes: list[Top4Box] = []
        self._latest_annotated_frame: np.ndarray | None = None

        model_path = Path(str(self.get_parameter("model_path").value)).expanduser()
        if not model_path.is_absolute():
            model_path = Path.cwd() / model_path
        model_path = model_path.resolve()
        if not model_path.exists():
            raise FileNotFoundError(
                f"YOLO pose model not found: {model_path}. "
                "Set model_path to the trained best.pt."
            )

        YOLO = self._load_yolo_class()
        self._model = YOLO(str(model_path))
        self._device = str(self.get_parameter("device").value)
        self._imgsz = int(self.get_parameter("imgsz").value)
        self._conf = float(self.get_parameter("conf").value)
        self._max_det = int(self.get_parameter("max_det").value)

        image_topic = str(self.get_parameter("image_topic").value)
        annotated_topic = str(self.get_parameter("annotated_image_topic").value)
        service_name = str(self.get_parameter("service_name").value)

        self._annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self._image_sub = self.create_subscription(Image, image_topic, self._on_image, 10)
        self._service = self.create_service(
            GetTop4Keypoints,
            service_name,
            self._on_get_top4_keypoints,
        )

        self.get_logger().info(
            "top4 keypoints service ready "
            f"(model={model_path}, image_topic={image_topic}, "
            f"annotated_image_topic={annotated_topic}, service={service_name})"
        )

    def _load_yolo_class(self):
        extra_pythonpath = Path(
            str(self.get_parameter("extra_pythonpath").value)
        ).expanduser()

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
                    "or set extra_pythonpath to the site-packages directory that contains "
                    "ultralytics and torch."
                ) from exc

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"failed to convert image: {exc}")
            return

        with self._lock:
            self._latest_frame = frame.copy()

        result = self._model.predict(
            source=frame,
            imgsz=self._imgsz,
            conf=self._conf,
            max_det=self._max_det,
            device=self._device,
            verbose=False,
        )[0]
        boxes = self._extract_boxes(result)
        annotated = frame.copy()
        self._draw_boxes(annotated, boxes)

        with self._lock:
            self._latest_header = msg.header
            self._latest_boxes = boxes
            self._latest_annotated_frame = annotated.copy()

        self._publish_view_image(msg.header, annotated)

    def _on_get_top4_keypoints(
        self,
        request: GetTop4Keypoints.Request,
        response: GetTop4Keypoints.Response,
    ) -> GetTop4Keypoints.Response:
        with self._lock:
            header = self._latest_header
            boxes = list(self._latest_boxes)
            annotated = (
                None
                if self._latest_annotated_frame is None
                else self._latest_annotated_frame.copy()
            )

        if header is None:
            response.success = False
            response.message = "No /image/raw frame has been received yet."
            return response

        response.header = header
        response.success = True
        response.message = f"Detected {len(boxes)} box(es)."
        response.boxes = boxes

        if request.publish_debug and annotated is not None:
            self._publish_view_image(header, annotated)

        return response

    def _extract_boxes(self, result) -> list[Top4Box]:
        if result.boxes is None or len(result.boxes) == 0 or result.keypoints is None:
            return []

        detections: list[Top4Box] = []
        boxes_conf = result.boxes.conf.detach().cpu().numpy()
        keypoints_xy = result.keypoints.xy.detach().cpu().numpy()
        keypoints_conf = None
        if result.keypoints.conf is not None:
            keypoints_conf = result.keypoints.conf.detach().cpu().numpy()

        order = np.argsort(-boxes_conf)
        for detection_index in order:
            points = keypoints_xy[int(detection_index)]
            if len(points) < 4:
                continue

            top4_box = Top4Box()
            top4_box.detection_confidence = float(boxes_conf[int(detection_index)])
            values: list[float] = []

            for keypoint_index in range(4):
                point = points[keypoint_index]
                confidence = 1.0
                if keypoints_conf is not None:
                    confidence = float(keypoints_conf[int(detection_index)][keypoint_index])
                values.extend([float(point[0]), float(point[1]), confidence])

            top4_box.keypoints = values
            detections.append(top4_box)

        return detections

    def _draw_boxes(self, image: np.ndarray, boxes: list[Top4Box]) -> None:
        colors = [
            (0, 255, 255),
            (255, 128, 0),
            (0, 255, 0),
            (255, 0, 255),
            (0, 128, 255),
        ]

        cv2.putText(
            image,
            f"YOLO top4 boxes: {len(boxes)}",
            (16, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        for box_index, box in enumerate(boxes):
            color = colors[box_index % len(colors)]
            points: list[tuple[int, int]] = []

            for keypoint_index in range(4):
                base = keypoint_index * 3
                x = int(round(box.keypoints[base]))
                y = int(round(box.keypoints[base + 1]))
                confidence = float(box.keypoints[base + 2])

                if x <= 0 and y <= 0:
                    continue

                points.append((x, y))
                cv2.circle(image, (x, y), 5, color, -1, lineType=cv2.LINE_AA)
                cv2.circle(image, (x, y), 7, (0, 0, 0), 1, lineType=cv2.LINE_AA)
                cv2.putText(
                    image,
                    f"b{box_index}k{keypoint_index}:{confidence:.2f}",
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
                    f"box {box_index}: {box.detection_confidence:.2f}",
                    (points[0][0], max(18, points[0][1] - 24)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                    cv2.LINE_AA,
                )

    def _publish_view_image(self, header, image: np.ndarray) -> None:
        annotated_msg = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        annotated_msg.header = header
        self._annotated_pub.publish(annotated_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Top4KeypointsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
