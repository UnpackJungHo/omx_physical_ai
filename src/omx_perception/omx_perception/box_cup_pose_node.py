from __future__ import annotations

from pathlib import Path
import sys
from threading import Lock

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from omx_interfaces.msg import KeypointDetection
from omx_interfaces.srv import GetKeypointDetections
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from omx_perception.color_classifier import (
    ClassifierParams,
    ColorRef,
    classify,
    load_reference_yaml,
)


CLASS_NAMES = {
    KeypointDetection.CLASS_BOX: "box",
    KeypointDetection.CLASS_CUP: "cup",
}

_COLOR_BGR = {
    "red": (0, 0, 220),
    "green": (0, 200, 0),
    "blue": (220, 80, 0),
    "unknown": (160, 160, 160),
}


class BoxCupPoseNode(Node):
    """Service-based YOLOv8-Pose detector for box top-corners + cup rim keypoints."""

    def __init__(self) -> None:
        super().__init__("box_cup_pose")

        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/image/raw")
        self.declare_parameter("annotated_image_topic", "/image/raw/box_cup_pose")
        self.declare_parameter("service_name", "/perception/get_box_cup_keypoints")
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
        self.declare_parameter("box_color_reference_path", "")

        self._bridge = CvBridge()
        self._lock = Lock()
        self._latest_header = None
        self._latest_detections: list[KeypointDetection] = []
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
        self._color_refs: list[ColorRef] | None = None
        self._color_params: ClassifierParams | None = None
        self._load_color_reference()

        image_topic = str(self.get_parameter("image_topic").value)
        annotated_topic = str(self.get_parameter("annotated_image_topic").value)
        service_name = str(self.get_parameter("service_name").value)

        self._annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self._image_sub = self.create_subscription(Image, image_topic, self._on_image, 10)
        self._service = self.create_service(
            GetKeypointDetections,
            service_name,
            self._on_get_detections,
        )

        self.get_logger().info(
            "box_cup pose service ready "
            f"(model={model_path}, image_topic={image_topic}, "
            f"annotated_image_topic={annotated_topic}, service={service_name}, "
            f"color_classifier={'enabled' if self._color_refs is not None else 'disabled'})"
        )

    def _load_color_reference(self) -> None:
        ref_path_str = str(self.get_parameter("box_color_reference_path").value).strip()
        if not ref_path_str:
            self.get_logger().warning(
                "box_color_reference_path not set; color classification disabled"
            )
            return

        ref_path = Path(ref_path_str).expanduser().resolve()
        refs, params = load_reference_yaml(ref_path)
        if refs is None or params is None:
            self.get_logger().warning(
                f"failed to load color reference yaml: {ref_path}; "
                "color classification disabled"
            )
            return

        self._color_refs = refs
        self._color_params = params
        self.get_logger().info(f"loaded {len(refs)} color references from {ref_path}")

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

        result = self._model.predict(
            source=frame,
            imgsz=self._imgsz,
            conf=self._conf,
            max_det=self._max_det,
            device=self._device,
            verbose=False,
        )[0]
        detections = self._extract_detections(result, frame)
        annotated = frame.copy()
        self._draw_detections(annotated, detections)

        with self._lock:
            self._latest_header = msg.header
            self._latest_detections = detections
            self._latest_annotated_frame = annotated.copy()

        self._publish_view_image(msg.header, annotated)

    def _on_get_detections(
        self,
        request: GetKeypointDetections.Request,
        response: GetKeypointDetections.Response,
    ) -> GetKeypointDetections.Response:
        with self._lock:
            header = self._latest_header
            detections = list(self._latest_detections)
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
        response.message = f"Detected {len(detections)} object(s)."
        response.detections = detections

        if request.publish_debug and annotated is not None:
            self._publish_view_image(header, annotated)

        return response

    def _extract_detections(self, result, frame: np.ndarray) -> list[KeypointDetection]:
        if result.boxes is None or len(result.boxes) == 0 or result.keypoints is None:
            return []

        boxes_conf = result.boxes.conf.detach().cpu().numpy()
        boxes_cls = result.boxes.cls.detach().cpu().numpy().astype(int)
        keypoints_xy = result.keypoints.xy.detach().cpu().numpy()
        keypoints_conf = None
        if result.keypoints.conf is not None:
            keypoints_conf = result.keypoints.conf.detach().cpu().numpy()

        detections: list[KeypointDetection] = []
        order = np.argsort(-boxes_conf)
        for detection_index in order:
            idx = int(detection_index)
            points = keypoints_xy[idx]
            if len(points) < 4:
                continue

            class_id = int(boxes_cls[idx])
            class_name = CLASS_NAMES.get(class_id, f"class_{class_id}")

            det = KeypointDetection()
            det.class_id = class_id
            det.class_name = class_name
            det.detection_confidence = float(boxes_conf[idx])

            values: list[float] = []
            polygon_pts: list[list[float]] = []
            for keypoint_index in range(4):
                point = points[keypoint_index]
                confidence = 1.0
                if keypoints_conf is not None:
                    confidence = float(keypoints_conf[idx][keypoint_index])
                values.extend([float(point[0]), float(point[1]), confidence])
                polygon_pts.append([float(point[0]), float(point[1])])
            det.keypoints = values
            if (
                class_id == KeypointDetection.CLASS_BOX
                and self._color_refs is not None
                and self._color_params is not None
            ):
                polygon_array = np.array(polygon_pts, dtype=float)
                det.color, det.color_confidence = classify(
                    frame,
                    polygon_array,
                    self._color_refs,
                    self._color_params,
                )
            else:
                det.color = ""
                det.color_confidence = 0.0

            detections.append(det)

        return detections

    def _draw_detections(self, image: np.ndarray, detections: list[KeypointDetection]) -> None:
        box_palette = [
            (0, 255, 255),
            (255, 128, 0),
            (0, 255, 0),
            (255, 0, 255),
            (0, 128, 255),
        ]
        cup_color = (0, 165, 255)  # BGR orange

        cv2.putText(
            image,
            f"YOLO box+cup: {len(detections)}",
            (16, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        box_counter = 0
        for det_index, det in enumerate(detections):
            if det.class_id == KeypointDetection.CLASS_CUP:
                color = cup_color
                draw_polygon = False
            else:
                color = box_palette[box_counter % len(box_palette)]
                box_counter += 1
                draw_polygon = True

            points: list[tuple[int, int]] = []

            for keypoint_index in range(4):
                base = keypoint_index * 3
                x = int(round(det.keypoints[base]))
                y = int(round(det.keypoints[base + 1]))
                confidence = float(det.keypoints[base + 2])

                if x <= 0 and y <= 0:
                    continue

                points.append((x, y))
                cv2.circle(image, (x, y), 5, color, -1, lineType=cv2.LINE_AA)
                cv2.circle(image, (x, y), 7, (0, 0, 0), 1, lineType=cv2.LINE_AA)
                cv2.putText(
                    image,
                    f"{det.class_name}{det_index}k{keypoint_index}:{confidence:.2f}",
                    (x + 7, y - 7),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    color,
                    1,
                    cv2.LINE_AA,
                )

            if draw_polygon and len(points) == 4:
                for start, end in ((0, 1), (1, 2), (2, 3), (3, 0)):
                    cv2.line(image, points[start], points[end], color, 2, lineType=cv2.LINE_AA)

            if points:
                label = f"{det.class_name} {det_index}: {det.detection_confidence:.2f}"
                label_color = color
                if det.class_id == KeypointDetection.CLASS_BOX and det.color:
                    label += f" [{det.color} {det.color_confidence:.2f}]"
                    label_color = _COLOR_BGR.get(det.color, color)

                cv2.putText(
                    image,
                    label,
                    (points[0][0], max(18, points[0][1] - 24)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    label_color,
                    2,
                    cv2.LINE_AA,
                )

    def _publish_view_image(self, header, image: np.ndarray) -> None:
        annotated_msg = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        annotated_msg.header = header
        self._annotated_pub.publish(annotated_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoxCupPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
