# Box + Cup Pose Perception Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace single-class `top4_*` perception nodes with a 2-class (Box+Cup) pipeline that publishes annotated debug images and provides pixel-keypoint and world-pose services for both classes.

**Architecture:** YOLOv8-pose `box_cup_pose_2class_96` model (4 keypoints/class) drives a refactored pose node and a class-aware world-pose node. New `KeypointDetection` msg and `GetKeypointDetections` srv carry per-detection class info. World poses are returned via the existing `BlockPose[]` (`color = "box" | "cup"`) so the only schema additions live in keypoint-side interfaces.

**Tech Stack:** ROS2 Humble (rclpy), Ultralytics YOLO, OpenCV `solvePnP`, tf2_ros, colcon, ament_cmake.

**Spec:** `docs/superpowers/specs/2026-05-04-box-cup-pose-perception-design.md` (commit `d197200`).

---

## File Structure

| Path | Action | Responsibility |
|---|---|---|
| `src/omx_interfaces/msg/KeypointDetection.msg` | create | Per-detection class id/name, confidence, 4 keypoints. |
| `src/omx_interfaces/srv/GetKeypointDetections.srv` | create | Service contract for detection retrieval. |
| `src/omx_interfaces/msg/Top4Box.msg` | delete | Obsolete; replaced by `KeypointDetection`. |
| `src/omx_interfaces/srv/GetTop4Keypoints.srv` | delete | Obsolete; replaced by `GetKeypointDetections`. |
| `src/omx_interfaces/msg/BlockPose.msg` | modify | Update `color` comment (semantics extension). |
| `src/omx_interfaces/CMakeLists.txt` | modify | Swap msg/srv generation list. |
| `src/omx_perception/omx_perception/box_cup_pose_node.py` | create | YOLO inference + annotated image + keypoint service. |
| `src/omx_perception/omx_perception/box_cup_world_pose_node.py` | create | Class-aware PnP → `BlockPose[]`. |
| `src/omx_perception/omx_perception/top4_keypoints_node.py` | delete | Obsolete. |
| `src/omx_perception/omx_perception/top4_world_pose_node.py` | delete | Obsolete. |
| `src/omx_perception/setup.py` | modify | Console script entry points. |
| `src/omx_perception/launch/perception.launch.py` | modify | Args, executables, params, topics, services. |
| `src/omx_perception/test/test_box_cup_world_pose_helpers.py` | create | Unit tests for pure helper functions. |

Each task is self-contained: builds compile, no half-removed references after each commit.

---

## Task 1: Add `KeypointDetection.msg`

**Files:**
- Create: `src/omx_interfaces/msg/KeypointDetection.msg`

- [ ] **Step 1: Write the new message**

Write `src/omx_interfaces/msg/KeypointDetection.msg`:

```
# YOLO pose detection with class info.
# class_id: 0=Box, 1=Cup (matches model class index).
# class_name: "box" | "cup" (lowercased convenience field).
# detection_confidence: YOLO box score (0..1).
# keypoints: fixed layout for four YOLOv8-Pose keypoints,
#   class_id == CLASS_BOX → 큐브 윗면 4 corners (top-left, top-right, bottom-right, bottom-left)
#   class_id == CLASS_CUP → 림(rim) 4 cardinal pixel 위치
#   [k0_x, k0_y, k0_conf,
#    k1_x, k1_y, k1_conf,
#    k2_x, k2_y, k2_conf,
#    k3_x, k3_y, k3_conf]

uint8 CLASS_BOX = 0
uint8 CLASS_CUP = 1

uint8 class_id
string class_name
float32 detection_confidence
float32[12] keypoints
```

- [ ] **Step 2: Stage**

```bash
git add src/omx_interfaces/msg/KeypointDetection.msg
```

(No build yet — combine with the srv + CMakeLists change in Task 3.)

---

## Task 2: Add `GetKeypointDetections.srv`

**Files:**
- Create: `src/omx_interfaces/srv/GetKeypointDetections.srv`

- [ ] **Step 1: Write the new service**

Write `src/omx_interfaces/srv/GetKeypointDetections.srv`:

```
# Publish an annotated debug image for this service call.
bool publish_debug
---
std_msgs/Header header
bool success
string message
omx_interfaces/KeypointDetection[] detections
```

- [ ] **Step 2: Stage**

```bash
git add src/omx_interfaces/srv/GetKeypointDetections.srv
```

---

## Task 3: Register new interfaces and update `BlockPose` semantics

**Files:**
- Modify: `src/omx_interfaces/CMakeLists.txt`
- Modify: `src/omx_interfaces/msg/BlockPose.msg`

- [ ] **Step 1: Update `CMakeLists.txt`**

Replace the existing `rosidl_generate_interfaces` call (current full block — replace lines listing `msg/Top4Box.msg` and `srv/GetTop4Keypoints.srv` with the new names):

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToNamed.action"
  "action/MoveToPose.action"
  "action/MoveToJoints.action"
  "action/GripperCommand.action"
  "action/PickPlace.action"
  "action/PickDetected.action"
  "msg/BlockPose.msg"
  "msg/KeypointDetection.msg"
  "srv/GetBlockPoses.srv"
  "srv/GetKeypointDetections.srv"
  "srv/PlanToJoints.srv"
  "srv/ExecutePlan.srv"
  "srv/ClearPlan.srv"
  "srv/CheckGrasp.srv"
  DEPENDENCIES std_msgs geometry_msgs action_msgs builtin_interfaces
)
```

- [ ] **Step 2: Delete the old msg/srv files**

```bash
git rm src/omx_interfaces/msg/Top4Box.msg src/omx_interfaces/srv/GetTop4Keypoints.srv
```

- [ ] **Step 3: Update `BlockPose.msg` color comment**

Edit `src/omx_interfaces/msg/BlockPose.msg`:

Change line:
```
string color             # "red" | "blue" | "green"
```
to:
```
string color             # "box" | "cup"  (legacy: was "red" | "blue" | "green")
```

- [ ] **Step 4: Build interfaces**

```bash
cd /home/kjhz/omx_ws
colcon build --symlink-install --packages-select omx_interfaces
```

Expected: build succeeds. New types `omx_interfaces/msg/KeypointDetection` and `omx_interfaces/srv/GetKeypointDetections` are generated.

- [ ] **Step 5: Verify generated types**

```bash
source /home/kjhz/omx_ws/install/setup.bash
ros2 interface show omx_interfaces/msg/KeypointDetection
ros2 interface show omx_interfaces/srv/GetKeypointDetections
ros2 interface list | grep -E "Top4Box|GetTop4Keypoints"
```

Expected: first two commands print the new schemas. Third command prints nothing (old types gone).

- [ ] **Step 6: Commit**

```bash
git add src/omx_interfaces/CMakeLists.txt src/omx_interfaces/msg/BlockPose.msg src/omx_interfaces/msg/KeypointDetection.msg src/omx_interfaces/srv/GetKeypointDetections.srv
git commit -m "$(cat <<'EOF'
feat(interfaces): add KeypointDetection msg and GetKeypointDetections srv

Replace single-class Top4Box / GetTop4Keypoints with class-aware
KeypointDetection / GetKeypointDetections to support Box+Cup. Update
BlockPose.color comment to reflect new semantics ("box" | "cup").

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: Add `box_cup_pose_node.py`

**Files:**
- Create: `src/omx_perception/omx_perception/box_cup_pose_node.py`

- [ ] **Step 1: Write the new node**

Write `src/omx_perception/omx_perception/box_cup_pose_node.py`:

```python
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


CLASS_NAMES = {
    KeypointDetection.CLASS_BOX: "box",
    KeypointDetection.CLASS_CUP: "cup",
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

    def _extract_detections(self, result) -> list[KeypointDetection]:
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
            for keypoint_index in range(4):
                point = points[keypoint_index]
                confidence = 1.0
                if keypoints_conf is not None:
                    confidence = float(keypoints_conf[idx][keypoint_index])
                values.extend([float(point[0]), float(point[1]), confidence])
            det.keypoints = values

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
                cv2.putText(
                    image,
                    f"{det.class_name} {det_index}: {det.detection_confidence:.2f}",
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
    node = BoxCupPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

- [ ] **Step 2: Stage**

```bash
git add src/omx_perception/omx_perception/box_cup_pose_node.py
```

(No build yet — combine with setup.py + launch update.)

---

## Task 5: Add `box_cup_world_pose_node.py` with helper module

**Files:**
- Create: `src/omx_perception/omx_perception/box_cup_world_pose_node.py`

- [ ] **Step 1: Write the new node**

Write `src/omx_perception/omx_perception/box_cup_world_pose_node.py`:

```python
from __future__ import annotations

from pathlib import Path
from threading import Event

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from omx_interfaces.msg import BlockPose, KeypointDetection
from omx_interfaces.srv import GetBlockPoses, GetKeypointDetections
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


CLASS_COLOR = {
    KeypointDetection.CLASS_BOX: "box",
    KeypointDetection.CLASS_CUP: "cup",
}


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = float(np.sqrt(x * x + y * y + z * z + w * w))
    if norm == 0.0:
        return np.eye(3, dtype=np.float64)

    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def object_points_for_class(class_id: int, cube_size_m: float, cup_radius_m: float) -> np.ndarray:
    """Return the canonical 4x3 object-points matrix used for solvePnP for a class."""
    if class_id == KeypointDetection.CLASS_BOX:
        half = cube_size_m / 2.0
        return np.asarray(
            [
                [-half, -half, 0.0],
                [half, -half, 0.0],
                [half, half, 0.0],
                [-half, half, 0.0],
            ],
            dtype=np.float64,
        )
    if class_id == KeypointDetection.CLASS_CUP:
        r = cup_radius_m
        return np.asarray(
            [
                [-r, 0.0, 0.0],
                [0.0, -r, 0.0],
                [r, 0.0, 0.0],
                [0.0, r, 0.0],
            ],
            dtype=np.float64,
        )
    raise ValueError(f"Unsupported class_id for PnP: {class_id}")


class BoxCupWorldPoseNode(Node):
    """Convert YOLO pose keypoints into OMX-F world-frame poses for Box and Cup."""

    def __init__(self) -> None:
        super().__init__("box_cup_world_pose")

        self.declare_parameter("camera_intrinsics_path", "")
        self.declare_parameter(
            "keypoints_service_name", "/perception/get_box_cup_keypoints"
        )
        self.declare_parameter(
            "world_service_name", "/perception/get_box_cup_world_poses"
        )
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("camera_frame", "default_cam")
        self.declare_parameter("cube_size_m", 0.030)
        self.declare_parameter("box_output_z_m", 0.015)
        self.declare_parameter("cup_radius_m", 0.07)
        self.declare_parameter("cup_height_m", 0.08)
        self.declare_parameter("cup_output_z_m", 0.08)
        self.declare_parameter("min_keypoint_confidence", 0.10)
        self.declare_parameter("keypoints_timeout_sec", 2.0)
        # Image keypoint index that corresponds to each canonical model index.
        # For Box, model indices follow the cube top-face corners (TL, TR, BR, BL).
        # For Cup, model indices follow rim cardinal points; only the resulting
        # translation is used, so the rotation around the cup axis is not pinned.
        self.declare_parameter("keypoint_order", [0, 1, 2, 3])

        intrinsics_path = Path(
            str(self.get_parameter("camera_intrinsics_path").value)
        ).expanduser()
        if not intrinsics_path.is_absolute():
            intrinsics_path = Path.cwd() / intrinsics_path
        intrinsics_path = intrinsics_path.resolve()
        if not intrinsics_path.exists():
            raise FileNotFoundError(f"camera_intrinsics.yaml not found: {intrinsics_path}")

        self._camera_matrix, self._dist_coeffs = self._load_intrinsics(intrinsics_path)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._camera_frame = str(self.get_parameter("camera_frame").value)
        self._cube_size_m = float(self.get_parameter("cube_size_m").value)
        self._box_output_z_m = float(self.get_parameter("box_output_z_m").value)
        self._cup_radius_m = float(self.get_parameter("cup_radius_m").value)
        self._cup_output_z_m = float(self.get_parameter("cup_output_z_m").value)
        self._min_keypoint_confidence = float(
            self.get_parameter("min_keypoint_confidence").value
        )
        self._keypoints_timeout_sec = float(
            self.get_parameter("keypoints_timeout_sec").value
        )
        self._keypoint_order = [
            int(index) for index in self.get_parameter("keypoint_order").value
        ]
        if sorted(self._keypoint_order) != [0, 1, 2, 3]:
            raise ValueError("keypoint_order must contain each index 0, 1, 2, 3 exactly once")

        self._callback_group = ReentrantCallbackGroup()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        keypoints_service_name = str(self.get_parameter("keypoints_service_name").value)
        world_service_name = str(self.get_parameter("world_service_name").value)
        self._keypoints_client = self.create_client(
            GetKeypointDetections,
            keypoints_service_name,
            callback_group=self._callback_group,
        )
        self._world_service = self.create_service(
            GetBlockPoses,
            world_service_name,
            self._on_get_world_poses,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "box_cup world pose service ready "
            f"(keypoints_service={keypoints_service_name}, world_service={world_service_name}, "
            f"target_frame={self._target_frame}, "
            f"box_output_z_m={self._box_output_z_m:.3f}, "
            f"cup_output_z_m={self._cup_output_z_m:.3f})"
        )

    def _load_intrinsics(self, path: Path) -> tuple[np.ndarray, np.ndarray]:
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)

        camera_matrix = np.asarray(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.asarray(
            data.get("distortion_coefficients", {}).get("data", [0.0] * 5),
            dtype=np.float64,
        ).reshape(-1, 1)
        return camera_matrix, dist_coeffs

    def _on_get_world_poses(
        self,
        request: GetBlockPoses.Request,
        response: GetBlockPoses.Response,
    ) -> GetBlockPoses.Response:
        del request

        keypoints_response = self._call_keypoints_service()
        if keypoints_response is None or not keypoints_response.success:
            return response

        source_frame = keypoints_response.header.frame_id or self._camera_frame
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"failed to lookup transform {self._target_frame} <- {source_frame}: {exc}"
            )
            return response

        blocks: list[BlockPose] = []
        for det in keypoints_response.detections:
            block = self._detection_to_block_pose(det, keypoints_response.header, transform)
            if block is not None:
                blocks.append(block)

        response.blocks = blocks
        return response

    def _call_keypoints_service(self):
        if not self._keypoints_client.wait_for_service(
            timeout_sec=self._keypoints_timeout_sec
        ):
            self.get_logger().warning("box_cup keypoint service is not available")
            return None

        request = GetKeypointDetections.Request()
        request.publish_debug = True
        future = self._keypoints_client.call_async(request)
        done = Event()
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout=self._keypoints_timeout_sec):
            self.get_logger().warning("box_cup keypoint service call timed out")
            return None

        try:
            return future.result()
        except Exception as exc:
            self.get_logger().warning(f"box_cup keypoint service call failed: {exc}")
            return None

    def _detection_to_block_pose(
        self, det: KeypointDetection, header, transform
    ) -> BlockPose | None:
        image_points = []
        for keypoint_index in self._keypoint_order:
            base = keypoint_index * 3
            confidence = float(det.keypoints[base + 2])
            if confidence < self._min_keypoint_confidence:
                return None
            image_points.append([float(det.keypoints[base]), float(det.keypoints[base + 1])])

        try:
            object_points = object_points_for_class(
                det.class_id, self._cube_size_m, self._cup_radius_m
            )
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return None

        image_points_array = np.asarray(image_points, dtype=np.float64)

        flag = getattr(cv2, "SOLVEPNP_IPPE", cv2.SOLVEPNP_ITERATIVE)
        ok, _rvec, tvec = cv2.solvePnP(
            object_points,
            image_points_array,
            self._camera_matrix,
            self._dist_coeffs,
            flags=flag,
        )
        if not ok:
            return None

        center_camera = tvec.reshape(3)
        center_world = self._transform_point(center_camera, transform)

        if det.class_id == KeypointDetection.CLASS_BOX:
            output_z = self._box_output_z_m
        elif det.class_id == KeypointDetection.CLASS_CUP:
            output_z = self._cup_output_z_m
        else:
            self.get_logger().warning(
                f"unsupported class_id {det.class_id}; dropping detection"
            )
            return None

        pose = PoseStamped()
        pose.header.stamp = header.stamp
        pose.header.frame_id = self._target_frame
        pose.pose.position.x = float(center_world[0])
        pose.pose.position.y = float(center_world[1])
        pose.pose.position.z = output_z
        pose.pose.orientation.w = 1.0

        block = BlockPose()
        block.header = pose.header
        block.color = CLASS_COLOR.get(det.class_id, det.class_name or "unknown")
        block.pose = pose
        block.grasp_pose = pose
        block.confidence = float(det.detection_confidence)
        return block

    def _transform_point(self, point_xyz: np.ndarray, transform) -> np.ndarray:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = quaternion_to_rotation_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
        offset = np.asarray([translation.x, translation.y, translation.z], dtype=np.float64)
        return matrix @ point_xyz + offset


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoxCupWorldPoseNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

- [ ] **Step 2: Stage**

```bash
git add src/omx_perception/omx_perception/box_cup_world_pose_node.py
```

---

## Task 6: Add unit tests for the world-pose helper

**Files:**
- Create: `src/omx_perception/test/__init__.py`
- Create: `src/omx_perception/test/test_box_cup_world_pose_helpers.py`

- [ ] **Step 1: Create test package init**

Write empty `src/omx_perception/test/__init__.py` (zero bytes).

- [ ] **Step 2: Write failing tests**

Write `src/omx_perception/test/test_box_cup_world_pose_helpers.py`:

```python
"""Pure-function unit tests for box_cup_world_pose_node helpers.

These tests do not initialise rclpy; they only import the module-level
helpers so they can run as a stand-alone pytest invocation.
"""
from __future__ import annotations

import numpy as np
import pytest
from omx_interfaces.msg import KeypointDetection

from omx_perception.box_cup_world_pose_node import (
    object_points_for_class,
    quaternion_to_rotation_matrix,
)


def test_object_points_box_returns_square_corners():
    cube_size_m = 0.030
    pts = object_points_for_class(
        KeypointDetection.CLASS_BOX, cube_size_m=cube_size_m, cup_radius_m=0.07
    )
    expected = np.asarray(
        [
            [-0.015, -0.015, 0.0],
            [0.015, -0.015, 0.0],
            [0.015, 0.015, 0.0],
            [-0.015, 0.015, 0.0],
        ],
        dtype=np.float64,
    )
    assert pts.shape == (4, 3)
    assert np.allclose(pts, expected)


def test_object_points_cup_returns_rim_cardinals():
    pts = object_points_for_class(
        KeypointDetection.CLASS_CUP, cube_size_m=0.030, cup_radius_m=0.07
    )
    expected = np.asarray(
        [
            [-0.07, 0.0, 0.0],
            [0.0, -0.07, 0.0],
            [0.07, 0.0, 0.0],
            [0.0, 0.07, 0.0],
        ],
        dtype=np.float64,
    )
    assert pts.shape == (4, 3)
    assert np.allclose(pts, expected)


def test_object_points_unknown_class_raises():
    with pytest.raises(ValueError):
        object_points_for_class(class_id=99, cube_size_m=0.030, cup_radius_m=0.07)


def test_quaternion_identity_returns_eye():
    matrix = quaternion_to_rotation_matrix(0.0, 0.0, 0.0, 1.0)
    assert np.allclose(matrix, np.eye(3))


def test_quaternion_z_180_flips_x_and_y():
    matrix = quaternion_to_rotation_matrix(0.0, 0.0, 1.0, 0.0)
    point = np.asarray([1.0, 2.0, 3.0])
    transformed = matrix @ point
    assert np.allclose(transformed, [-1.0, -2.0, 3.0])
```

- [ ] **Step 3: Run tests (must fail because module not yet on path)**

```bash
cd /home/kjhz/omx_ws
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
python3 -m pytest src/omx_perception/test/test_box_cup_world_pose_helpers.py -v
```

Expected (before Task 7's setup.py + colcon build): collection error or `ModuleNotFoundError: No module named 'omx_perception.box_cup_world_pose_node'` — that's the failing-test state we want.

- [ ] **Step 4: Stage tests**

```bash
git add src/omx_perception/test/__init__.py src/omx_perception/test/test_box_cup_world_pose_helpers.py
```

---

## Task 7: Update `setup.py` and remove old nodes

**Files:**
- Modify: `src/omx_perception/setup.py`
- Delete: `src/omx_perception/omx_perception/top4_keypoints_node.py`
- Delete: `src/omx_perception/omx_perception/top4_world_pose_node.py`

- [ ] **Step 1: Edit `setup.py` console_scripts**

Replace the `entry_points` block in `src/omx_perception/setup.py`:

```python
    entry_points={
        "console_scripts": [
            "camera_control_node = omx_perception.camera_control_node:main",
            "box_cup_pose_node = omx_perception.box_cup_pose_node:main",
            "box_cup_world_pose_node = omx_perception.box_cup_world_pose_node:main",
        ],
    },
```

- [ ] **Step 2: Delete old node files**

```bash
git rm src/omx_perception/omx_perception/top4_keypoints_node.py src/omx_perception/omx_perception/top4_world_pose_node.py
```

- [ ] **Step 3: Build the perception package**

```bash
cd /home/kjhz/omx_ws
colcon build --symlink-install --packages-select omx_perception
```

Expected: build succeeds without warnings about missing `top4_*` modules.

- [ ] **Step 4: Run unit tests (must pass now)**

```bash
source /home/kjhz/omx_ws/install/setup.bash
python3 -m pytest src/omx_perception/test/test_box_cup_world_pose_helpers.py -v
```

Expected: `5 passed`.

- [ ] **Step 5: Commit nodes + setup + tests**

```bash
git add src/omx_perception/setup.py src/omx_perception/omx_perception/box_cup_pose_node.py src/omx_perception/omx_perception/box_cup_world_pose_node.py src/omx_perception/test/__init__.py src/omx_perception/test/test_box_cup_world_pose_helpers.py
git commit -m "$(cat <<'EOF'
feat(perception): add box_cup pose and world-pose nodes

Replace top4_keypoints_node / top4_world_pose_node with class-aware
box_cup_pose_node and box_cup_world_pose_node. Box uses cube-top-corner
PnP (z=box_output_z_m); Cup uses rim-cardinal PnP with cup_radius_m
(z=cup_output_z_m=0.08, the drop target). World-pose response uses
BlockPose.color="box"|"cup" so downstream pick&place can split targets.

Add pure-function unit tests for solvePnP object_points construction
and quaternion-to-rotation conversion.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: Update `perception.launch.py`

**Files:**
- Modify: `src/omx_perception/launch/perception.launch.py`

- [ ] **Step 1: Replace launch file**

Overwrite `src/omx_perception/launch/perception.launch.py` with:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_params = PathJoinSubstitution([
        FindPackageShare("omx_perception"),
        "config",
        "camera_params.yaml",
    ])
    camera_intrinsics = PathJoinSubstitution([
        FindPackageShare("omx_perception"),
        "config",
        "camera_intrinsics.yaml",
    ])

    box_cup_model_path_arg = DeclareLaunchArgument(
        "box_cup_model_path",
        default_value="/home/kjhz/omx_ws/runs/pose/box_cup_pose_2class_96/weights/best.pt",
        description="Absolute path to the trained YOLOv8-Pose 2-class best.pt.",
    )
    box_cup_device_arg = DeclareLaunchArgument(
        "box_cup_device",
        default_value="0",
        description="Ultralytics inference device for box_cup_pose_node.",
    )
    box_cup_conf_arg = DeclareLaunchArgument(
        "box_cup_conf",
        default_value="0.85",
        description="YOLO confidence threshold for box_cup_pose_node.",
    )
    box_cup_extra_pythonpath_arg = DeclareLaunchArgument(
        "box_cup_extra_pythonpath",
        default_value="/home/kjhz/miniconda3/envs/driving/lib/python3.12/site-packages",
        description="Optional site-packages path that contains ultralytics and torch.",
    )

    camera_control = Node(
        package="omx_perception",
        executable="camera_control_node",
        name="camera_control",
        namespace="camera",
        output="screen",
        parameters=[camera_params],
    )

    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        output="both",
        parameters=[camera_params],
        remappings=[
            ("image_raw", "/image/raw"),
            ("image_raw/compressed", "/image/raw/compressed"),
            ("image_raw/compressedDepth", "/image/raw/compressedDepth"),
            ("image_raw/theora", "/image/raw/theora"),
            ("image_raw/zstd", "/image/raw/zstd"),
            ("camera_info", "/camera/info"),
        ],
    )

    box_cup_pose = Node(
        package="omx_perception",
        executable="box_cup_pose_node",
        name="box_cup_pose",
        output="screen",
        parameters=[
            {
                "model_path": LaunchConfiguration("box_cup_model_path"),
                "image_topic": "/image/raw",
                "annotated_image_topic": "/image/raw/box_cup_pose",
                "service_name": "/perception/get_box_cup_keypoints",
                "extra_pythonpath": LaunchConfiguration("box_cup_extra_pythonpath"),
                "device": ParameterValue(LaunchConfiguration("box_cup_device"), value_type=str),
                "conf": ParameterValue(LaunchConfiguration("box_cup_conf"), value_type=float),
                "imgsz": 640,
                "max_det": 20,
            }
        ],
    )

    box_cup_world_pose = Node(
        package="omx_perception",
        executable="box_cup_world_pose_node",
        name="box_cup_world_pose",
        output="screen",
        parameters=[
            {
                "camera_intrinsics_path": camera_intrinsics,
                "keypoints_service_name": "/perception/get_box_cup_keypoints",
                "world_service_name": "/perception/get_box_cup_world_poses",
                "target_frame": "world",
                "camera_frame": "default_cam",
                "cube_size_m": 0.030,
                "box_output_z_m": 0.015,
                "cup_radius_m": 0.07,
                "cup_height_m": 0.08,
                "cup_output_z_m": 0.08,
                "min_keypoint_confidence": 0.10,
                "keypoints_timeout_sec": 2.0,
                "keypoint_order": [0, 1, 2, 3],
            }
        ],
    )

    delayed_camera_and_perception = TimerAction(
        period=0.5,
        actions=[camera_node, box_cup_pose, box_cup_world_pose],
    )

    return LaunchDescription([
        box_cup_model_path_arg,
        box_cup_device_arg,
        box_cup_conf_arg,
        box_cup_extra_pythonpath_arg,
        camera_control,
        delayed_camera_and_perception,
    ])
```

- [ ] **Step 2: Build to install the new launch file**

```bash
cd /home/kjhz/omx_ws
colcon build --symlink-install --packages-select omx_perception
```

Expected: build succeeds.

- [ ] **Step 3: Verify launch parses**

```bash
source /home/kjhz/omx_ws/install/setup.bash
ros2 launch -p omx_perception perception.launch.py 2>&1 | head -30
```

(Use `-p` to print the launch description without executing.)

Expected: prints node descriptions for `camera_control`, `usb_cam`, `box_cup_pose`, `box_cup_world_pose`. No reference to `top4_*`. No syntax errors.

- [ ] **Step 4: Commit**

```bash
git add src/omx_perception/launch/perception.launch.py
git commit -m "$(cat <<'EOF'
feat(perception): switch perception.launch to box_cup pipeline

Update launch arguments, executables, topics, services, and parameters
to drive box_cup_pose_node and box_cup_world_pose_node with the new
2-class YOLOv8-Pose model.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>
EOF
)"
```

---

## Task 9: Workspace-wide cleanup verification

**Files:**
- (Verification only.)

- [ ] **Step 1: Verify no stale references in source**

```bash
cd /home/kjhz/omx_ws
grep -rn "Top4Box\|GetTop4Keypoints\|top4_pose\|get_top4_world_poses\|top4_keypoints\|top4_world_pose\|get_top4_keypoints" \
  --include="*.py" --include="*.cpp" --include="*.hpp" --include="*.h" \
  --include="*.launch.py" --include="*.yaml" --include="*.xml" \
  --include="*.cmake" --include="CMakeLists.txt" \
  src/
```

Expected: no output (the build artifacts under `install/` and `build/` are not searched because the search is scoped to `src/`).

- [ ] **Step 2: Full workspace build**

```bash
cd /home/kjhz/omx_ws
colcon build --symlink-install
```

Expected: every package builds. `omx_skill_executor`, `omx_motion_server`, and `omx_bringup` are unchanged but still compile against the updated `omx_interfaces` (they only use `BlockPose` / `GetBlockPoses` which kept their schema).

- [ ] **Step 3: Re-run unit tests after full build**

```bash
source /home/kjhz/omx_ws/install/setup.bash
python3 -m pytest src/omx_perception/test/test_box_cup_world_pose_helpers.py -v
```

Expected: `5 passed`.

- [ ] **Step 4: List new ROS interfaces**

```bash
ros2 interface list | grep -E "omx_interfaces/(msg|srv)"
```

Expected output contains:
- `omx_interfaces/msg/BlockPose`
- `omx_interfaces/msg/KeypointDetection`
- `omx_interfaces/srv/GetBlockPoses`
- `omx_interfaces/srv/GetKeypointDetections`

Does NOT contain `Top4Box` or `GetTop4Keypoints`.

- [ ] **Step 5: Manual launch sanity-check (optional, hardware dependent)**

If a USB camera and the trained model are available:

```bash
source /home/kjhz/omx_ws/install/setup.bash
ros2 launch omx_perception perception.launch.py
```

In another shell, verify:
```bash
ros2 topic list | grep box_cup_pose
# Expect: /image/raw/box_cup_pose

ros2 service call /perception/get_box_cup_keypoints \
  omx_interfaces/srv/GetKeypointDetections "{publish_debug: true}"
# Expect: success: True, detections: [...] with class_id 0 (box) and 1 (cup) populated.

ros2 service call /perception/get_box_cup_world_poses \
  omx_interfaces/srv/GetBlockPoses "{color: ''}"
# Expect: blocks: [...] with color "box" and "cup" entries (when both are visible).

rqt_image_view /image/raw/box_cup_pose
# Expect: box detections with quad outline + label "box N: <conf>"; cup detections with orange dots + label "cup N: <conf>".
```

If hardware is not available, document that this step was skipped in the task PR/branch notes.

- [ ] **Step 6: Final commit (only if Step 1 grep produced output that needed fixing)**

If Step 1 was clean and Step 2/3 passed, no extra commit is needed; the implementation work is already committed across Tasks 3, 7, and 8.

---

## Self-Review

**1. Spec coverage:**
- §3.1 KeypointDetection.msg → Task 1 ✓
- §3.2 GetKeypointDetections.srv → Task 2 ✓
- §3.3 BlockPose.msg comment → Task 3 ✓
- §3.4 CMakeLists.txt swap + delete old files → Task 3 ✓
- §4.1/4.2 file/exec/topic/service rename → Tasks 4, 5, 7, 8 ✓
- §4.3 box_cup_pose_node behaviour (class extraction, dual color drawing, polygon for box only) → Task 4 ✓
- §4.4 class-aware PnP, BlockPose.color, drop-target z for cup → Task 5 ✓
- §4.5 new parameters → Task 5 (and consumed in Task 8 launch) ✓
- §5.1 setup.py update → Task 7 ✓
- §5.2 launch update → Task 8 ✓
- §7 verification (build, grep, runtime) → Task 9 ✓

**2. Placeholder scan:** No "TBD"/"TODO"/"add appropriate"/"similar to". Every code step has full code. Every command step has the exact command and expected output.

**3. Type consistency:**
- `KeypointDetection.CLASS_BOX` / `CLASS_CUP` constants used identically across the pose node, world-pose node, and tests.
- `GetKeypointDetections.Request.publish_debug` consumed in pose node and produced in world-pose node — matches.
- World-pose node parameter `keypoints_service_name` matches the launch parameter and the pose node's `service_name`.
- `object_points_for_class(class_id, cube_size_m, cup_radius_m)` signature matches its three call sites (helper definition, internal call inside `_detection_to_block_pose`, and unit tests).
- `BlockPose.color` set in `CLASS_COLOR` mapping (`{0:"box", 1:"cup"}`) — consistent with spec §4.4.
