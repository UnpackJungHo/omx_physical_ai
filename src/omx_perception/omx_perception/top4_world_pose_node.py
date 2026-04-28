from __future__ import annotations

from pathlib import Path
from threading import Event

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from omx_interfaces.msg import BlockPose, Top4Box
from omx_interfaces.srv import GetBlockPoses, GetTop4Keypoints
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


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


class Top4WorldPoseNode(Node):
    """Convert YOLO top-face keypoints into OMX-F world-frame x/y poses."""

    def __init__(self) -> None:
        super().__init__("top4_world_pose")

        self.declare_parameter("camera_intrinsics_path", "")
        self.declare_parameter("top4_service_name", "/perception/get_top4_keypoints")
        self.declare_parameter("world_service_name", "/perception/get_top4_world_poses")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("camera_frame", "default_cam")
        self.declare_parameter("cube_size_m", 0.030)
        # 30 mm cube center height is 15 mm = 1.5 cm = 0.015 m.
        self.declare_parameter("output_z_m", 0.015)
        self.declare_parameter("min_keypoint_confidence", 0.10)
        self.declare_parameter("top4_timeout_sec", 2.0)
        # Image keypoint index that corresponds to each canonical square corner.
        # Canonical order: top-left, top-right, bottom-right, bottom-left on the
        # cube top face as labeled in the training dataset.
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
        self._output_z_m = float(self.get_parameter("output_z_m").value)
        self._min_keypoint_confidence = float(
            self.get_parameter("min_keypoint_confidence").value
        )
        self._top4_timeout_sec = float(self.get_parameter("top4_timeout_sec").value)
        self._keypoint_order = [
            int(index) for index in self.get_parameter("keypoint_order").value
        ]
        if sorted(self._keypoint_order) != [0, 1, 2, 3]:
            raise ValueError("keypoint_order must contain each index 0, 1, 2, 3 exactly once")

        self._callback_group = ReentrantCallbackGroup()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        top4_service_name = str(self.get_parameter("top4_service_name").value)
        world_service_name = str(self.get_parameter("world_service_name").value)
        self._top4_client = self.create_client(
            GetTop4Keypoints,
            top4_service_name,
            callback_group=self._callback_group,
        )
        self._world_service = self.create_service(
            GetBlockPoses,
            world_service_name,
            self._on_get_world_poses,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "top4 world pose service ready "
            f"(top4_service={top4_service_name}, world_service={world_service_name}, "
            f"target_frame={self._target_frame}, output_z_m={self._output_z_m:.3f})"
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

        top4_response = self._call_top4_service()
        if top4_response is None or not top4_response.success:
            return response

        source_frame = top4_response.header.frame_id or self._camera_frame
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
        for box in top4_response.boxes:
            block = self._box_to_block_pose(box, top4_response.header, transform)
            if block is not None:
                blocks.append(block)

        response.blocks = blocks
        return response

    def _call_top4_service(self):
        if not self._top4_client.wait_for_service(timeout_sec=self._top4_timeout_sec):
            self.get_logger().warning("top4 keypoint service is not available")
            return None

        request = GetTop4Keypoints.Request()
        request.publish_debug = True
        future = self._top4_client.call_async(request)
        done = Event()
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout=self._top4_timeout_sec):
            self.get_logger().warning("top4 keypoint service call timed out")
            return None

        try:
            return future.result()
        except Exception as exc:
            self.get_logger().warning(f"top4 keypoint service call failed: {exc}")
            return None

    def _box_to_block_pose(self, box: Top4Box, header, transform) -> BlockPose | None:
        image_points = []
        for keypoint_index in self._keypoint_order:
            base = keypoint_index * 3
            confidence = float(box.keypoints[base + 2])
            if confidence < self._min_keypoint_confidence:
                return None
            image_points.append([float(box.keypoints[base]), float(box.keypoints[base + 1])])

        half = self._cube_size_m / 2.0
        object_points = np.asarray(
            [
                [-half, -half, 0.0],
                [half, -half, 0.0],
                [half, half, 0.0],
                [-half, half, 0.0],
            ],
            dtype=np.float64,
        )
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

        pose = PoseStamped()
        pose.header.stamp = header.stamp
        pose.header.frame_id = self._target_frame
        pose.pose.position.x = float(center_world[0])
        pose.pose.position.y = float(center_world[1])
        pose.pose.position.z = self._output_z_m
        pose.pose.orientation.w = 1.0

        block = BlockPose()
        block.header = pose.header
        block.color = "top4"
        block.pose = pose
        block.grasp_pose = pose
        block.confidence = float(box.detection_confidence)
        return block

    def _transform_point(self, point_xyz: np.ndarray, transform) -> np.ndarray:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = quaternion_to_rotation_matrix(rotation.x, rotation.y, rotation.z, rotation.w)
        offset = np.asarray([translation.x, translation.y, translation.z], dtype=np.float64)
        return matrix @ point_xyz + offset


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Top4WorldPoseNode()
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
