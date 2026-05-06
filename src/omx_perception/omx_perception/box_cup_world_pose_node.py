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
        # detection 이 캡처된 시점의 TF 를 사용해 카메라가 움직이는 동안에도
        # 좌표변환과 이미지 좌표가 동일 시점으로 일치하도록 한다.
        # stamp 가 비어있는 경우 (sec=nanosec=0) 만 latest 로 fallback.
        header_stamp = keypoints_response.header.stamp
        use_latest = header_stamp.sec == 0 and header_stamp.nanosec == 0
        lookup_time = Time() if use_latest else Time.from_msg(header_stamp)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._target_frame,
                source_frame,
                lookup_time,
                timeout=Duration(seconds=0.5),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"failed to lookup transform {self._target_frame} <- {source_frame} "
                f"at stamp={'latest' if use_latest else f'{header_stamp.sec}.{header_stamp.nanosec:09d}'}: {exc}"
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

        _, tvec = self._solve_pnp_with_fallback(object_points, image_points_array)
        if tvec is None:
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
        if det.class_id == KeypointDetection.CLASS_CUP:
            block.color = "cup"
        else:
            block.color = det.color if det.color else "unknown"
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

    def _solve_pnp_with_fallback(
        self,
        object_points: np.ndarray,
        image_points: np.ndarray,
    ) -> tuple[bool, np.ndarray | None]:
        flags = [getattr(cv2, "SOLVEPNP_IPPE", cv2.SOLVEPNP_ITERATIVE)]
        if flags[0] != cv2.SOLVEPNP_ITERATIVE:
            flags.append(cv2.SOLVEPNP_ITERATIVE)

        for flag in flags:
            ok, _rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self._camera_matrix,
                self._dist_coeffs,
                flags=flag,
            )
            if ok:
                return True, tvec

        return False, None


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
