from __future__ import annotations

import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from omx_perception.camera_geometry import CameraIntrinsics, TransformSnapshot
from omx_perception.perception_pipeline import (
    ColorPrototype,
    DetectorSettings,
    WorkspaceRect,
    build_undistort_maps,
    compute_confidence,
    draw_debug,
    draw_reject,
    estimate_block_pose,
    gray_world_white_balance,
    label_color_rect,
    load_color_prototypes,
    segment_foreground,
)


def _latest_stream_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )


class DetectorNode(Node):
    """ROS wrapper around the current saturation + rect-pose pipeline."""

    def __init__(self) -> None:
        super().__init__("detector_node")

        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("camera_frame", "default_cam")
        self.declare_parameter("block_size_m", 0.030)
        self.declare_parameter("table_z_m", 0.18)
        self.declare_parameter("workspace_x_min_m", 0.08)
        self.declare_parameter("workspace_x_max_m", 0.35)
        self.declare_parameter("workspace_y_min_m", -0.15)
        self.declare_parameter("workspace_y_max_m", 0.15)
        self.declare_parameter("color_chroma_min", 10.0)
        self.declare_parameter("color_majority_min", 0.40)
        self.declare_parameter("canny_sigma", 0.33)
        self.declare_parameter("min_contour_area_px", 200.0)
        self.declare_parameter("max_contour_area_px", 20000.0)
        self.declare_parameter("saturation_min", 40)
        self.declare_parameter("rect_fill_min", 0.75)
        self.declare_parameter("aspect_ratio_min", 0.60)
        self.declare_parameter("aspect_ratio_max", 1.66)
        self.declare_parameter("use_white_balance", True)
        self.declare_parameter("color_prototypes_path", "")
        self.declare_parameter("process_period_sec", 0.02)
        self.declare_parameter("publish_debug_canny", False)
        self.declare_parameter("publish_debug_segmentation", False)
        self.declare_parameter("slow_frame_warn_ms", 80.0)

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._intrinsics: CameraIntrinsics | None = None
        self._undistort_map_1: np.ndarray | None = None
        self._undistort_map_2: np.ndarray | None = None
        self._image_size: tuple[int, int] | None = None

        self._prototypes: list[ColorPrototype] = self._load_prototypes()
        self._latest_image_msg: Image | None = None
        self._received_frames = 0
        self._processed_frames = 0
        self._overwritten_frames = 0
        self._last_processing_ms = 0.0
        self._last_slow_log_at = 0.0

        stream_qos = _latest_stream_qos()

        self.create_subscription(CameraInfo, "/camera_info", self._on_camera_info, 10)
        self.create_subscription(Image, "/image/raw", self._on_image, stream_qos)
        self._blocks_pub = self.create_publisher(String, "/omx/perception/blocks", stream_qos)
        self._debug_pub = self.create_publisher(Image, "/omx/perception/debug_image", stream_qos)
        self._canny_pub = self.create_publisher(Image, "/omx/perception/debug_canny", stream_qos)
        self._seg_pub = self.create_publisher(Image, "/omx/perception/debug_segmentation", stream_qos)
        self.create_timer(
            float(self.get_parameter("process_period_sec").value),
            self._process_latest_frame,
        )
        self.get_logger().info(
            "detector pipeline mode=latest_frame_rect_pose_v2 "
            "(fast path: minAreaRect + workspace + color)"
        )

    def _load_prototypes(self) -> list[ColorPrototype]:
        path = str(self.get_parameter("color_prototypes_path").value)
        if not path:
            self.get_logger().warn("color_prototypes_path empty — color labeling disabled")
            return []
        try:
            with Path(path).open("r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
        except OSError as exc:
            self.get_logger().warn(f"Failed to read prototypes '{path}': {exc}")
            return []
        prototypes = load_color_prototypes(data)
        self.get_logger().info(
            f"Loaded color prototypes: {[p.name for p in prototypes]} from '{path}'"
        )
        return prototypes

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if msg.k[0] == 0.0:
            return
        dist = tuple(float(v) for v in msg.d) if msg.d else (0.0, 0.0, 0.0, 0.0, 0.0)
        self._intrinsics = CameraIntrinsics(
            fx=float(msg.k[0]),
            fy=float(msg.k[4]),
            cx=float(msg.k[2]),
            cy=float(msg.k[5]),
            frame_id=msg.header.frame_id,
            dist_coeffs=dist,
        )

    def _ensure_undistort_maps(self, width: int, height: int) -> None:
        if self._intrinsics is None:
            return
        if self._image_size == (width, height) and self._undistort_map_1 is not None:
            return
        map1, map2 = build_undistort_maps(self._intrinsics, (width, height))
        self._undistort_map_1 = map1
        self._undistort_map_2 = map2
        self._image_size = (width, height)

    def _pipeline_intrinsics(self) -> CameraIntrinsics:
        assert self._intrinsics is not None
        return CameraIntrinsics(
            fx=self._intrinsics.fx,
            fy=self._intrinsics.fy,
            cx=self._intrinsics.cx,
            cy=self._intrinsics.cy,
            frame_id=self._intrinsics.frame_id,
            dist_coeffs=(0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def _lookup_transform(self, ref: str, cam: str) -> TransformSnapshot | None:
        try:
            tf = self._tf_buffer.lookup_transform(ref, cam, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return TransformSnapshot(
            reference_frame=ref,
            translation_m=np.array([t.x, t.y, t.z]),
            rotation_xyzw=np.array([r.x, r.y, r.z, r.w]),
        )

    def _on_image(self, msg: Image) -> None:
        self._received_frames += 1
        if self._latest_image_msg is not None:
            self._overwritten_frames += 1
        self._latest_image_msg = msg

    def _process_latest_frame(self) -> None:
        if self._intrinsics is None:
            return
        msg = self._latest_image_msg
        if msg is None:
            return
        self._latest_image_msg = None
        started_at = time.perf_counter()

        ref_frame = str(self.get_parameter("reference_frame").value)
        cam_frame = str(self.get_parameter("camera_frame").value)
        block_size = float(self.get_parameter("block_size_m").value)
        table_z = float(self.get_parameter("table_z_m").value)
        workspace = WorkspaceRect(
            x_min_m=float(self.get_parameter("workspace_x_min_m").value),
            x_max_m=float(self.get_parameter("workspace_x_max_m").value),
            y_min_m=float(self.get_parameter("workspace_y_min_m").value),
            y_max_m=float(self.get_parameter("workspace_y_max_m").value),
            plane_z_m=table_z + block_size,
        )
        settings = DetectorSettings(
            block_size_m=block_size,
            color_chroma_min=float(self.get_parameter("color_chroma_min").value),
            color_majority_min=float(self.get_parameter("color_majority_min").value),
            canny_sigma=float(self.get_parameter("canny_sigma").value),
            min_contour_area_px=float(self.get_parameter("min_contour_area_px").value),
            max_contour_area_px=float(self.get_parameter("max_contour_area_px").value),
            saturation_min=int(self.get_parameter("saturation_min").value),
            rect_fill_min=float(self.get_parameter("rect_fill_min").value),
            aspect_ratio_min=float(self.get_parameter("aspect_ratio_min").value),
            aspect_ratio_max=float(self.get_parameter("aspect_ratio_max").value),
        )

        transform = self._lookup_transform(ref_frame, cam_frame)
        if transform is None:
            return

        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = bgr.shape[:2]
        self._ensure_undistort_maps(w, h)
        if self._undistort_map_1 is not None:
            bgr = cv2.remap(
                bgr, self._undistort_map_1, self._undistort_map_2, cv2.INTER_LINEAR
            )
        if bool(self.get_parameter("use_white_balance").value):
            bgr = gray_world_white_balance(bgr)

        intrinsics = self._pipeline_intrinsics()
        contours, edges, filled = segment_foreground(
            bgr,
            settings.canny_sigma,
            saturation_min=settings.saturation_min,
            compute_edges=bool(self.get_parameter("publish_debug_canny").value),
        )

        debug_img = bgr.copy()
        detections: list[dict] = []
        stats = {"total": 0, "too_small": 0, "too_big": 0, "accepted": 0}
        reject_counts: dict[str, int] = {}

        for contour in contours:
            stats["total"] += 1
            area = float(cv2.contourArea(contour))
            if area < settings.min_contour_area_px:
                stats["too_small"] += 1
                continue
            if area > settings.max_contour_area_px:
                stats["too_big"] += 1
                draw_reject(debug_img, contour, "too_big", f"a={int(area)}")
                continue
            detection, reason = self._process_contour(
                contour, bgr, debug_img, intrinsics, transform,
                workspace, table_z, settings,
            )
            if detection is not None:
                detections.append(detection)
                stats["accepted"] += 1
            else:
                reject_counts[reason] = reject_counts.get(reason, 0) + 1

        self._last_processing_ms = (time.perf_counter() - started_at) * 1000.0
        self._processed_frames += 1
        self._warn_on_slow_frame()
        self._draw_frame_summary(debug_img, stats, reject_counts, len(self._prototypes))

        stamp = msg.header.stamp
        self._blocks_pub.publish(
            String(
                data=json.dumps(
                    {
                        "stamp_sec": stamp.sec,
                        "stamp_nanosec": stamp.nanosec,
                        "frame_id": ref_frame,
                        "blocks": detections,
                    }
                )
            )
        )
        debug_msg = self._bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
        debug_msg.header = msg.header
        self._debug_pub.publish(debug_msg)

        if bool(self.get_parameter("publish_debug_canny").value) and edges is not None:
            canny_msg = self._bridge.cv2_to_imgmsg(edges, encoding="mono8")
            canny_msg.header = msg.header
            self._canny_pub.publish(canny_msg)

        if bool(self.get_parameter("publish_debug_segmentation").value):
            seg_vis = cv2.cvtColor(filled, cv2.COLOR_GRAY2BGR)
            cv2.drawContours(seg_vis, contours, -1, (0, 255, 0), 1)
            seg_msg = self._bridge.cv2_to_imgmsg(seg_vis, encoding="bgr8")
            seg_msg.header = msg.header
            self._seg_pub.publish(seg_msg)

    def _warn_on_slow_frame(self) -> None:
        warn_ms = float(self.get_parameter("slow_frame_warn_ms").value)
        if self._last_processing_ms <= warn_ms:
            return
        now = time.monotonic()
        if (now - self._last_slow_log_at) < 2.0:
            return
        self._last_slow_log_at = now
        self.get_logger().warn(
            f"slow perception frame: {self._last_processing_ms:.1f} ms "
            f"(rx={self._received_frames} proc={self._processed_frames} "
            f"overwrite={self._overwritten_frames})"
        )

    def _draw_frame_summary(
        self,
        image: np.ndarray,
        stats: dict,
        reject_counts: dict,
        proto_count: int,
    ) -> None:
        lines = [
            f"frames rx={self._received_frames} proc={self._processed_frames} "
            f"overwrite={self._overwritten_frames}",
            f"proc={self._last_processing_ms:.1f}ms mode=rect_pose_fast",
            f"contours={stats['total']} small={stats['too_small']} "
            f"big={stats['too_big']} ok={stats['accepted']}",
            f"prototypes={proto_count}",
        ]
        if reject_counts:
            top = sorted(reject_counts.items(), key=lambda kv: -kv[1])
            lines.append("rejects: " + ", ".join(f"{k}={v}" for k, v in top[:4]))
        for i, line in enumerate(lines):
            y = 18 + i * 18
            cv2.putText(
                image, line, (8, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3,
            )
            cv2.putText(
                image, line, (8, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
            )

    def _process_contour(
        self,
        contour: np.ndarray,
        bgr: np.ndarray,
        debug_img: np.ndarray,
        intrinsics: CameraIntrinsics,
        transform: TransformSnapshot,
        workspace: WorkspaceRect,
        table_z: float,
        settings: DetectorSettings,
    ) -> tuple[dict | None, str]:
        area = float(cv2.contourArea(contour))
        estimate, reason = estimate_block_pose(
            contour,
            intrinsics,
            transform,
            block_size_m=settings.block_size_m,
            table_z_m=table_z,
            rect_fill_min=settings.rect_fill_min,
            aspect_ratio_min=settings.aspect_ratio_min,
            aspect_ratio_max=settings.aspect_ratio_max,
        )
        if estimate is None:
            draw_reject(debug_img, contour, reason, f"a={int(area)}")
            return None, reason

        if not (
            workspace.x_min_m <= estimate.x_world <= workspace.x_max_m
            and workspace.y_min_m <= estimate.y_world <= workspace.y_max_m
            and (table_z - 0.02) <= estimate.z_world <= (table_z + settings.block_size_m + 0.02)
        ):
            reason = "outside_ws"
            draw_reject(
                debug_img,
                contour,
                reason,
                f"xyz=({estimate.x_world:.2f},{estimate.y_world:.2f},{estimate.z_world:.2f})",
            )
            return None, reason

        color_name, color_ratio = label_color_rect(
            bgr, estimate.rect_corners_px.astype(np.float32), self._prototypes,
            chroma_min=settings.color_chroma_min,
            majority_min=settings.color_majority_min,
        )
        if color_name is None:
            reason = "color_none_proto" if not self._prototypes else "color_fail"
            draw_reject(
                debug_img, contour, reason,
                f"fill={estimate.rect_fill:.2f} r={color_ratio:.2f}",
            )
            return None, reason

        confidence = compute_confidence(
            estimate.rect_fill,
            settings.rect_fill_min,
            estimate.aspect_ratio,
            color_ratio,
            settings.color_majority_min,
        )

        label_text = (
            f"{color_name} c={confidence:.2f} "
            f"xy=({estimate.x_world*100:.1f},{estimate.y_world*100:.1f})cm "
            f"fill={estimate.rect_fill:.2f}"
        )
        draw_debug(
            debug_img, contour, estimate.rect_corners_px.astype(np.float32),
            color_name, label_text, center_px=estimate.center_px,
        )

        return (
            {
                "color": color_name,
                "x": estimate.x_world,
                "y": estimate.y_world,
                "z": estimate.z_world,
                "yaw": estimate.yaw_world,
                "confidence": float(confidence),
                "pixel_u": float(estimate.pixel_u),
                "pixel_v": float(estimate.pixel_v),
                "source": "rect_pose",
                "rect_fill": round(estimate.rect_fill, 3),
                "aspect_ratio": round(estimate.aspect_ratio, 3),
                "color_ratio": round(color_ratio, 3),
            },
            "ok",
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
