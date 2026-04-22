from __future__ import annotations

import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from omx_perception.camera_geometry import CameraIntrinsics, TransformSnapshot
from omx_perception.perception_pipeline import (
    WorkspaceRect,
    build_undistort_maps,
    contour_center_in_workspace,
    estimate_block_pose,
    extract_rect_patch_ab,
    gray_world_white_balance,
    segment_foreground,
    split_large_blob,
)


class ColorCalibrator(Node):
    """Capture a*,b* median samples from detected cubes to build color prototypes."""

    def __init__(self) -> None:
        super().__init__("color_calibrator")

        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("camera_frame", "default_cam")
        self.declare_parameter("block_size_m", 0.030)
        self.declare_parameter("table_z_m", 0.18)
        self.declare_parameter("workspace_x_min_m", 0.08)
        self.declare_parameter("workspace_x_max_m", 0.35)
        self.declare_parameter("workspace_y_min_m", -0.15)
        self.declare_parameter("workspace_y_max_m", 0.15)
        self.declare_parameter("canny_sigma", 0.33)
        self.declare_parameter("min_contour_area_px", 200.0)
        self.declare_parameter("max_contour_area_px", 20000.0)
        self.declare_parameter("saturation_min", 40)
        self.declare_parameter("rect_fill_min", 0.75)
        self.declare_parameter("aspect_ratio_min", 0.60)
        self.declare_parameter("aspect_ratio_max", 1.66)
        self.declare_parameter("use_white_balance", True)
        self.declare_parameter("chroma_min", 10.0)
        self.declare_parameter("samples_per_color", 20)
        self.declare_parameter("output_path", "color_prototypes.yaml")

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._intrinsics: CameraIntrinsics | None = None
        self._undistort_map_1: np.ndarray | None = None
        self._undistort_map_2: np.ndarray | None = None
        self._image_size: tuple[int, int] | None = None

        self._lock = threading.Lock()
        self._current_color: str | None = None
        self._samples: dict[str, list[tuple[float, float]]] = {}
        self._diag: dict[str, int] = {}
        self._frames_seen = 0

        self.create_subscription(CameraInfo, "/camera_info", self._on_camera_info, 10)
        self.create_subscription(Image, "/image/raw", self._on_image, 10)

    def start_sampling(self, color: str) -> None:
        with self._lock:
            self._current_color = color
            self._samples.setdefault(color, [])

    def stop_sampling(self) -> None:
        with self._lock:
            self._current_color = None

    def sample_count(self, color: str) -> int:
        with self._lock:
            return len(self._samples.get(color, []))

    def drain_diagnostics(self) -> tuple[int, dict[str, int]]:
        with self._lock:
            snapshot = dict(self._diag)
            frames = self._frames_seen
            self._diag.clear()
            self._frames_seen = 0
            return frames, snapshot

    def save_prototypes(self, path: Path) -> dict:
        with self._lock:
            snapshot = {k: list(v) for k, v in self._samples.items() if v}
        data: dict = {}
        for color, samples in snapshot.items():
            arr = np.asarray(samples, dtype=np.float64)
            data[color] = {
                "a_star": float(np.median(arr[:, 0])),
                "b_star": float(np.median(arr[:, 1])),
                "samples": len(samples),
            }
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(data, sort_keys=True), encoding="utf-8")
        return data

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

    def _ensure_maps(self, width: int, height: int) -> None:
        if self._intrinsics is None:
            return
        if self._image_size == (width, height) and self._undistort_map_1 is not None:
            return
        m1, m2 = build_undistort_maps(self._intrinsics, (width, height))
        self._undistort_map_1, self._undistort_map_2 = m1, m2
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

    def _bump(self, key: str) -> None:
        with self._lock:
            self._diag[key] = self._diag.get(key, 0) + 1

    def _on_image(self, msg: Image) -> None:
        with self._lock:
            target_color = self._current_color
            target_count = int(self.get_parameter("samples_per_color").value)
            current_count = len(self._samples.get(target_color, [])) if target_color else 0
            self._frames_seen += 1
        if target_color is None or current_count >= target_count:
            return
        if self._intrinsics is None:
            self._bump("no_intrinsics")
            return
        transform = self._lookup_transform(
            str(self.get_parameter("reference_frame").value),
            str(self.get_parameter("camera_frame").value),
        )
        if transform is None:
            self._bump("no_tf")
            return

        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = bgr.shape[:2]
        self._ensure_maps(w, h)
        if self._undistort_map_1 is not None:
            bgr = cv2.remap(
                bgr, self._undistort_map_1, self._undistort_map_2, cv2.INTER_LINEAR
            )
        if bool(self.get_parameter("use_white_balance").value):
            bgr = gray_world_white_balance(bgr)

        intrinsics = self._pipeline_intrinsics()
        canny_sigma = float(self.get_parameter("canny_sigma").value)
        sat_min = int(self.get_parameter("saturation_min").value)
        contours, _, _ = segment_foreground(bgr, canny_sigma, saturation_min=sat_min)

        block_size = float(self.get_parameter("block_size_m").value)
        table_z = float(self.get_parameter("table_z_m").value)
        workspace = WorkspaceRect(
            x_min_m=float(self.get_parameter("workspace_x_min_m").value),
            x_max_m=float(self.get_parameter("workspace_x_max_m").value),
            y_min_m=float(self.get_parameter("workspace_y_min_m").value),
            y_max_m=float(self.get_parameter("workspace_y_max_m").value),
            plane_z_m=table_z + block_size,
        )
        min_area = float(self.get_parameter("min_contour_area_px").value)
        max_area = float(self.get_parameter("max_contour_area_px").value)
        rect_fill_min = float(self.get_parameter("rect_fill_min").value)
        aspect_min = float(self.get_parameter("aspect_ratio_min").value)
        aspect_max = float(self.get_parameter("aspect_ratio_max").value)
        chroma_min = float(self.get_parameter("chroma_min").value)

        self._bump("frames_processed")
        if not contours:
            self._bump("no_contours")
        for raw in contours:
            self._bump("contours")
            area = float(cv2.contourArea(raw))
            if area < min_area:
                self._bump("area_too_small")
                continue
            if area > max_area * 3.0:
                self._bump("area_too_big")
                continue
            for contour in split_large_blob(raw, max_area, min_area):
                in_workspace, _ = contour_center_in_workspace(
                    contour, intrinsics, transform, workspace
                )
                if not in_workspace:
                    self._bump("outside_ws")
                    continue
                estimate, reason = estimate_block_pose(
                    contour, intrinsics, transform,
                    block_size_m=block_size,
                    table_z_m=table_z,
                    rect_fill_min=rect_fill_min,
                    aspect_ratio_min=aspect_min,
                    aspect_ratio_max=aspect_max,
                )
                if estimate is None:
                    self._bump(reason)
                    continue
                patch = extract_rect_patch_ab(
                    bgr, estimate.rect_corners_px, chroma_min=chroma_min
                )
                if patch is None:
                    self._bump("patch_achromatic")
                    continue
                a_med, b_med, _ = patch
                with self._lock:
                    if self._current_color != target_color:
                        return
                    self._samples.setdefault(target_color, []).append((a_med, b_med))
                    self._diag["sampled"] = self._diag.get("sampled", 0) + 1
                    if len(self._samples[target_color]) >= target_count:
                        self._current_color = None
                        return


def _prompt(message: str) -> str:
    return input(message).strip().lower()


def _interactive_loop(node: ColorCalibrator) -> None:
    print("Color prototype calibrator. Place one block per color in the workspace.")
    print("Commands: <color> | save | quit")
    while True:
        command = _prompt("> ")
        if not command:
            continue
        if command in ("quit", "exit"):
            return
        if command == "save":
            path = Path(str(node.get_parameter("output_path").value))
            data = node.save_prototypes(path)
            print(f"Saved {len(data)} prototypes to {path}: {data}")
            continue
        if command not in ("red", "green", "blue"):
            print("Unknown color. Use red/green/blue/save/quit.")
            continue

        target = int(node.get_parameter("samples_per_color").value)
        node.start_sampling(command)
        node.drain_diagnostics()
        print(f"Sampling '{command}': {target} samples needed...")
        deadline = time.monotonic() + 60.0
        last_count = -1
        last_diag_print = time.monotonic()
        while time.monotonic() < deadline:
            count = node.sample_count(command)
            if count != last_count:
                print(f"  {command}: {count}/{target}")
                last_count = count
            if count >= target:
                break
            now = time.monotonic()
            if now - last_diag_print >= 2.0:
                frames, diag = node.drain_diagnostics()
                if frames == 0:
                    print("  [diag] no /image/raw received — is usb_cam running?")
                else:
                    summary = ", ".join(f"{k}={v}" for k, v in sorted(diag.items()))
                    print(f"  [diag] frames={frames} {summary or '(no contours processed)'}")
                last_diag_print = now
            time.sleep(0.2)
        node.stop_sampling()
        final = node.sample_count(command)
        if final < target:
            print(f"Timed out at {final}/{target} samples for '{command}'.")
            frames, diag = node.drain_diagnostics()
            if diag:
                summary = ", ".join(f"{k}={v}" for k, v in sorted(diag.items()))
                print(f"Final diagnostics — frames={frames} {summary}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ColorCalibrator()
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()
    try:
        _interactive_loop(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
