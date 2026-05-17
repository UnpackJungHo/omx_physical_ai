#!/usr/bin/env python3
"""Calibration CLI: collect LAB reference values from live camera."""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path
from threading import Event, Lock

import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from omx_interfaces.msg import KeypointDetection
from omx_interfaces.srv import GetKeypointDetections
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from omx_perception.color_classifier import ClassifierParams, extract_valid_lab_pixels


class CalibrationNode(Node):
    def __init__(
        self,
        colors: list[str],
        frames_per_color: int,
        output: Path,
    ) -> None:
        super().__init__("box_color_calibrate")
        self._colors = colors
        self._frames_per_color = frames_per_color
        self._output = output

        self._bridge = CvBridge()
        self._latest_image: np.ndarray | None = None
        self._image_lock = Lock()

        self._image_sub = self.create_subscription(
            Image,
            "/image/raw",
            self._on_image,
            10,
        )
        self._keypoints_client = self.create_client(
            GetKeypointDetections,
            "/perception/get_box_cup_keypoints",
        )

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError:
            return

        with self._image_lock:
            self._latest_image = frame

    def run(self, executor: SingleThreadedExecutor) -> int:
        self.get_logger().info("waiting for keypoint service...")
        if not self._keypoints_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                "keypoint service unavailable after 10s. Is perception running?"
            )
            return 1

        params = ClassifierParams()
        existing_data: dict = {}
        if self._output.exists():
            try:
                with self._output.open("r", encoding="utf-8") as stream:
                    existing_data = yaml.safe_load(stream) or {}
                params = ClassifierParams(
                    inset_ratio=float(existing_data.get("inset_ratio", params.inset_ratio)),
                    saturation_min=int(existing_data.get("saturation_min", params.saturation_min)),
                    luminance_low_percentile=float(
                        existing_data.get(
                            "luminance_low_percentile",
                            params.luminance_low_percentile,
                        )
                    ),
                    luminance_high_percentile=float(
                        existing_data.get(
                            "luminance_high_percentile",
                            params.luminance_high_percentile,
                        )
                    ),
                    min_valid_pixels=int(
                        existing_data.get("min_valid_pixels", params.min_valid_pixels)
                    ),
                    distance_threshold=float(
                        existing_data.get("distance_threshold", params.distance_threshold)
                    ),
                )
            except Exception as exc:
                self.get_logger().warning(f"could not load existing yaml: {exc}")

        new_refs: list[dict] = []
        for color_name in self._colors:
            input(f"\nPlace {color_name.upper()} box in view. Press [Enter] when ready...")
            print(f"Collecting {self._frames_per_color} frames for {color_name}...")
            pixels = self._collect_pixels(color_name, executor, params)
            if pixels is None:
                return 1

            median_a = float(np.median(pixels[:, 1]))
            median_b = float(np.median(pixels[:, 2]))
            new_refs.append(
                {"name": color_name, "lab_ab": [round(median_a, 1), round(median_b, 1)]}
            )
            print(f"  -> {color_name}: a*={median_a:.1f}, b*={median_b:.1f}")

        self._save_yaml(params, new_refs)
        return 0

    def _collect_pixels(
        self,
        color_name: str,
        executor: SingleThreadedExecutor,
        params: ClassifierParams,
    ) -> np.ndarray | None:
        all_pixels: list[np.ndarray] = []
        collected = 0
        max_attempts = self._frames_per_color * 8

        for _attempt in range(max_attempts):
            if collected >= self._frames_per_color:
                break

            request = GetKeypointDetections.Request()
            request.publish_debug = False
            future = self._keypoints_client.call_async(request)
            done = Event()
            future.add_done_callback(lambda _: done.set())

            deadline = self.get_clock().now().nanoseconds + int(2e9)
            while not done.is_set():
                executor.spin_once(timeout_sec=0.05)
                if self.get_clock().now().nanoseconds > deadline:
                    self.get_logger().warning("service call timed out, skipping frame")
                    break

            if not done.is_set():
                continue

            try:
                response = future.result()
            except Exception as exc:
                self.get_logger().warning(f"service call failed: {exc}")
                continue

            if not response.success:
                continue

            box_dets = [
                det
                for det in response.detections
                if det.class_id == KeypointDetection.CLASS_BOX
            ]
            if len(box_dets) != 1:
                self.get_logger().warning(
                    f"expected 1 box, got {len(box_dets)}; skipping frame"
                )
                continue

            with self._image_lock:
                frame = self._latest_image.copy() if self._latest_image is not None else None

            if frame is None:
                self.get_logger().warning("no image received yet, skipping frame")
                continue

            det = box_dets[0]
            polygon_pts = np.array(
                [[det.keypoints[i * 3], det.keypoints[i * 3 + 1]] for i in range(4)],
                dtype=float,
            )
            pixels = extract_valid_lab_pixels(frame, polygon_pts, params)
            if len(pixels) >= params.min_valid_pixels:
                all_pixels.append(pixels)
                collected += 1
                print(
                    f"  [{collected}/{self._frames_per_color}] "
                    f"frame ok ({len(pixels)} px)",
                    end="\r",
                )

        print()

        if collected < self._frames_per_color:
            self.get_logger().error(
                f"only {collected}/{self._frames_per_color} valid frames for {color_name} "
                f"after {max_attempts} attempts. Aborting; yaml not modified."
            )
            return None

        return np.vstack(all_pixels)

    def _save_yaml(
        self,
        params: ClassifierParams,
        new_refs: list[dict],
    ) -> None:
        if self._output.exists():
            backup = self._output.with_suffix(".yaml.bak")
            shutil.copy2(self._output, backup)
            print(f"\nBacked up existing yaml -> {backup}")

        data = {
            "inset_ratio": params.inset_ratio,
            "saturation_min": params.saturation_min,
            "luminance_low_percentile": params.luminance_low_percentile,
            "luminance_high_percentile": params.luminance_high_percentile,
            "min_valid_pixels": params.min_valid_pixels,
            "distance_threshold": params.distance_threshold,
            "references": new_refs,
        }

        self._output.parent.mkdir(parents=True, exist_ok=True)
        with self._output.open("w", encoding="utf-8") as stream:
            yaml.dump(data, stream, default_flow_style=False, sort_keys=False)
        print(f"Saved reference yaml -> {self._output}")


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="Calibrate box color references")
    parser.add_argument("--colors", nargs="+", default=["red", "green", "blue"])
    parser.add_argument("--frames-per-color", type=int, default=30)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("src/omx_perception/config/box_color_reference.yaml"),
    )
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = CalibrationNode(
        colors=parsed.colors,
        frames_per_color=parsed.frames_per_color,
        output=parsed.output.resolve(),
    )
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        exit_code = node.run(executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\nAborted; yaml not modified.")
        exit_code = 1
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)
