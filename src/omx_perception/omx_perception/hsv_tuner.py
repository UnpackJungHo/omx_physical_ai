#!/usr/bin/env python3
"""Interactive HSV range calibration tool for block detection.

Static image mode:
    python3 hsv_tuner.py --color red --image /path/to/image.png

ROS live mode (subscribes to /image/raw):
    ros2 run omx_perception hsv_tuner --ros-args -p color:=red

Keys:
    s — save current range to color_ranges.yaml
    r — reset to defaults
    q — quit
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

_DEFAULT_RANGES: dict = {
    "red": [
        {"h": [0, 15], "s": [50, 255], "v": [40, 255]},
        {"h": [160, 180], "s": [50, 255], "v": [40, 255]},
    ],
    "green": [{"h": [40, 95], "s": [40, 255], "v": [40, 255]}],
    "blue": [{"h": [95, 135], "s": [60, 255], "v": [40, 255]}],
}

_RANGES_FILE = Path(__file__).parent.parent / "config" / "color_ranges.yaml"


def _get_ranges_file() -> Path:
    """Resolve to installed path at runtime; fall back to source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory("omx_perception")) / "config" / "color_ranges.yaml"
        if p.exists():
            return p
    except Exception:
        pass
    return _RANGES_FILE


def _apply_clahe(bgr: np.ndarray) -> np.ndarray:
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    return cv2.cvtColor(cv2.merge([clahe.apply(l), a, b]), cv2.COLOR_LAB2BGR)


def _build_mask(hsv: np.ndarray, h_lo: int, h_hi: int, s_lo: int, s_hi: int, v_lo: int, v_hi: int) -> np.ndarray:
    lo = np.array([h_lo, s_lo, v_lo], dtype=np.uint8)
    hi = np.array([h_hi, s_hi, v_hi], dtype=np.uint8)
    mask = cv2.inRange(hsv, lo, hi)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
    return mask


def _load_ranges() -> dict:
    p = _get_ranges_file()
    if p.exists():
        data = yaml.safe_load(p.read_text())
        if isinstance(data, dict):
            return data
    return _DEFAULT_RANGES


def _save_ranges(ranges: dict) -> None:
    p = _get_ranges_file()
    p.write_text(yaml.dump(ranges, default_flow_style=False, allow_unicode=True))
    print(f"[hsv_tuner] Saved → {p}")


def nothing(_: int) -> None:
    pass


def run_static(color: str, image_path: str) -> None:
    bgr_raw = cv2.imread(image_path)
    if bgr_raw is None:
        print(f"[hsv_tuner] Cannot read image: {image_path}")
        sys.exit(1)

    ranges = _load_ranges()
    # Use first range entry for initial trackbar values
    r0 = ranges.get(color, _DEFAULT_RANGES.get(color, [{}]))[0]
    is_red = color == "red"
    r1 = ranges.get(color, [{}])[1] if is_red and len(ranges.get(color, [])) > 1 else {}

    win = f"HSV Tuner - {color}"
    cv2.namedWindow(win, cv2.WINDOW_KEEPRATIO)
    cv2.waitKey(1)  # flush Qt event loop so window handle is valid before createTrackbar

    cv2.createTrackbar("H_lo", win, r0.get("h", [0, 0])[0], 179, nothing)
    cv2.createTrackbar("H_hi", win, r0.get("h", [0, 179])[1], 179, nothing)
    cv2.createTrackbar("S_lo", win, r0.get("s", [0, 0])[0], 255, nothing)
    cv2.createTrackbar("S_hi", win, r0.get("s", [0, 255])[1], 255, nothing)
    cv2.createTrackbar("V_lo", win, r0.get("v", [0, 0])[0], 255, nothing)
    cv2.createTrackbar("V_hi", win, r0.get("v", [0, 255])[1], 255, nothing)

    if is_red:
        cv2.createTrackbar("H2_lo", win, r1.get("h", [160, 160])[0], 179, nothing)
        cv2.createTrackbar("H2_hi", win, r1.get("h", [160, 180])[1], 179, nothing)

    print(f"[hsv_tuner] Tuning '{color}'. Keys: s=save, r=reset, q=quit")

    while True:
        h_lo = cv2.getTrackbarPos("H_lo", win)
        h_hi = cv2.getTrackbarPos("H_hi", win)
        s_lo = cv2.getTrackbarPos("S_lo", win)
        s_hi = cv2.getTrackbarPos("S_hi", win)
        v_lo = cv2.getTrackbarPos("V_lo", win)
        v_hi = cv2.getTrackbarPos("V_hi", win)

        bgr = _apply_clahe(bgr_raw)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = _build_mask(hsv, h_lo, h_hi, s_lo, s_hi, v_lo, v_hi)

        if is_red:
            h2_lo = cv2.getTrackbarPos("H2_lo", win)
            h2_hi = cv2.getTrackbarPos("H2_hi", win)
            mask2 = _build_mask(hsv, h2_lo, h2_hi, s_lo, s_hi, v_lo, v_hi)
            mask = cv2.bitwise_or(mask, mask2)

        result = cv2.bitwise_and(bgr, bgr, mask=mask)
        display = np.hstack([bgr, result])
        cv2.imshow(win, display)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("r"):
            defaults = _DEFAULT_RANGES.get(color, [{}])
            cv2.setTrackbarPos("H_lo", win, defaults[0].get("h", [0, 0])[0])
            cv2.setTrackbarPos("H_hi", win, defaults[0].get("h", [0, 179])[1])
            cv2.setTrackbarPos("S_lo", win, defaults[0].get("s", [0, 0])[0])
            cv2.setTrackbarPos("S_hi", win, defaults[0].get("s", [0, 255])[1])
            cv2.setTrackbarPos("V_lo", win, defaults[0].get("v", [0, 0])[0])
            cv2.setTrackbarPos("V_hi", win, defaults[0].get("v", [0, 255])[1])
        elif key == ord("s"):
            all_ranges = _load_ranges()
            new_entry = [{"h": [h_lo, h_hi], "s": [s_lo, s_hi], "v": [v_lo, v_hi]}]
            if is_red:
                h2_lo = cv2.getTrackbarPos("H2_lo", win)
                h2_hi = cv2.getTrackbarPos("H2_hi", win)
                new_entry.append({"h": [h2_lo, h2_hi], "s": [s_lo, s_hi], "v": [v_lo, v_hi]})
            all_ranges[color] = new_entry
            _save_ranges(all_ranges)
            print(f"  range[0]: H=[{h_lo},{h_hi}] S=[{s_lo},{s_hi}] V=[{v_lo},{v_hi}]")

    cv2.destroyAllWindows()


def run_ros(color: str) -> None:
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    class TunerNode(Node):
        def __init__(self) -> None:
            super().__init__("hsv_tuner")
            self.declare_parameter("color", color)
            self._color = self.get_parameter("color").value
            self._bridge = CvBridge()
            self._latest: np.ndarray | None = None
            self.create_subscription(Image, "/image/raw", self._cb, 10)
            self.get_logger().info(f"Tuning color='{self._color}', waiting for /image/raw ...")

        def _cb(self, msg: Image) -> None:
            self._latest = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    rclpy.init()
    node = TunerNode()
    color_param = node.get_parameter("color").value

    ranges = _load_ranges()
    r0 = ranges.get(color_param, _DEFAULT_RANGES.get(color_param, [{}]))[0]
    is_red = color_param == "red"
    r1 = ranges.get(color_param, [{}])[1] if is_red and len(ranges.get(color_param, [])) > 1 else {}

    win = f"HSV Tuner - {color_param}"
    cv2.namedWindow(win, cv2.WINDOW_KEEPRATIO)
    cv2.waitKey(1)

    cv2.createTrackbar("H_lo", win, r0.get("h", [0, 0])[0], 179, nothing)
    cv2.createTrackbar("H_hi", win, r0.get("h", [0, 179])[1], 179, nothing)
    cv2.createTrackbar("S_lo", win, r0.get("s", [0, 0])[0], 255, nothing)
    cv2.createTrackbar("S_hi", win, r0.get("s", [0, 255])[1], 255, nothing)
    cv2.createTrackbar("V_lo", win, r0.get("v", [0, 0])[0], 255, nothing)
    cv2.createTrackbar("V_hi", win, r0.get("v", [0, 255])[1], 255, nothing)
    if is_red:
        cv2.createTrackbar("H2_lo", win, r1.get("h", [160, 160])[0], 179, nothing)
        cv2.createTrackbar("H2_hi", win, r1.get("h", [160, 180])[1], 179, nothing)

    print(f"[hsv_tuner] Tuning '{color_param}'. Keys: s=save, r=reset, q=quit")

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)

        if node._latest is None:
            if cv2.waitKey(30) & 0xFF == ord("q"):
                break
            continue

        h_lo = cv2.getTrackbarPos("H_lo", win)
        h_hi = cv2.getTrackbarPos("H_hi", win)
        s_lo = cv2.getTrackbarPos("S_lo", win)
        s_hi = cv2.getTrackbarPos("S_hi", win)
        v_lo = cv2.getTrackbarPos("V_lo", win)
        v_hi = cv2.getTrackbarPos("V_hi", win)

        bgr = _apply_clahe(node._latest)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = _build_mask(hsv, h_lo, h_hi, s_lo, s_hi, v_lo, v_hi)

        if is_red:
            h2_lo = cv2.getTrackbarPos("H2_lo", win)
            h2_hi = cv2.getTrackbarPos("H2_hi", win)
            mask = cv2.bitwise_or(mask, _build_mask(hsv, h2_lo, h2_hi, s_lo, s_hi, v_lo, v_hi))

        result = cv2.bitwise_and(bgr, bgr, mask=mask)
        cv2.imshow(win, np.hstack([bgr, result]))

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            all_ranges = _load_ranges()
            new_entry = [{"h": [h_lo, h_hi], "s": [s_lo, s_hi], "v": [v_lo, v_hi]}]
            if is_red:
                h2_lo = cv2.getTrackbarPos("H2_lo", win)
                h2_hi = cv2.getTrackbarPos("H2_hi", win)
                new_entry.append({"h": [h2_lo, h2_hi], "s": [s_lo, s_hi], "v": [v_lo, v_hi]})
            all_ranges[color_param] = new_entry
            _save_ranges(all_ranges)

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description="HSV block detector calibration tool")
    parser.add_argument("--color", choices=["red", "green", "blue"], default="red")
    parser.add_argument("--image", default="", help="Static image path (omit for ROS live mode)")
    args, _ = parser.parse_known_args()

    if args.image:
        run_static(args.color, args.image)
    else:
        run_ros(args.color)


if __name__ == "__main__":
    main()
