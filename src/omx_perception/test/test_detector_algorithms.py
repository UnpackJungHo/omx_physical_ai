import unittest
import types
import sys

import cv2
import numpy as np

if "rclpy" not in sys.modules:
    rclpy_stub = types.ModuleType("rclpy")
    rclpy_stub.time = types.SimpleNamespace(Time=lambda: None)
    sys.modules["rclpy"] = rclpy_stub

if "rclpy.node" not in sys.modules:
    rclpy_node_stub = types.ModuleType("rclpy.node")

    class DummyNode:
        pass

    rclpy_node_stub.Node = DummyNode
    sys.modules["rclpy.node"] = rclpy_node_stub

if "cv_bridge" not in sys.modules:
    cv_bridge_stub = types.ModuleType("cv_bridge")

    class DummyCvBridge:
        pass

    cv_bridge_stub.CvBridge = DummyCvBridge
    sys.modules["cv_bridge"] = cv_bridge_stub

if "sensor_msgs" not in sys.modules:
    sensor_msgs_stub = types.ModuleType("sensor_msgs")
    sensor_msgs_msg_stub = types.ModuleType("sensor_msgs.msg")

    class DummyCameraInfo:
        pass

    class DummyImage:
        pass

    sensor_msgs_msg_stub.CameraInfo = DummyCameraInfo
    sensor_msgs_msg_stub.Image = DummyImage
    sys.modules["sensor_msgs"] = sensor_msgs_stub
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg_stub

if "std_msgs" not in sys.modules:
    std_msgs_stub = types.ModuleType("std_msgs")
    std_msgs_msg_stub = types.ModuleType("std_msgs.msg")

    class DummyString:
        pass

    std_msgs_msg_stub.String = DummyString
    sys.modules["std_msgs"] = std_msgs_stub
    sys.modules["std_msgs.msg"] = std_msgs_msg_stub

if "tf2_ros" not in sys.modules:
    tf2_ros_stub = types.ModuleType("tf2_ros")

    class DummyBuffer:
        pass

    class DummyTransformListener:
        def __init__(self, *args, **kwargs) -> None:
            pass

    tf2_ros_stub.Buffer = DummyBuffer
    tf2_ros_stub.TransformListener = DummyTransformListener
    sys.modules["tf2_ros"] = tf2_ros_stub

from omx_perception.detector_node import (
    _classify_core_color,
    _compute_side_support_metrics,
    _largest_contour,
    _split_component_contour,
)


class DetectorAlgorithmTest(unittest.TestCase):
    def test_split_component_contour_separates_connected_objects(self) -> None:
        combined = np.zeros((120, 220), dtype=np.uint8)
        strict = np.zeros_like(combined)

        # Left flat object and right cube candidate connected by a narrow bridge.
        cv2.rectangle(combined, (20, 70), (120, 105), 255, thickness=-1)
        cv2.rectangle(combined, (120, 83), (132, 92), 255, thickness=-1)
        cv2.rectangle(combined, (132, 48), (182, 108), 255, thickness=-1)

        # Two distinct color seeds inside the merged component.
        cv2.rectangle(strict, (28, 76), (72, 99), 255, thickness=-1)
        cv2.rectangle(strict, (142, 56), (170, 84), 255, thickness=-1)

        contour = _largest_contour(combined)
        self.assertIsNotNone(contour)

        split = _split_component_contour(combined, strict, contour, min_area=500)
        self.assertGreaterEqual(len(split), 2)

        areas = sorted(int(cv2.contourArea(cnt)) for cnt in split)
        self.assertGreater(areas[0], 700)
        self.assertGreater(areas[1], 1200)

    def test_classify_core_color_prefers_blue_with_margin(self) -> None:
        core = np.zeros((40, 40), dtype=np.uint8)
        core[10:30, 10:30] = 255

        strict_masks = {
            "red": np.zeros_like(core),
            "green": np.zeros_like(core),
            "blue": core.copy(),
        }
        dominance_masks = {
            "red": np.zeros_like(core),
            "green": np.zeros_like(core),
            "blue": core.copy(),
        }

        color, purity, margin, scores = _classify_core_color(core, strict_masks, dominance_masks)
        self.assertEqual(color, "blue")
        self.assertGreater(purity, 0.95)
        self.assertGreater(margin, 0.9)
        self.assertGreater(scores["blue"], scores["red"])
        self.assertGreater(scores["blue"], scores["green"])

    def test_side_support_metrics_distinguish_cube_from_flat_patch(self) -> None:
        contour_mask = np.zeros((80, 80), dtype=np.uint8)
        cv2.rectangle(contour_mask, (20, 20), (58, 62), 255, thickness=-1)

        core_mask = np.zeros_like(contour_mask)
        cv2.rectangle(core_mask, (28, 24), (50, 42), 255, thickness=-1)

        cube_hsv = np.zeros((80, 80, 3), dtype=np.uint8)
        cube_hsv[:, :, 2] = 60
        cube_hsv[core_mask > 0, 2] = 190
        cube_hsv[(contour_mask > 0) & (core_mask == 0), 2] = 105

        flat_hsv = np.zeros((80, 80, 3), dtype=np.uint8)
        flat_hsv[:, :, 2] = 60
        flat_hsv[core_mask > 0, 2] = 190
        flat_hsv[(contour_mask > 0) & (core_mask == 0), 2] = 182

        cube_metrics = _compute_side_support_metrics(contour_mask, core_mask, cube_hsv)
        flat_metrics = _compute_side_support_metrics(contour_mask, core_mask, flat_hsv)

        self.assertGreater(cube_metrics["side_area_ratio"], 0.2)
        self.assertGreater(cube_metrics["side_value_drop"], 40.0)
        self.assertLess(flat_metrics["side_value_drop"], 12.0)


if __name__ == "__main__":
    unittest.main()
