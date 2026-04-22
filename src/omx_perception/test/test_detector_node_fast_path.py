import sys
import types
import unittest

import cv2
import numpy as np


def _ensure_module(name: str) -> types.ModuleType:
    module = sys.modules.get(name)
    if module is None:
        module = types.ModuleType(name)
        sys.modules[name] = module
    return module


if "rclpy" not in sys.modules:
    sys.modules["rclpy"] = types.ModuleType("rclpy")

if "rclpy.node" not in sys.modules:
    rclpy_node_stub = types.ModuleType("rclpy.node")

    class DummyNode:
        pass

    rclpy_node_stub.Node = DummyNode
    sys.modules["rclpy.node"] = rclpy_node_stub

if "rclpy.qos" not in sys.modules:
    rclpy_qos_stub = types.ModuleType("rclpy.qos")

    class DummyQoSProfile:
        def __init__(self, *args, **kwargs) -> None:
            self.args = args
            self.kwargs = kwargs

    class DummyHistoryPolicy:
        KEEP_LAST = 1

    class DummyReliabilityPolicy:
        BEST_EFFORT = 1

    rclpy_qos_stub.QoSProfile = DummyQoSProfile
    rclpy_qos_stub.HistoryPolicy = DummyHistoryPolicy
    rclpy_qos_stub.ReliabilityPolicy = DummyReliabilityPolicy
    sys.modules["rclpy.qos"] = rclpy_qos_stub

if "cv_bridge" not in sys.modules:
    cv_bridge_stub = types.ModuleType("cv_bridge")

    class DummyCvBridge:
        pass

    cv_bridge_stub.CvBridge = DummyCvBridge
    sys.modules["cv_bridge"] = cv_bridge_stub

sensor_msg_stub = _ensure_module("sensor_msgs.msg")


class DummyCameraInfo:
    pass


class DummyImage:
    pass


sensor_msg_stub.CameraInfo = DummyCameraInfo
sensor_msg_stub.Image = DummyImage
_ensure_module("sensor_msgs")

std_msg_stub = _ensure_module("std_msgs.msg")


class DummyString:
    def __init__(self, data: str = "") -> None:
        self.data = data


std_msg_stub.String = DummyString
_ensure_module("std_msgs")

tf2_stub = _ensure_module("tf2_ros")


class DummyBuffer:
    pass


class DummyTransformListener:
    def __init__(self, *args, **kwargs) -> None:
        pass


tf2_stub.Buffer = DummyBuffer
tf2_stub.TransformListener = DummyTransformListener


from omx_perception.camera_geometry import CameraIntrinsics, TransformSnapshot
from omx_perception.detector_node import DetectorNode
from omx_perception.perception_pipeline import ColorPrototype, DetectorSettings, WorkspaceRect


def _top_down_transform() -> TransformSnapshot:
    return TransformSnapshot(
        reference_frame="world",
        translation_m=np.array([0.20, 0.0, 0.30], dtype=np.float64),
        rotation_xyzw=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
    )


def _make_color_prototypes() -> list[ColorPrototype]:
    def proto(name: str, bgr: tuple[int, int, int]) -> ColorPrototype:
        lab = cv2.cvtColor(np.array([[list(bgr)]], dtype=np.uint8), cv2.COLOR_BGR2LAB)
        return ColorPrototype(
            name=name,
            a_star=float(lab[0, 0, 1]) - 128.0,
            b_star=float(lab[0, 0, 2]) - 128.0,
        )

    return [
        proto("red", (0, 0, 255)),
        proto("green", (0, 255, 0)),
        proto("blue", (255, 0, 0)),
    ]


class DetectorNodeFastPathTest(unittest.TestCase):
    def test_on_image_keeps_only_latest_frame(self) -> None:
        node = DetectorNode.__new__(DetectorNode)
        node._latest_image_msg = None
        node._received_frames = 0
        node._overwritten_frames = 0

        first = DummyImage()
        second = DummyImage()

        DetectorNode._on_image(node, first)
        DetectorNode._on_image(node, second)

        self.assertIs(node._latest_image_msg, second)
        self.assertEqual(node._received_frames, 2)
        self.assertEqual(node._overwritten_frames, 1)

    def test_process_contour_returns_fast_rect_pose_detection(self) -> None:
        node = DetectorNode.__new__(DetectorNode)
        node._prototypes = _make_color_prototypes()

        image = np.zeros((120, 120, 3), dtype=np.uint8)
        contour = np.array(
            [[40, 40], [80, 40], [80, 80], [40, 80]],
            dtype=np.int32,
        ).reshape(-1, 1, 2)
        cv2.fillConvexPoly(image, contour.reshape(-1, 2), (0, 0, 255))
        debug = image.copy()

        intrinsics = CameraIntrinsics(fx=600.0, fy=600.0, cx=60.0, cy=60.0)
        transform = _top_down_transform()
        workspace = WorkspaceRect(
            x_min_m=0.0,
            x_max_m=0.4,
            y_min_m=-0.2,
            y_max_m=0.2,
            plane_z_m=0.21,
        )
        settings = DetectorSettings(
            block_size_m=0.030,
            color_chroma_min=10.0,
            color_majority_min=0.40,
            rect_fill_min=0.75,
            aspect_ratio_min=0.60,
            aspect_ratio_max=1.66,
        )

        detection, reason = DetectorNode._process_contour(
            node,
            contour,
            image,
            debug,
            intrinsics,
            transform,
            workspace,
            0.18,
            settings,
        )

        self.assertEqual(reason, "ok")
        self.assertIsNotNone(detection)
        assert detection is not None
        self.assertEqual(detection["source"], "rect_pose")
        self.assertEqual(detection["color"], "red")
        self.assertGreater(detection["confidence"], 0.90)


if __name__ == "__main__":
    unittest.main()
