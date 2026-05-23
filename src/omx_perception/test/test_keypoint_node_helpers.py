"""Unit tests for _extract_detections helper in box_cup_keypoint_node.

rclpy is NOT initialized — only pure data-structure logic is tested.
"""
from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest


def _make_node():
    """Return a BoxCupKeypointNode instance without ROS init by mocking rclpy internals."""
    import sys
    # Patch rclpy.node.Node so we can instantiate without a live ROS context
    node_mock = MagicMock()
    node_mock.get_parameter = MagicMock(
        side_effect=lambda name: _fake_param(name)
    )

    from omx_perception import box_cup_keypoint_node as mod
    original_node_cls = mod.Node

    # Temporarily replace Node base class
    mod.Node = MagicMock(return_value=node_mock)
    try:
        # We only need the helper method, so extract it unbound
        extract = mod.BoxCupKeypointNode._extract_detections
    finally:
        mod.Node = original_node_cls

    return extract


def _fake_param(name: str):
    defaults = {
        "model_path": "",
        "image_topic": "/image/raw",
        "output_dir": "/tmp",
        "extra_pythonpath": "",
        "device": "cpu",
        "imgsz": 640,
        "conf": 0.25,
        "max_det": 20,
        "publish_debug": False,
    }
    p = MagicMock()
    p.value = defaults.get(name, "")
    return p


def _make_result(n_detections: int, n_keypoints: int = 4, has_conf: bool = True):
    """Build a fake YOLO result object."""
    result = MagicMock()
    if n_detections == 0:
        result.boxes = None
        return result

    conf_vals = np.array([0.9 - 0.1 * i for i in range(n_detections)], dtype=np.float32)
    cls_vals = np.array([i % 2 for i in range(n_detections)], dtype=np.float32)

    boxes = MagicMock()
    # MagicMock.__len__ defaults to 0 — must be set explicitly
    boxes.__len__ = MagicMock(return_value=n_detections)
    boxes.conf.detach().cpu().numpy.return_value = conf_vals
    boxes.cls.detach().cpu().numpy().astype.return_value = cls_vals.astype(int)
    result.boxes = boxes

    keypoints = MagicMock()
    # shape: (n_detections, n_keypoints, 2)
    xy = np.zeros((n_detections, n_keypoints, 2), dtype=np.float32)
    for i in range(n_detections):
        for k in range(n_keypoints):
            xy[i, k] = [float(i * 100 + k * 10), float(i * 50 + k * 5)]

    keypoints.xy.detach().cpu().numpy.return_value = xy

    if has_conf:
        kp_conf = np.full((n_detections, n_keypoints), 0.8, dtype=np.float32)
        keypoints.conf = MagicMock()
        keypoints.conf.detach().cpu().numpy.return_value = kp_conf
    else:
        keypoints.conf = None

    result.keypoints = keypoints
    return result


class FakeSelf:
    """Minimal self for calling _extract_detections as an unbound function."""


def _call_extract(result):
    from omx_perception.box_cup_keypoint_node import BoxCupKeypointNode
    fake_self = FakeSelf()
    return BoxCupKeypointNode._extract_detections(fake_self, result)


# -----------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------

def test_empty_boxes_returns_empty():
    result = _make_result(0)
    dets = _call_extract(result)
    assert dets == []


def test_single_detection_structure():
    result = _make_result(1)
    dets = _call_extract(result)
    assert len(dets) == 1
    det = dets[0]
    assert "class_id" in det
    assert "class_name" in det
    assert "confidence" in det
    assert "keypoints" in det


def test_keypoints_length_is_four():
    result = _make_result(1)
    dets = _call_extract(result)
    assert len(dets[0]["keypoints"]) == 4


def test_keypoint_tuple_has_three_elements():
    result = _make_result(1)
    dets = _call_extract(result)
    for kp in dets[0]["keypoints"]:
        assert len(kp) == 3, "keypoint must be (x, y, conf)"


def test_confidence_without_keypoint_conf_defaults_to_1():
    result = _make_result(1, has_conf=False)
    dets = _call_extract(result)
    for kp in dets[0]["keypoints"]:
        assert kp[2] == pytest.approx(1.0)


def test_multiple_detections_sorted_by_confidence_desc():
    result = _make_result(3)
    dets = _call_extract(result)
    confs = [d["confidence"] for d in dets]
    assert confs == sorted(confs, reverse=True)


def test_class_name_matches_class_id():
    result = _make_result(2)
    dets = _call_extract(result)
    for det in dets:
        if det["class_id"] == 0:
            assert det["class_name"] == "Box"
        elif det["class_id"] == 1:
            assert det["class_name"] == "Cup"


def test_skips_detection_with_fewer_than_4_keypoints():
    result = _make_result(1, n_keypoints=3)
    dets = _call_extract(result)
    assert dets == []
