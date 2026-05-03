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
