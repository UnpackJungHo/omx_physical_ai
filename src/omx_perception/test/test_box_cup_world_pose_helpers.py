"""Pure-function unit tests for box_cup_world_pose_node helpers.

These tests do not initialise rclpy; they only import the module-level
helpers so they can run as a stand-alone pytest invocation.
"""
from __future__ import annotations

import math

import numpy as np
import pytest
from omx_interfaces.msg import KeypointDetection

from omx_perception.box_cup_world_pose_node import (
    box_yaw_world,
    object_points_for_class,
    quaternion_from_yaw,
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


def test_box_yaw_world_identity_returns_zero():
    rvec = np.zeros((3, 1), dtype=np.float64)
    rotation_world_cam = np.eye(3, dtype=np.float64)
    assert math.isclose(box_yaw_world(rvec, rotation_world_cam), 0.0, abs_tol=1e-9)


def test_box_yaw_world_object_rotated_30deg_about_z():
    # object 가 카메라 z축 기준 30° 회전; 카메라=월드 일치.
    rvec = np.asarray([0.0, 0.0, math.radians(30.0)], dtype=np.float64).reshape(3, 1)
    rotation_world_cam = np.eye(3, dtype=np.float64)
    assert math.isclose(
        box_yaw_world(rvec, rotation_world_cam), math.radians(30.0), abs_tol=1e-6
    )


def test_box_yaw_world_applies_world_cam_rotation():
    # object 는 회전 없음, 카메라→월드 변환이 z축 90°.
    rvec = np.zeros((3, 1), dtype=np.float64)
    rotation_world_cam = quaternion_to_rotation_matrix(0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert math.isclose(
        box_yaw_world(rvec, rotation_world_cam), math.radians(90.0), abs_tol=1e-6
    )


def test_box_yaw_world_composes_object_and_world_cam_rotations():
    # object 가 카메라 z축 기준 30° 회전 + camera→world 변환이 z축 90°
    # → world yaw = 30° + 90° = 120°.
    rvec = np.asarray([0.0, 0.0, math.radians(30.0)], dtype=np.float64).reshape(3, 1)
    rotation_world_cam = quaternion_to_rotation_matrix(
        0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)
    )
    assert math.isclose(
        box_yaw_world(rvec, rotation_world_cam), math.radians(120.0), abs_tol=1e-6
    )


def test_quaternion_from_yaw_roundtrip():
    for deg in (-90.0, -30.0, 0.0, 45.0, 120.0):
        qx, qy, qz, qw = quaternion_from_yaw(math.radians(deg))
        matrix = quaternion_to_rotation_matrix(qx, qy, qz, qw)
        x_axis = matrix @ np.asarray([1.0, 0.0, 0.0])
        assert math.isclose(
            math.atan2(x_axis[1], x_axis[0]), math.radians(deg), abs_tol=1e-6
        )
