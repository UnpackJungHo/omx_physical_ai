from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    frame_id: str = "default_cam"
    # 5-element plumb_bob: (k1, k2, p1, p2, k3)
    dist_coeffs: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)

    def camera_matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    def dist_array(self) -> np.ndarray:
        return np.array(self.dist_coeffs, dtype=np.float64)

    @classmethod
    def from_camera_info_yaml(cls, path: str | Path) -> "CameraIntrinsics":
        with Path(path).open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)

        camera_matrix = data["camera_matrix"]["data"]
        dist = data.get("distortion_coefficients", {}).get("data", [0.0] * 5)
        return cls(
            fx=float(camera_matrix[0]),
            fy=float(camera_matrix[4]),
            cx=float(camera_matrix[2]),
            cy=float(camera_matrix[5]),
            frame_id=str(data.get("camera_name", "default_cam")),
            dist_coeffs=tuple(float(v) for v in dist),
        )


@dataclass(frozen=True)
class TransformSnapshot:
    reference_frame: str
    translation_m: np.ndarray
    rotation_xyzw: np.ndarray


@dataclass(frozen=True)
class Plane:
    frame_id: str
    normal_xyz: np.ndarray
    point_xyz: np.ndarray


def normalize(vector: Iterable[float]) -> np.ndarray:
    array = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(array)
    if norm == 0.0:
        raise ValueError("Zero-length vector is not valid.")
    return array / norm


def quaternion_to_rotation_matrix(quaternion_xyzw: Iterable[float]) -> np.ndarray:
    x, y, z, w = normalize(quaternion_xyzw)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def ray_direction_in_camera(
    intrinsics: CameraIntrinsics,
    pixel_u: float,
    pixel_v: float,
) -> np.ndarray:
    x = (pixel_u - intrinsics.cx) / intrinsics.fx
    y = (pixel_v - intrinsics.cy) / intrinsics.fy
    return normalize([x, y, 1.0])


def ray_in_reference_frame(
    intrinsics: CameraIntrinsics,
    transform: TransformSnapshot,
    pixel_u: float,
    pixel_v: float,
) -> tuple[np.ndarray, np.ndarray]:
    rotation = quaternion_to_rotation_matrix(transform.rotation_xyzw)
    direction_camera = ray_direction_in_camera(intrinsics, pixel_u, pixel_v)
    direction_reference = normalize(rotation @ direction_camera)
    origin_reference = transform.translation_m.astype(float)
    return origin_reference, direction_reference


def intersect_ray_with_plane(
    ray_origin: Iterable[float],
    ray_direction: Iterable[float],
    plane: Plane,
) -> np.ndarray | None:
    origin = np.asarray(ray_origin, dtype=float)
    direction = normalize(ray_direction)
    numerator = np.dot(plane.normal_xyz, plane.point_xyz - origin)
    denominator = np.dot(plane.normal_xyz, direction)
    if abs(denominator) < 1e-9:
        return None

    distance = numerator / denominator
    if distance < 0.0:
        return None

    return origin + distance * direction


def world_direction_to_image_vanishing_point(
    intrinsics: CameraIntrinsics,
    transform: TransformSnapshot,
    direction_world: Iterable[float],
) -> np.ndarray | None:
    """Project a world-frame direction (at infinity) onto the image plane.

    Returns pixel (u, v) where parallel world lines of this direction converge,
    or None if the direction is perpendicular to the optical axis (ideal point).
    """
    rotation_world_to_cam = quaternion_to_rotation_matrix(transform.rotation_xyzw).T
    direction_camera = rotation_world_to_cam @ np.asarray(direction_world, dtype=float)
    if abs(direction_camera[2]) < 1e-9:
        return None
    u = intrinsics.fx * direction_camera[0] / direction_camera[2] + intrinsics.cx
    v = intrinsics.fy * direction_camera[1] / direction_camera[2] + intrinsics.cy
    return np.array([u, v], dtype=float)


def project_world_point_to_image(
    intrinsics: CameraIntrinsics,
    transform: TransformSnapshot,
    point_world: Iterable[float],
) -> np.ndarray | None:
    """Project a world-frame 3D point onto the image plane (pinhole, no distortion)."""
    rotation_cam_to_world = quaternion_to_rotation_matrix(transform.rotation_xyzw)
    rotation_world_to_cam = rotation_cam_to_world.T
    point = np.asarray(point_world, dtype=float)
    point_cam = rotation_world_to_cam @ (point - transform.translation_m.astype(float))
    if point_cam[2] <= 1e-6:
        return None
    u = intrinsics.fx * point_cam[0] / point_cam[2] + intrinsics.cx
    v = intrinsics.fy * point_cam[1] / point_cam[2] + intrinsics.cy
    return np.array([u, v], dtype=float)
