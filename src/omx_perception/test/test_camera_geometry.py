import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from omx_perception.camera_geometry import (
    CameraIntrinsics,
    FrameConfig,
    Plane,
    TransformSnapshot,
    intersect_ray_with_plane,
    quaternion_to_rotation_matrix,
    ray_direction_in_camera,
    ray_in_reference_frame,
)


class CameraGeometryTest(unittest.TestCase):
    def test_center_pixel_points_forward(self) -> None:
        intrinsics = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
        ray = ray_direction_in_camera(intrinsics, pixel_u=320.0, pixel_v=240.0)
        np.testing.assert_allclose(ray, np.array([0.0, 0.0, 1.0]), atol=1e-7)

    def test_quaternion_to_rotation_matrix_rotates_z_axis(self) -> None:
        quaternion = np.array([0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)])
        rotation = quaternion_to_rotation_matrix(quaternion)
        rotated = rotation @ np.array([1.0, 0.0, 0.0])
        np.testing.assert_allclose(rotated, np.array([0.0, 1.0, 0.0]), atol=1e-7)

    def test_ray_projects_to_table_plane(self) -> None:
        intrinsics = CameraIntrinsics(fx=100.0, fy=100.0, cx=50.0, cy=50.0)
        transform = TransformSnapshot(
            reference_frame="world",
            translation_m=np.array([0.0, 0.0, 0.5]),
            rotation_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
        )
        plane = Plane(
            frame_id="world",
            normal_xyz=np.array([0.0, 0.0, 1.0]),
            point_xyz=np.array([0.0, 0.0, 1.0]),
        )

        origin, direction = ray_in_reference_frame(
            intrinsics,
            transform,
            pixel_u=50.0,
            pixel_v=50.0,
        )
        intersection = intersect_ray_with_plane(origin, direction, plane)

        np.testing.assert_allclose(origin, np.array([0.0, 0.0, 0.5]), atol=1e-7)
        np.testing.assert_allclose(direction, np.array([0.0, 0.0, 1.0]), atol=1e-7)
        np.testing.assert_allclose(intersection, np.array([0.0, 0.0, 1.0]), atol=1e-7)

    def test_yaml_loaders_accept_expected_schema(self) -> None:
        camera_info_yaml = """
camera_name: default_cam
camera_matrix:
  data: [600.0, 0.0, 320.0, 0.0, 610.0, 240.0, 0.0, 0.0, 1.0]
"""
        extrinsics_yaml = """
frames:
  camera_frame: default_cam
  reference_frame: world
  robot_root_frame: link0
  camera_parent_frame: link5
table_plane:
  frame_id: world
  normal_xyz: [0.0, 0.0, 1.0]
  point_xyz: [0.0, 0.0, 0.0]
"""
        with tempfile.TemporaryDirectory() as temp_dir:
            camera_info_path = Path(temp_dir) / "camera.yaml"
            extrinsics_path = Path(temp_dir) / "extrinsics.yaml"
            camera_info_path.write_text(camera_info_yaml, encoding="utf-8")
            extrinsics_path.write_text(extrinsics_yaml, encoding="utf-8")

            intrinsics = CameraIntrinsics.from_camera_info_yaml(camera_info_path)
            frame_config = FrameConfig.from_yaml(extrinsics_path)
            plane = Plane.from_yaml(extrinsics_path)

        self.assertEqual(intrinsics.frame_id, "default_cam")
        self.assertEqual(frame_config.reference_frame, "world")
        self.assertEqual(frame_config.robot_root_frame, "link0")
        self.assertEqual(plane.frame_id, "world")
        self.assertAlmostEqual(intrinsics.fx, 600.0)


if __name__ == "__main__":
    unittest.main()
