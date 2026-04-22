import unittest

import cv2
import numpy as np

from omx_perception.camera_geometry import CameraIntrinsics, TransformSnapshot
from omx_perception.perception_pipeline import (
    ColorPrototype,
    compute_confidence,
    contour_center_in_workspace,
    estimate_block_pose,
    extract_rect_patch_ab,
    label_color_rect,
    load_color_prototypes,
    segment_foreground,
    split_large_blob,
    WorkspaceRect,
)


def _top_down_transform() -> TransformSnapshot:
    """Camera 0.30m above origin, optical axis pointing down (world -Z)."""
    return TransformSnapshot(
        reference_frame="world",
        translation_m=np.array([0.20, 0.0, 0.30], dtype=np.float64),
        rotation_xyzw=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
    )


def _synthetic_rect_contour(
    center_px: tuple[float, float],
    size_px: float,
) -> np.ndarray:
    cu, cv = center_px
    half = size_px / 2.0
    pts = np.array(
        [
            [cu - half, cv - half],
            [cu + half, cv - half],
            [cu + half, cv + half],
            [cu - half, cv + half],
        ],
        dtype=np.int32,
    )
    return pts.reshape(-1, 1, 2)


class EstimateBlockPoseTest(unittest.TestCase):
    def test_center_ray_intersects_plane_at_expected_xy(self) -> None:
        intrinsics = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
        transform = _top_down_transform()
        contour = _synthetic_rect_contour((320.0, 240.0), 60.0)

        estimate, reason = estimate_block_pose(
            contour,
            intrinsics,
            transform,
            block_size_m=0.030,
            table_z_m=0.18,
            rect_fill_min=0.70,
            aspect_ratio_min=0.60,
            aspect_ratio_max=1.66,
        )

        self.assertEqual(reason, "ok")
        self.assertIsNotNone(estimate)
        assert estimate is not None
        self.assertAlmostEqual(estimate.x_world, 0.20, places=3)
        self.assertAlmostEqual(estimate.y_world, 0.0, places=3)
        self.assertAlmostEqual(estimate.z_world, 0.18 + 0.015, places=4)

    def test_rejects_elongated_shape(self) -> None:
        intrinsics = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
        transform = _top_down_transform()
        pts = np.array(
            [[310, 200], [330, 200], [330, 280], [310, 280]],
            dtype=np.int32,
        ).reshape(-1, 1, 2)

        estimate, reason = estimate_block_pose(
            pts,
            intrinsics,
            transform,
            block_size_m=0.030,
            table_z_m=0.18,
            rect_fill_min=0.70,
            aspect_ratio_min=0.60,
            aspect_ratio_max=1.66,
        )

        self.assertIsNone(estimate)
        self.assertEqual(reason, "aspect_bad")

    def test_rejects_low_fill_shape(self) -> None:
        intrinsics = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
        transform = _top_down_transform()
        pts = np.array(
            [[300, 240], [340, 240], [340, 280]],
            dtype=np.int32,
        ).reshape(-1, 1, 2)

        estimate, reason = estimate_block_pose(
            pts,
            intrinsics,
            transform,
            block_size_m=0.030,
            table_z_m=0.18,
            rect_fill_min=0.70,
            aspect_ratio_min=0.60,
            aspect_ratio_max=1.66,
        )

        self.assertIsNone(estimate)
        self.assertEqual(reason, "fill_low")


class ForegroundAndWorkspaceTest(unittest.TestCase):
    def test_segment_foreground_can_skip_canny_work(self) -> None:
        image = np.zeros((80, 80, 3), dtype=np.uint8)
        image[20:60, 20:60] = (0, 0, 255)

        contours, edges, mask = segment_foreground(
            image,
            canny_sigma=0.33,
            saturation_min=20,
            compute_edges=False,
        )

        self.assertGreaterEqual(len(contours), 1)
        self.assertIsNone(edges)
        self.assertGreater(np.count_nonzero(mask), 0)

    def test_contour_center_in_workspace_accepts_centered_rect(self) -> None:
        intrinsics = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
        transform = _top_down_transform()
        contour = _synthetic_rect_contour((320.0, 240.0), 60.0)
        workspace = WorkspaceRect(
            x_min_m=0.10,
            x_max_m=0.30,
            y_min_m=-0.10,
            y_max_m=0.10,
            plane_z_m=0.21,
        )

        inside, hit = contour_center_in_workspace(
            contour,
            intrinsics,
            transform,
            workspace,
        )

        self.assertTrue(inside)
        self.assertIsNotNone(hit)

    def test_split_large_blob_returns_original_for_single_small_blob(self) -> None:
        contour = _synthetic_rect_contour((50.0, 50.0), 20.0)

        split = split_large_blob(
            contour,
            max_single_area_px=5000.0,
            min_area_px=50.0,
        )

        self.assertEqual(len(split), 1)
        np.testing.assert_array_equal(split[0], contour)


class RectPatchColorTest(unittest.TestCase):
    def test_extract_rect_patch_ab_returns_color_statistics(self) -> None:
        image = np.full((100, 100, 3), (0, 0, 255), dtype=np.uint8)
        rect = np.array(
            [[30, 30], [70, 30], [70, 70], [30, 70]],
            dtype=np.float32,
        )

        patch = extract_rect_patch_ab(image, rect, chroma_min=10.0)

        self.assertIsNotNone(patch)
        assert patch is not None
        a_star, b_star, sample_count = patch
        self.assertGreater(a_star, 0.0)
        self.assertGreater(b_star, 0.0)
        self.assertGreater(sample_count, 100)

    def test_solid_red_patch_matches_red_prototype(self) -> None:
        image = np.full((100, 100, 3), (0, 0, 255), dtype=np.uint8)
        lab = cv2.cvtColor(np.array([[[0, 0, 255]]], dtype=np.uint8), cv2.COLOR_BGR2LAB)
        a_red = float(lab[0, 0, 1]) - 128.0
        b_red = float(lab[0, 0, 2]) - 128.0
        prototypes = [
            ColorPrototype(name="red", a_star=a_red, b_star=b_red),
            ColorPrototype(name="green", a_star=-50.0, b_star=40.0),
            ColorPrototype(name="blue", a_star=20.0, b_star=-60.0),
        ]
        rect = np.array(
            [[30, 30], [70, 30], [70, 70], [30, 70]],
            dtype=np.float32,
        )

        name, ratio = label_color_rect(image, rect, prototypes)

        self.assertEqual(name, "red")
        self.assertGreater(ratio, 0.95)

    def test_rejects_achromatic_patch(self) -> None:
        image = np.full((100, 100, 3), (128, 128, 128), dtype=np.uint8)
        prototypes = [
            ColorPrototype(name="red", a_star=80.0, b_star=67.0),
            ColorPrototype(name="green", a_star=-50.0, b_star=40.0),
        ]
        rect = np.array(
            [[30, 30], [70, 30], [70, 70], [30, 70]],
            dtype=np.float32,
        )

        name, ratio = label_color_rect(image, rect, prototypes)

        self.assertIsNone(name)
        self.assertEqual(ratio, 0.0)

    def test_empty_prototypes(self) -> None:
        image = np.full((100, 100, 3), (0, 0, 255), dtype=np.uint8)
        rect = np.array(
            [[30, 30], [70, 30], [70, 70], [30, 70]],
            dtype=np.float32,
        )

        name, ratio = label_color_rect(image, rect, [])

        self.assertIsNone(name)
        self.assertEqual(ratio, 0.0)


class ConfidenceTest(unittest.TestCase):
    def test_perfect_case_returns_one(self) -> None:
        score = compute_confidence(
            rect_fill=1.0,
            rect_fill_min=0.75,
            aspect_ratio=1.0,
            color_ratio=1.0,
            color_ratio_min=0.4,
        )
        self.assertAlmostEqual(score, 1.0, places=6)

    def test_weakest_axis_dominates(self) -> None:
        score = compute_confidence(
            rect_fill=1.0,
            rect_fill_min=0.75,
            aspect_ratio=1.0,
            color_ratio=0.5,
            color_ratio_min=0.4,
        )
        self.assertAlmostEqual(score, 0.5, places=6)


class LoadColorPrototypesTest(unittest.TestCase):
    def test_loads_expected_schema(self) -> None:
        data = {
            "red": {"a_star": 80.0, "b_star": 67.0, "samples": 12},
            "green": {"a_star": -50.0, "b_star": 40.0},
        }

        prototypes = load_color_prototypes(data)

        self.assertEqual(len(prototypes), 2)
        self.assertEqual({p.name for p in prototypes}, {"red", "green"})

    def test_ignores_malformed_entries(self) -> None:
        data = {
            "red": {"a_star": 80.0, "b_star": 67.0},
            "bad": "not-a-dict",
            "missing": {"a_star": 10.0},
        }

        prototypes = load_color_prototypes(data)

        self.assertEqual(len(prototypes), 1)
        self.assertEqual(prototypes[0].name, "red")


if __name__ == "__main__":
    unittest.main()
