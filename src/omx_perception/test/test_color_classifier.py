from __future__ import annotations

import cv2
import numpy as np

from omx_perception.color_classifier import (
    ClassifierParams,
    ColorRef,
    classify,
    extract_valid_lab_pixels,
    polygon_inset,
)


def _solid_bgr(h: int, w: int, bgr: tuple[int, int, int]) -> np.ndarray:
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:, :] = bgr
    return img


def _full_polygon(h: int, w: int, margin: int = 10) -> np.ndarray:
    return np.array(
        [[margin, margin], [w - margin, margin], [w - margin, h - margin], [margin, h - margin]],
        dtype=float,
    )


def _ocv_lab_ab(bgr: tuple[int, int, int]) -> tuple[float, float]:
    pixel = np.array([[[bgr[0], bgr[1], bgr[2]]]], dtype=np.uint8)
    lab = cv2.cvtColor(pixel, cv2.COLOR_BGR2LAB)[0, 0]
    return float(lab[1]), float(lab[2])


def test_polygon_inset_moves_towards_centroid():
    pts = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])
    result = polygon_inset(pts, 0.5)
    centroid = np.array([5.0, 5.0])
    for orig, inset_pt in zip(pts, result):
        expected = centroid + 0.5 * (orig - centroid)
        np.testing.assert_allclose(inset_pt, expected, atol=1e-9)


def test_polygon_inset_ratio_1_is_identity():
    pts = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])
    np.testing.assert_allclose(polygon_inset(pts, 1.0), pts, atol=1e-9)


def test_polygon_inset_ratio_0_collapses_to_centroid():
    pts = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])
    result = polygon_inset(pts, 0.0)
    centroid = pts.mean(axis=0)
    for pt in result:
        np.testing.assert_allclose(pt, centroid, atol=1e-9)


def test_extract_returns_pixels_for_saturated_color():
    img = _solid_bgr(100, 100, (30, 30, 200))
    polygon = _full_polygon(100, 100)
    params = ClassifierParams(
        inset_ratio=0.7,
        saturation_min=30,
        luminance_low_percentile=10.0,
        luminance_high_percentile=95.0,
        min_valid_pixels=10,
        distance_threshold=30.0,
    )
    pixels = extract_valid_lab_pixels(img, polygon, params)
    assert pixels.shape[1] == 3
    assert len(pixels) >= params.min_valid_pixels


def test_extract_returns_empty_for_gray():
    img = _solid_bgr(100, 100, (128, 128, 128))
    polygon = _full_polygon(100, 100)
    params = ClassifierParams(
        inset_ratio=0.7,
        saturation_min=80,
        luminance_low_percentile=10.0,
        luminance_high_percentile=95.0,
        min_valid_pixels=60,
        distance_threshold=30.0,
    )
    pixels = extract_valid_lab_pixels(img, polygon, params)
    assert len(pixels) == 0


def test_classify_correct_color():
    bgr = (30, 30, 200)
    img = _solid_bgr(100, 100, bgr)
    polygon = _full_polygon(100, 100)
    a_ref, b_ref = _ocv_lab_ab(bgr)
    refs = [
        ColorRef("red", a_ref, b_ref),
        ColorRef("green", a_ref - 70.0, b_ref + 40.0),
        ColorRef("blue", a_ref + 40.0, b_ref - 70.0),
    ]
    params = ClassifierParams(
        inset_ratio=0.7,
        saturation_min=30,
        luminance_low_percentile=10.0,
        luminance_high_percentile=95.0,
        min_valid_pixels=10,
        distance_threshold=30.0,
    )
    name, conf = classify(img, polygon, refs, params)
    assert name == "red"
    assert conf > 0.0


def test_classify_returns_unknown_when_distance_exceeds_threshold():
    bgr = (30, 30, 200)
    img = _solid_bgr(100, 100, bgr)
    polygon = _full_polygon(100, 100)
    a_ref, b_ref = _ocv_lab_ab(bgr)
    refs = [ColorRef("red", a_ref + 100.0, b_ref + 100.0)]
    params = ClassifierParams(
        inset_ratio=0.7,
        saturation_min=30,
        luminance_low_percentile=10.0,
        luminance_high_percentile=95.0,
        min_valid_pixels=10,
        distance_threshold=10.0,
    )
    name, conf = classify(img, polygon, refs, params)
    assert name == "unknown"
    assert conf == 0.0


def test_classify_returns_unknown_when_too_few_valid_pixels():
    img = _solid_bgr(100, 100, (128, 128, 128))
    polygon = _full_polygon(100, 100)
    refs = [ColorRef("red", 170.0, 150.0)]
    params = ClassifierParams(
        inset_ratio=0.7,
        saturation_min=200,
        luminance_low_percentile=10.0,
        luminance_high_percentile=95.0,
        min_valid_pixels=60,
        distance_threshold=50.0,
    )
    name, conf = classify(img, polygon, refs, params)
    assert name == "unknown"
    assert conf == 0.0


def test_classify_confidence_is_1_at_zero_distance():
    bgr = (30, 30, 200)
    img = _solid_bgr(100, 100, bgr)
    polygon = _full_polygon(100, 100)
    a_ref, b_ref = _ocv_lab_ab(bgr)
    refs = [ColorRef("red", a_ref, b_ref)]
    params = ClassifierParams(
        inset_ratio=0.7,
        saturation_min=30,
        luminance_low_percentile=10.0,
        luminance_high_percentile=95.0,
        min_valid_pixels=10,
        distance_threshold=30.0,
    )
    name, conf = classify(img, polygon, refs, params)
    assert name == "red"
    assert conf > 0.9
