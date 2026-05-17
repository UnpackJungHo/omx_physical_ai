from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import NamedTuple

import cv2
import numpy as np
import yaml


class ColorRef(NamedTuple):
    name: str
    a: float  # OpenCV 8-bit LAB a* (0-255, neutral=128)
    b: float  # OpenCV 8-bit LAB b* (0-255, neutral=128)


@dataclass
class ClassifierParams:
    inset_ratio: float = 0.7
    saturation_min: int = 30
    luminance_low_percentile: float = 10.0
    luminance_high_percentile: float = 95.0
    min_valid_pixels: int = 60
    distance_threshold: float = 20.0


def load_reference_yaml(
    path: str | Path,
) -> tuple[list[ColorRef], ClassifierParams] | tuple[None, None]:
    """Load color references and classifier params from yaml."""
    try:
        with Path(path).expanduser().open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except Exception:
        return None, None

    try:
        params = ClassifierParams(
            inset_ratio=float(data.get("inset_ratio", 0.7)),
            saturation_min=int(data.get("saturation_min", 30)),
            luminance_low_percentile=float(data.get("luminance_low_percentile", 10.0)),
            luminance_high_percentile=float(data.get("luminance_high_percentile", 95.0)),
            min_valid_pixels=int(data.get("min_valid_pixels", 60)),
            distance_threshold=float(data.get("distance_threshold", 20.0)),
        )

        refs: list[ColorRef] = []
        for entry in data.get("references", []):
            ab = entry["lab_ab"]
            refs.append(ColorRef(name=str(entry["name"]), a=float(ab[0]), b=float(ab[1])))
    except Exception:
        return None, None

    return refs, params


def polygon_inset(pts: np.ndarray, ratio: float) -> np.ndarray:
    """Scale polygon towards its centroid by ratio."""
    centroid = pts.mean(axis=0)
    return centroid + ratio * (pts - centroid)


def extract_valid_lab_pixels(
    bgr_image: np.ndarray,
    polygon_pts: np.ndarray,
    params: ClassifierParams,
) -> np.ndarray:
    """Return LAB pixels after polygon inset and illumination filtering."""
    h, w = bgr_image.shape[:2]

    inset_pts = polygon_inset(polygon_pts.copy().astype(float), params.inset_ratio)
    inset_pts[:, 0] = np.clip(inset_pts[:, 0], 0, w - 1)
    inset_pts[:, 1] = np.clip(inset_pts[:, 1], 0, h - 1)

    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, [inset_pts.astype(np.int32)], 255)

    if cv2.countNonZero(mask) == 0:
        return np.empty((0, 3), dtype=np.uint8)

    lab_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2LAB)
    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

    roi_lab = lab_image[mask == 255]
    roi_hsv = hsv_image[mask == 255]
    if len(roi_lab) == 0:
        return np.empty((0, 3), dtype=np.uint8)

    l_vals = roi_lab[:, 0].astype(float)
    s_vals = roi_hsv[:, 1].astype(float)
    l_low = np.percentile(l_vals, params.luminance_low_percentile)
    l_high = np.percentile(l_vals, params.luminance_high_percentile)

    valid = (
        (l_vals >= l_low)
        & (l_vals <= l_high)
        & (s_vals >= params.saturation_min)
    )
    return roi_lab[valid]


def classify(
    bgr_image: np.ndarray,
    polygon_pts: np.ndarray,
    refs: list[ColorRef],
    params: ClassifierParams,
) -> tuple[str, float]:
    """Classify the color of a box ROI as a reference name or unknown."""
    valid_pixels = extract_valid_lab_pixels(bgr_image, polygon_pts, params)
    if len(valid_pixels) < params.min_valid_pixels or not refs:
        return "unknown", 0.0

    median_a = float(np.median(valid_pixels[:, 1]))
    median_b = float(np.median(valid_pixels[:, 2]))

    best_name = "unknown"
    best_dist = math.inf
    for ref in refs:
        dist = math.sqrt((median_a - ref.a) ** 2 + (median_b - ref.b) ** 2)
        if dist < best_dist:
            best_dist = dist
            best_name = ref.name

    if best_dist > params.distance_threshold:
        return "unknown", 0.0

    confidence = max(0.0, min(1.0, 1.0 - best_dist / params.distance_threshold))
    return best_name, confidence
