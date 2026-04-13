"""HSV-based color block detector using OpenCV."""

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class DetectedBlock:
    color: str
    cx: int  # pixel x
    cy: int  # pixel y
    area: float
    confidence: float


# HSV ranges for each color (lower, upper)
# Each entry can be a single (lower, upper) or two ranges for red (hue wrap)
HSV_RANGES = {
    "red": [
        (np.array([0, 100, 80]), np.array([10, 255, 255])),
        (np.array([170, 100, 80]), np.array([180, 255, 255])),
    ],
    "green": [
        (np.array([40, 80, 60]), np.array([85, 255, 255])),
    ],
    "blue": [
        (np.array([100, 100, 60]), np.array([130, 255, 255])),
    ],
}

MIN_CONTOUR_AREA = 500  # px^2 — filters out noise


def detect_blocks(
    bgr_image: np.ndarray,
    target_color: Optional[str] = None,
) -> List[DetectedBlock]:
    """Detect colored blocks in a BGR image.

    Args:
        bgr_image: Input image in BGR format (from cv_bridge).
        target_color: If set, only detect this color. Empty string or None → all.

    Returns:
        List of DetectedBlock, sorted by descending area.
    """
    hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
    colors_to_check = (
        [target_color]
        if target_color
        else list(HSV_RANGES.keys())
    )

    results: List[DetectedBlock] = []

    for color in colors_to_check:
        if color not in HSV_RANGES:
            continue

        mask = _build_mask(hsv, color)
        mask = _clean_mask(mask)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_CONTOUR_AREA:
                continue

            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            confidence = _compute_confidence(area, mask, cnt)
            results.append(DetectedBlock(color, cx, cy, area, confidence))

    results.sort(key=lambda b: b.area, reverse=True)
    return results


def _build_mask(hsv: np.ndarray, color: str) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in HSV_RANGES[color]:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
    return mask


def _clean_mask(mask: np.ndarray) -> np.ndarray:
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def _compute_confidence(
    area: float,
    mask: np.ndarray,
    contour: np.ndarray,
) -> float:
    """Estimate confidence from contour solidity and area magnitude."""
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    solidity = area / hull_area if hull_area > 0 else 0.0
    # Scale area to [0, 1] with saturation at 20_000 px^2
    area_score = min(area / 20_000.0, 1.0)
    return float(np.clip(0.6 * solidity + 0.4 * area_score, 0.0, 1.0))


def draw_detections(
    bgr_image: np.ndarray,
    blocks: List[DetectedBlock],
) -> np.ndarray:
    """Draw bounding boxes and labels on image (for debug visualization)."""
    out = bgr_image.copy()
    color_bgr: dict = {
        "red": (0, 0, 220),
        "green": (0, 200, 0),
        "blue": (220, 0, 0),
    }
    for b in blocks:
        c = color_bgr.get(b.color, (255, 255, 255))
        cv2.circle(out, (b.cx, b.cy), 6, c, -1)
        cv2.putText(
            out,
            f"{b.color} {b.confidence:.2f}",
            (b.cx + 8, b.cy - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            c,
            1,
        )
    return out
