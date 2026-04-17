"""Robust color block detector using OpenCV.

The pipeline is split into two stages:
  1. Candidate extraction: find block-like colored blobs under loose thresholds
  2. Color classification: assign exactly one color per blob using HSV + LAB stats

This design is more stable than per-color independent contour extraction:
  - one physical block yields one contour
  - overlapping green/blue masks are resolved by a classifier instead of thresholds
  - hue can become unreliable under overexposure, so classification also uses LAB
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

HsvRange = Tuple[np.ndarray, np.ndarray]
HsvRangeMap = Dict[str, List[HsvRange]]

_TARGET_V = 110.0
_BOOST_MAX = 2.0

_CANDIDATE_H_MARGIN = 6
_CANDIDATE_S_MARGIN = 25
_CANDIDATE_V_MARGIN = 15
_CANDIDATE_MIN_SUPPORT = 250
_GENERIC_S_FLOOR = 40
_GENERIC_V_FLOOR = 50

_AB_WEIGHT_A = 0.25
_AB_WEIGHT_B = 1.50
_HUE_DISTANCE_SCALE = 1.25

_COLOR_PROTOTYPES = {
    "red": {
        "hue_centers": (0.0, 165.0),
        "lab_ab": (166.0, 120.0),
    },
    "green": {
        "hue_centers": (84.0,),
        "lab_ab": (75.0, 133.0),
    },
    "blue": {
        "hue_centers": (100.0,),
        "lab_ab": (120.0, 96.0),
    },
}


@dataclass
class DetectedBlock:
    color: str
    cx: int
    cy: int
    area: float
    confidence: float


@dataclass
class _ColorStats:
    hue_mean: float
    sat_mean: float
    val_mean: float
    lab_a_mean: float
    lab_b_mean: float
    support_pixels: int
    support_ratio: float


def default_hsv_ranges() -> HsvRangeMap:
    """Return HSV thresholds tuned for the current camera and lighting setup."""
    return {
        "red": [
            (np.array([0, 35, 80]), np.array([10, 255, 255])),
            (np.array([155, 35, 80]), np.array([180, 255, 255])),
        ],
        "green": [
            (np.array([75, 80, 70]), np.array([88, 255, 255])),
        ],
        "blue": [
            (np.array([90, 175, 70]), np.array([118, 255, 255])),
        ],
    }


MIN_CONTOUR_AREA = 2000
MIN_SOLIDITY = 0.30
ASPECT_MIN = 0.30
ASPECT_MAX = 3.00


def preprocess_hsv(bgr_image: np.ndarray) -> np.ndarray:
    """Apply illumination-robust preprocessing and return the boosted HSV image."""
    lab = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2LAB)
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
    lab[:, :, 0] = clahe.apply(lab[:, :, 0])
    contrast = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    hsv = cv2.cvtColor(contrast, cv2.COLOR_BGR2HSV).astype(np.float32)
    boost = np.clip(hsv[:, :, 2] / _TARGET_V, 1.0, _BOOST_MAX)
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * boost, 0, 255)
    return hsv.astype(np.uint8)


def preprocess(bgr_image: np.ndarray) -> np.ndarray:
    """Return the preprocessed image in BGR space for debug and compatibility."""
    hsv = preprocess_hsv(bgr_image)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def detect_blocks(
    bgr_image: np.ndarray,
    target_color: Optional[str] = None,
    hsv_ranges: Optional[HsvRangeMap] = None,
    min_area: int = MIN_CONTOUR_AREA,
) -> List[DetectedBlock]:
    """Detect colored blocks from a raw BGR image."""
    hsv = preprocess_hsv(bgr_image)
    blocks, _ = detect_blocks_and_masks_from_hsv(
        hsv,
        target_color=target_color,
        hsv_ranges=hsv_ranges,
        min_area=min_area,
    )
    return blocks


def detect_blocks_from_hsv(
    hsv: np.ndarray,
    target_color: Optional[str] = None,
    hsv_ranges: Optional[HsvRangeMap] = None,
    min_area: int = MIN_CONTOUR_AREA,
) -> List[DetectedBlock]:
    """Detect colored blocks from a preprocessed HSV image."""
    blocks, _ = detect_blocks_and_masks_from_hsv(
        hsv,
        target_color=target_color,
        hsv_ranges=hsv_ranges,
        min_area=min_area,
    )
    return blocks


def detect_blocks_and_masks(
    bgr_image: np.ndarray,
    target_color: Optional[str] = None,
    hsv_ranges: Optional[HsvRangeMap] = None,
    min_area: int = MIN_CONTOUR_AREA,
) -> tuple[List[DetectedBlock], Dict[str, np.ndarray]]:
    """Detect blocks from a raw BGR image and return final per-color masks."""
    hsv = preprocess_hsv(bgr_image)
    return detect_blocks_and_masks_from_hsv(
        hsv,
        target_color=target_color,
        hsv_ranges=hsv_ranges,
        min_area=min_area,
    )


def detect_blocks_and_masks_from_hsv(
    hsv: np.ndarray,
    target_color: Optional[str] = None,
    hsv_ranges: Optional[HsvRangeMap] = None,
    min_area: int = MIN_CONTOUR_AREA,
) -> tuple[List[DetectedBlock], Dict[str, np.ndarray]]:
    """Detect blocks from a preprocessed HSV image and return final per-color masks."""
    ranges = hsv_ranges if hsv_ranges is not None else default_hsv_ranges()
    pre_bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    lab = cv2.cvtColor(pre_bgr, cv2.COLOR_BGR2LAB)

    candidate_mask, raw_support_mask = _build_candidate_masks(hsv, ranges)
    contours, _ = cv2.findContours(candidate_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    results: List[DetectedBlock] = []
    final_masks = {
        color: np.zeros(hsv.shape[:2], dtype=np.uint8)
        for color in ranges.keys()
    }
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        hull_area = cv2.contourArea(cv2.convexHull(cnt))
        solidity = area / hull_area if hull_area > 0 else 0.0
        if solidity < MIN_SOLIDITY:
            continue

        _, _, w, h = cv2.boundingRect(cnt)
        aspect = w / h if h > 0 else 0.0
        if not (ASPECT_MIN <= aspect <= ASPECT_MAX):
            continue

        moments = cv2.moments(cnt)
        if moments["m00"] == 0:
            continue

        contour_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        cv2.drawContours(contour_mask, [cnt], -1, 255, -1)

        support_mask = _extract_support_mask(hsv, raw_support_mask, contour_mask)
        support_pixels = int(np.count_nonzero(support_mask))
        if support_pixels < _CANDIDATE_MIN_SUPPORT:
            continue

        stats = _compute_color_stats(hsv, lab, support_mask, area)
        color, confidence = _classify_block(stats, area, solidity)
        if target_color and color != target_color:
            continue

        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        results.append(
            DetectedBlock(
                color=color,
                cx=cx,
                cy=cy,
                area=area,
                confidence=confidence,
            )
        )
        final_masks[color] = cv2.bitwise_or(final_masks[color], contour_mask)

    results.sort(key=lambda block: block.area, reverse=True)
    if target_color:
        filtered_masks = {
            target_color: final_masks.get(target_color, np.zeros(hsv.shape[:2], dtype=np.uint8))
        }
        return results, filtered_masks
    return results, final_masks


def build_color_mask(
    bgr_image: np.ndarray,
    color: str,
    hsv_ranges: Optional[HsvRangeMap] = None,
) -> np.ndarray:
    """Build a cleaned binary mask for one color from a raw BGR image."""
    hsv = preprocess_hsv(bgr_image)
    return build_color_mask_from_hsv(hsv, color, hsv_ranges=hsv_ranges)


def build_color_mask_from_hsv(
    hsv: np.ndarray,
    color: str,
    hsv_ranges: Optional[HsvRangeMap] = None,
) -> np.ndarray:
    """Build a cleaned binary mask for one color from a preprocessed HSV image."""
    ranges = hsv_ranges if hsv_ranges is not None else default_hsv_ranges()
    if color not in ranges:
        return np.zeros(hsv.shape[:2], dtype=np.uint8)
    return _clean_mask(_build_mask(hsv, ranges[color]))


def draw_detections(bgr_image: np.ndarray, blocks: List[DetectedBlock]) -> np.ndarray:
    out = bgr_image.copy()
    color_bgr = {"red": (0, 0, 220), "green": (0, 200, 0), "blue": (220, 80, 0)}
    for block in blocks:
        color = color_bgr.get(block.color, (255, 255, 255))
        cv2.circle(out, (block.cx, block.cy), 8, color, -1)
        cv2.circle(out, (block.cx, block.cy), 8, (255, 255, 255), 1)
        cv2.putText(
            out,
            f"{block.color} {block.confidence:.2f}",
            (block.cx + 10, block.cy - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )
    return out


def _build_mask(hsv: np.ndarray, ranges: List[HsvRange]) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in ranges:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
    return mask


def _build_candidate_masks(hsv: np.ndarray, hsv_ranges: HsvRangeMap) -> tuple[np.ndarray, np.ndarray]:
    raw = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for ranges in hsv_ranges.values():
        for lo, hi in ranges:
            relaxed_lo = lo.copy()
            relaxed_hi = hi.copy()
            relaxed_lo[0] = max(0, int(relaxed_lo[0]) - _CANDIDATE_H_MARGIN)
            relaxed_hi[0] = min(180, int(relaxed_hi[0]) + _CANDIDATE_H_MARGIN)
            relaxed_lo[1] = max(0, int(relaxed_lo[1]) - _CANDIDATE_S_MARGIN)
            relaxed_lo[2] = max(0, int(relaxed_lo[2]) - _CANDIDATE_V_MARGIN)
            raw = cv2.bitwise_or(raw, cv2.inRange(hsv, relaxed_lo, relaxed_hi))

    generic = cv2.inRange(
        hsv,
        np.array([0, _GENERIC_S_FLOOR, _GENERIC_V_FLOOR], dtype=np.uint8),
        np.array([180, 255, 255], dtype=np.uint8),
    )
    raw = cv2.bitwise_and(raw, generic)
    return _clean_candidate_mask(raw), raw


def _extract_support_mask(hsv: np.ndarray, raw_support_mask: np.ndarray, contour_mask: np.ndarray) -> np.ndarray:
    support = cv2.bitwise_and(raw_support_mask, raw_support_mask, mask=contour_mask)

    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    highlight = ((val >= 245) & (sat <= 80)).astype(np.uint8) * 255
    reliable = ((sat >= _GENERIC_S_FLOOR) & (val >= _GENERIC_V_FLOOR)).astype(np.uint8) * 255
    support = cv2.bitwise_and(support, reliable)
    support = cv2.bitwise_and(support, cv2.bitwise_not(highlight))

    if np.count_nonzero(support) >= _CANDIDATE_MIN_SUPPORT:
        return support

    fallback = cv2.bitwise_and(contour_mask, reliable)
    fallback = cv2.bitwise_and(fallback, cv2.bitwise_not(highlight))
    return fallback


def _compute_color_stats(
    hsv: np.ndarray,
    lab: np.ndarray,
    support_mask: np.ndarray,
    contour_area: float,
) -> _ColorStats:
    ys, xs = np.where(support_mask > 0)

    hue = hsv[ys, xs, 0].astype(np.float32)
    sat = hsv[ys, xs, 1].astype(np.float32)
    val = hsv[ys, xs, 2].astype(np.float32)
    lab_a = lab[ys, xs, 1].astype(np.float32)
    lab_b = lab[ys, xs, 2].astype(np.float32)

    chroma = np.sqrt((lab_a - 128.0) ** 2 + (lab_b - 128.0) ** 2)
    weights = np.clip((sat - 20.0) / 235.0, 0.05, 1.0) * np.clip(chroma / 60.0, 0.25, 1.5)

    hue_mean = _weighted_circular_mean(hue, weights)
    sat_mean = float(np.average(sat, weights=weights))
    val_mean = float(np.average(val, weights=weights))
    lab_a_mean = float(np.average(lab_a, weights=weights))
    lab_b_mean = float(np.average(lab_b, weights=weights))
    support_ratio = float(np.clip(len(xs) / max(contour_area, 1.0), 0.0, 1.5))

    return _ColorStats(
        hue_mean=hue_mean,
        sat_mean=sat_mean,
        val_mean=val_mean,
        lab_a_mean=lab_a_mean,
        lab_b_mean=lab_b_mean,
        support_pixels=len(xs),
        support_ratio=support_ratio,
    )


def _classify_block(stats: _ColorStats, area: float, solidity: float) -> tuple[str, float]:
    hue_weight = float(np.clip((stats.sat_mean - 70.0) / 120.0, 0.0, 1.0))

    distances: Dict[str, float] = {}
    for color, prototype in _COLOR_PROTOTYPES.items():
        lab_distance = _weighted_ab_distance(
            stats.lab_a_mean,
            stats.lab_b_mean,
            prototype["lab_ab"][0],
            prototype["lab_ab"][1],
        )
        hue_distance = min(
            _circular_hue_distance(stats.hue_mean, center)
            for center in prototype["hue_centers"]
        )
        distances[color] = lab_distance + _HUE_DISTANCE_SCALE * hue_weight * hue_distance

    ranking = sorted(distances.items(), key=lambda item: item[1])
    best_color, best_distance = ranking[0]
    second_distance = ranking[1][1] if len(ranking) > 1 else best_distance + 1.0

    margin = max(0.0, second_distance - best_distance)
    margin_score = float(np.clip(margin / 25.0, 0.0, 1.0))
    area_score = min(area / 15_000.0, 1.0)
    support_score = float(np.clip(stats.support_ratio, 0.0, 1.0))
    confidence = float(
        np.clip(
            0.45 * margin_score + 0.25 * support_score + 0.20 * solidity + 0.10 * area_score,
            0.0,
            1.0,
        )
    )

    return best_color, confidence


def _clean_mask(mask: np.ndarray) -> np.ndarray:
    """Remove speckle noise and reconnect ring-shaped overexposed masks."""
    k_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    k_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_small, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_large, iterations=3)
    return mask


def _clean_candidate_mask(mask: np.ndarray) -> np.ndarray:
    """Clean candidate blobs without merging nearby blocks into one contour."""
    k_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    k_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_small, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_large, iterations=1)
    return mask


def _weighted_circular_mean(hue: np.ndarray, weights: np.ndarray) -> float:
    angles = hue * (2.0 * np.pi / 180.0)
    sin_sum = np.sum(np.sin(angles) * weights)
    cos_sum = np.sum(np.cos(angles) * weights)
    angle = np.arctan2(sin_sum, cos_sum)
    if angle < 0:
        angle += 2.0 * np.pi
    return float(angle * 180.0 / (2.0 * np.pi))


def _circular_hue_distance(hue_a: float, hue_b: float) -> float:
    diff = abs(hue_a - hue_b)
    return min(diff, 180.0 - diff)


def _weighted_ab_distance(a0: float, b0: float, a1: float, b1: float) -> float:
    delta_a = a0 - a1
    delta_b = b0 - b1
    return float(np.sqrt(_AB_WEIGHT_A * delta_a * delta_a + _AB_WEIGHT_B * delta_b * delta_b))
