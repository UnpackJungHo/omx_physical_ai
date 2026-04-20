from __future__ import annotations

import json
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from omx_perception.camera_geometry import (
    CameraIntrinsics,
    Plane,
    TransformSnapshot,
    intersect_ray_with_plane,
    quaternion_to_rotation_matrix,
    ray_in_reference_frame,
)

# Top-face 4 corners in object space (Z=0 = top surface, Z+ points into block).
_BLOCK_HALF_M = 0.015
_TOP_FACE_OBJ = np.array(
    [
        [-_BLOCK_HALF_M, -_BLOCK_HALF_M, 0.0],
        [_BLOCK_HALF_M, -_BLOCK_HALF_M, 0.0],
        [_BLOCK_HALF_M, _BLOCK_HALF_M, 0.0],
        [-_BLOCK_HALF_M, _BLOCK_HALF_M, 0.0],
    ],
    dtype=np.float64,
)

_DEBUG_BGR: dict[str, tuple[int, int, int]] = {
    "red": (0, 0, 255),
    "green": (0, 200, 0),
    "blue": (255, 80, 0),
}

_DEFAULT_RANGES: dict = {
    "red": [
        {"h": [0, 15], "s": [50, 255], "v": [40, 255]},
        {"h": [160, 180], "s": [50, 255], "v": [40, 255]},
    ],
    "green": [{"h": [42, 95], "s": [16, 255], "v": [50, 255]}],
    "blue": [{"h": [95, 135], "s": [60, 255], "v": [40, 255]}],
}


def _normalized_roi_to_pixels(
    image_shape: tuple[int, int, int],
    roi_norm: tuple[float, float, float, float] | None,
) -> tuple[int, int, int, int] | None:
    if roi_norm is None:
        return None

    h, w = image_shape[:2]
    x_min_n, y_min_n, x_max_n, y_max_n = roi_norm
    x_min_n = float(np.clip(x_min_n, 0.0, 1.0))
    y_min_n = float(np.clip(y_min_n, 0.0, 1.0))
    x_max_n = float(np.clip(x_max_n, 0.0, 1.0))
    y_max_n = float(np.clip(y_max_n, 0.0, 1.0))

    if x_max_n <= x_min_n or y_max_n <= y_min_n:
        return None

    x_min = int(round(x_min_n * w))
    y_min = int(round(y_min_n * h))
    x_max = int(round(x_max_n * w))
    y_max = int(round(y_max_n * h))
    return x_min, y_min, x_max, y_max


def _apply_roi_to_mask(
    mask: np.ndarray,
    roi_rect: tuple[int, int, int, int] | None,
) -> np.ndarray:
    if roi_rect is None:
        return mask

    x_min, y_min, x_max, y_max = roi_rect
    roi_mask = np.zeros_like(mask)
    roi_mask[y_min:y_max, x_min:x_max] = 255
    return cv2.bitwise_and(mask, roi_mask)


def _point_in_roi(
    x: float,
    y: float,
    roi_rect: tuple[int, int, int, int] | None,
) -> bool:
    if roi_rect is None:
        return True
    x_min, y_min, x_max, y_max = roi_rect
    return x_min <= x <= x_max and y_min <= y <= y_max


def _draw_roi_overlay(
    image: np.ndarray,
    roi_rect: tuple[int, int, int, int] | None,
) -> np.ndarray:
    if roi_rect is None:
        return image

    x_min, y_min, x_max, y_max = roi_rect
    overlay = image.copy()
    cv2.rectangle(overlay, (x_min, y_min), (x_max, y_max), (0, 255, 255), thickness=-1)
    blended = cv2.addWeighted(overlay, 0.08, image, 0.92, 0.0)
    cv2.rectangle(blended, (x_min, y_min), (x_max, y_max), (0, 255, 255), 2)
    cv2.putText(
        blended,
        "ROI",
        (x_min + 8, max(y_min - 8, 18)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        2,
    )
    return blended


def _draw_reject(
    image: np.ndarray,
    contour: np.ndarray,
    reason: str,
) -> None:
    rect = cv2.boundingRect(contour)
    x, y, _, _ = rect
    cv2.drawContours(image, [cv2.convexHull(contour)], -1, (160, 160, 160), 1)
    cv2.putText(
        image,
        reason,
        (x, max(y - 4, 12)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (180, 180, 180),
        1,
    )


def _make_mask_debug_image(
    enhanced: np.ndarray,
    strict_mask: np.ndarray,
    dominance_mask: np.ndarray,
    combined_mask: np.ndarray,
    color: str,
) -> np.ndarray:
    strict_bgr = cv2.cvtColor(strict_mask, cv2.COLOR_GRAY2BGR)
    dominance_bgr = cv2.cvtColor(dominance_mask, cv2.COLOR_GRAY2BGR)
    combined_bgr = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)

    panels = [
        ("enhanced", enhanced.copy()),
        ("strict", strict_bgr),
        ("dominance", dominance_bgr),
        ("combined", combined_bgr),
    ]
    rendered: list[np.ndarray] = []
    for label, panel in panels:
        frame = panel.copy()
        cv2.putText(
            frame,
            f"{color}:{label}",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )
        rendered.append(frame)
    return np.hstack(rendered)


def _load_color_ranges(path: str) -> dict:
    p = Path(path)
    if p.is_file():
        data = yaml.safe_load(p.read_text())
        if isinstance(data, dict):
            return data
    return _DEFAULT_RANGES


def _first_range_or_default(color_ranges: dict, color: str, index: int) -> dict:
    ranges = color_ranges.get(color, _DEFAULT_RANGES[color])
    if index < len(ranges):
        return ranges[index]
    defaults = _DEFAULT_RANGES[color]
    if index < len(defaults):
        return defaults[index]
    return {"h": [0, 179], "s": [0, 255], "v": [0, 255]}


def _gray_world_white_balance(bgr: np.ndarray) -> np.ndarray:
    bgr_f = bgr.astype(np.float32) + 1.0
    means = bgr_f.reshape(-1, 3).mean(axis=0)
    scale = np.clip(means.mean() / means, 0.75, 1.35)
    balanced = np.clip(bgr_f * scale.reshape(1, 1, 3), 0.0, 255.0)
    return balanced.astype(np.uint8)


def _apply_clahe(bgr: np.ndarray) -> np.ndarray:
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    return cv2.cvtColor(cv2.merge([clahe.apply(l), a, b]), cv2.COLOR_LAB2BGR)


def _prepare_image(bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    white_balanced = _gray_world_white_balance(bgr)
    enhanced = _apply_clahe(white_balanced)
    hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(enhanced, cv2.COLOR_BGR2LAB)
    return enhanced, hsv, lab


def _build_hsv_mask(hsv: np.ndarray, ranges: list[dict]) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for r in ranges:
        lo = np.array([r["h"][0], r["s"][0], r["v"][0]], dtype=np.uint8)
        hi = np.array([r["h"][1], r["s"][1], r["v"][1]], dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
    return mask


def _build_dominance_mask(bgr: np.ndarray, hsv: np.ndarray, lab: np.ndarray, color: str) -> np.ndarray:
    bgr_f = bgr.astype(np.float32)
    b, g, r = cv2.split(bgr_f)
    total = b + g + r + 1.0
    nb = b / total
    ng = g / total
    nr = r / total

    sat = hsv[:, :, 1].astype(np.float32)
    val = hsv[:, :, 2].astype(np.float32)
    lab_a = lab[:, :, 1].astype(np.float32)
    lab_b = lab[:, :, 2].astype(np.float32)

    if color == "red":
        dominant = (
            ((nr - np.maximum(nb, ng)) > 0.018)
            & (lab_a > 132.0)
            & (val > 45.0)
        )
        relaxed = (r > g + 6.0) & (r > b + 6.0) & (lab_a > 128.0) & (sat > 18.0)
    elif color == "green":
        dominant = (
            ((ng - np.maximum(nb, nr)) > 0.015)
            & (lab_a < 122.0)
            & (val > 40.0)
        )
        relaxed = (g > r + 5.0) & (g > b + 5.0) & (lab_a < 126.0) & (sat > 16.0)
    elif color == "blue":
        dominant = (
            ((nb - np.maximum(nr, ng)) > 0.018)
            & (lab_b < 122.0)
            & (val > 40.0)
        )
        relaxed = (b > r + 6.0) & (b > g + 6.0) & (lab_b < 126.0) & (sat > 14.0)
    else:
        return np.zeros(hsv.shape[:2], dtype=np.uint8)

    return np.where(dominant | relaxed, 255, 0).astype(np.uint8)


def _stabilize_mask(mask: np.ndarray) -> np.ndarray:
    kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=2)
    return cv2.medianBlur(mask, 5)


def _build_mask(
    strict_mask: np.ndarray,
    dominance_mask: np.ndarray,
    expand_kernel_size: int,
    expand_iterations: int,
    min_strict_pixels: int,
) -> np.ndarray:
    strict_mask = _stabilize_mask(strict_mask)
    dominance_mask = _stabilize_mask(dominance_mask)

    if int(np.count_nonzero(strict_mask)) < min_strict_pixels:
        return strict_mask

    kernel_size = max(3, int(expand_kernel_size))
    if kernel_size % 2 == 0:
        kernel_size += 1

    grow_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    neighborhood = cv2.dilate(strict_mask, grow_kernel, iterations=max(1, int(expand_iterations)))
    constrained_dominance = cv2.bitwise_and(dominance_mask, neighborhood)
    combined = cv2.bitwise_or(strict_mask, constrained_dominance)
    return _stabilize_mask(combined)


def _contour_to_mask(
    image_shape: tuple[int, int],
    contour: np.ndarray,
) -> np.ndarray:
    mask = np.zeros(image_shape, dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, thickness=-1)
    return mask


def _largest_contour(mask: np.ndarray) -> np.ndarray | None:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    return max(contours, key=cv2.contourArea)


def _extract_candidate_core(
    contour: np.ndarray,
    strict_mask: np.ndarray,
    hsv: np.ndarray,
) -> tuple[np.ndarray | None, np.ndarray, float]:
    contour_mask = _contour_to_mask(strict_mask.shape, contour)
    strict_inside = cv2.bitwise_and(strict_mask, contour_mask)
    strict_px = np.count_nonzero(strict_inside)
    if strict_px < 25:
        return None, np.zeros_like(strict_mask), 0.0

    values = hsv[:, :, 2][strict_inside > 0]
    high_v_thresh = float(np.percentile(values, 65.0))
    core_mask = np.zeros_like(strict_mask)
    core_mask[(strict_inside > 0) & (hsv[:, :, 2] >= high_v_thresh)] = 255
    core_mask = cv2.morphologyEx(
        core_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        iterations=1,
    )
    core_mask = cv2.morphologyEx(
        core_mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
        iterations=1,
    )

    if np.count_nonzero(core_mask) < 20:
        fallback = cv2.erode(
            strict_inside,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
            iterations=1,
        )
        core_mask = fallback if np.count_nonzero(fallback) >= 20 else strict_inside

    core_contour = _largest_contour(core_mask)
    if core_contour is None:
        return None, np.zeros_like(strict_mask), 0.0

    refined = _contour_to_mask(strict_mask.shape, core_contour)
    core_area = float(np.count_nonzero(refined))
    return core_contour, refined, core_area


def _classify_core_color(
    core_mask: np.ndarray,
    strict_masks: dict[str, np.ndarray],
    dominance_masks: dict[str, np.ndarray],
) -> tuple[str, float, float, dict[str, float]]:
    core_px = float(np.count_nonzero(core_mask))
    if core_px < 1.0:
        return "unknown", 0.0, 0.0, {}

    scores: dict[str, float] = {}
    for color in strict_masks:
        strict_ratio = float(np.count_nonzero(cv2.bitwise_and(strict_masks[color], core_mask)) / core_px)
        dominance_ratio = float(np.count_nonzero(cv2.bitwise_and(dominance_masks[color], core_mask)) / core_px)
        scores[color] = 0.72 * strict_ratio + 0.28 * dominance_ratio

    ranked = sorted(scores.items(), key=lambda item: item[1], reverse=True)
    best_color, best_score = ranked[0]
    second_score = ranked[1][1] if len(ranked) > 1 else 0.0
    return best_color, float(best_score), float(best_score - second_score), scores


def _compute_side_support_metrics(
    contour_mask: np.ndarray,
    core_mask: np.ndarray,
    hsv: np.ndarray,
) -> dict[str, float]:
    contour_px = float(np.count_nonzero(contour_mask))
    core_px = float(np.count_nonzero(core_mask))
    if contour_px < 1.0 or core_px < 1.0:
        return {
            "support_ratio": 0.0,
            "side_area_ratio": 0.0,
            "side_value_drop": 0.0,
        }

    dilated_core = cv2.dilate(
        core_mask,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
        iterations=1,
    )
    side_mask = cv2.bitwise_and(contour_mask, cv2.bitwise_not(dilated_core))
    side_px = float(np.count_nonzero(side_mask))

    value_plane = hsv[:, :, 2].astype(np.float32)
    core_v = float(value_plane[core_mask > 0].mean()) if core_px > 0.0 else 0.0
    side_v = float(value_plane[side_mask > 0].mean()) if side_px > 0.0 else core_v
    value_drop = max(0.0, core_v - side_v)

    return {
        "support_ratio": contour_px / core_px,
        "side_area_ratio": side_px / core_px,
        "side_value_drop": value_drop,
    }


def _component_needs_split(
    contour: np.ndarray,
    rect: tuple,
    min_area: int,
    max_area: int,
    max_aspect_ratio: float,
) -> bool:
    area = float(cv2.contourArea(contour))
    rect_w, rect_h = rect[1]
    if rect_w < 1e-6 or rect_h < 1e-6:
        return False
    aspect = float(max(rect_w, rect_h) / max(min(rect_w, rect_h), 1e-6))
    return area > max_area * 1.15 or aspect > max(max_aspect_ratio * 1.15, 2.4) or area > min_area * 8.0


def _split_component_contour(
    combined_mask: np.ndarray,
    strict_mask: np.ndarray,
    contour: np.ndarray,
    min_area: int,
) -> list[np.ndarray]:
    x, y, w, h = cv2.boundingRect(contour)
    pad = 4
    x0 = max(0, x - pad)
    y0 = max(0, y - pad)
    x1 = min(combined_mask.shape[1], x + w + pad)
    y1 = min(combined_mask.shape[0], y + h + pad)

    contour_local = contour.copy()
    contour_local[:, 0, 0] -= x0
    contour_local[:, 0, 1] -= y0

    component_mask = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
    cv2.drawContours(component_mask, [contour_local], -1, 255, thickness=-1)
    component_mask = cv2.bitwise_and(component_mask, combined_mask[y0:y1, x0:x1])
    strict_crop = cv2.bitwise_and(strict_mask[y0:y1, x0:x1], component_mask)
    if np.count_nonzero(strict_crop) < max(30, int(min_area * 0.12)):
        return [contour]

    seed_mask = cv2.morphologyEx(
        strict_crop,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        iterations=1,
    )
    seed_mask = cv2.erode(
        seed_mask,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        iterations=1,
    )
    if np.count_nonzero(seed_mask) < 25:
        seed_mask = strict_crop

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(seed_mask)
    seeds: list[np.ndarray] = []
    min_seed_area = max(20, int(min_area * 0.08))
    for label in range(1, num_labels):
        if int(stats[label, cv2.CC_STAT_AREA]) < min_seed_area:
            continue
        seed = np.zeros_like(seed_mask)
        seed[labels == label] = 255
        seeds.append(seed)

    if len(seeds) < 2:
        return [contour]

    seeds = sorted(seeds, key=lambda seed: int(np.count_nonzero(seed)), reverse=True)[:4]
    component_pixels = component_mask > 0
    dist_stack: list[np.ndarray] = []
    for seed in seeds:
        distance_input = np.full_like(seed, 255)
        distance_input[seed > 0] = 0
        dist_stack.append(cv2.distanceTransform(distance_input, cv2.DIST_L2, 5))

    distances = np.stack(dist_stack, axis=0)
    nearest = np.argmin(distances, axis=0) + 1
    nearest[~component_pixels] = 0

    split_contours: list[np.ndarray] = []
    min_split_area = max(int(min_area * 0.55), 120)
    for label in range(1, len(seeds) + 1):
        submask = np.zeros_like(component_mask)
        submask[nearest == label] = 255
        submask = cv2.morphologyEx(
            submask,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
            iterations=1,
        )
        sub_contour = _largest_contour(submask)
        if sub_contour is None or cv2.contourArea(sub_contour) < min_split_area:
            continue
        sub_contour[:, 0, 0] += x0
        sub_contour[:, 0, 1] += y0
        split_contours.append(sub_contour)

    return split_contours if len(split_contours) >= 2 else [contour]


def _order_quad(pts: np.ndarray) -> np.ndarray:
    rect = np.zeros((4, 2), dtype=np.float64)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).flatten()
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    rect[1] = pts[np.argmin(d)]
    rect[3] = pts[np.argmax(d)]
    return rect


def _extract_top_face_quad(contour: np.ndarray) -> np.ndarray | None:
    hull = cv2.convexHull(contour)
    peri = cv2.arcLength(hull, True)
    if peri < 1e-6:
        return None

    approx = cv2.approxPolyDP(hull, 0.02 * peri, True).reshape(-1, 2)
    if len(approx) != 4:
        return None

    rect = cv2.minAreaRect(hull)
    w, h = rect[1]
    if w < 1e-6 or h < 1e-6:
        return None

    area = cv2.contourArea(hull)
    aspect = max(w, h) / max(min(w, h), 1e-6)
    fill = area / max(w * h, 1e-6)
    if aspect > 1.8 or fill < 0.55:
        return None

    return _order_quad(approx.astype(np.float64))


def _extract_oriented_box(contour: np.ndarray) -> tuple[np.ndarray, tuple]:
    hull = cv2.convexHull(contour)
    rect = cv2.minAreaRect(hull)
    return cv2.boxPoints(rect).astype(np.float64), rect


def _solve_pnp(
    quad: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
) -> tuple[np.ndarray | None, np.ndarray | None, float]:
    flag = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_IPPE)
    ok, rvec, tvec = cv2.solvePnP(_TOP_FACE_OBJ, quad, K, D, flags=flag)
    if not ok:
        return None, None, float("inf")
    projected, _ = cv2.projectPoints(_TOP_FACE_OBJ, rvec, tvec, K, D)
    err = float(np.mean(np.linalg.norm(projected.reshape(-1, 2) - quad, axis=1)))
    return rvec, tvec, err


def _intersect_ray_with_horizontal_plane(
    intrinsics: CameraIntrinsics,
    transform: TransformSnapshot,
    pixel_u: float,
    pixel_v: float,
    plane_z: float,
) -> np.ndarray | None:
    plane = Plane(
        frame_id=transform.reference_frame,
        normal_xyz=np.array([0.0, 0.0, 1.0], dtype=np.float64),
        point_xyz=np.array([0.0, 0.0, plane_z], dtype=np.float64),
    )
    origin, direction = ray_in_reference_frame(intrinsics, transform, pixel_u, pixel_v)
    return intersect_ray_with_plane(origin, direction, plane)


def _estimate_yaw_from_box(
    box: np.ndarray,
    intrinsics: CameraIntrinsics,
    transform: TransformSnapshot,
    center_plane_z: float,
) -> float:
    best_yaw = 0.0
    best_len = 0.0

    for idx in range(4):
        p0 = box[idx]
        p1 = box[(idx + 1) % 4]
        if np.linalg.norm(p1 - p0) < 3.0:
            continue

        world0 = _intersect_ray_with_horizontal_plane(
            intrinsics, transform, float(p0[0]), float(p0[1]), center_plane_z
        )
        world1 = _intersect_ray_with_horizontal_plane(
            intrinsics, transform, float(p1[0]), float(p1[1]), center_plane_z
        )
        if world0 is None or world1 is None:
            continue

        delta = world1[:2] - world0[:2]
        length = float(np.linalg.norm(delta))
        if length <= best_len:
            continue

        best_len = length
        best_yaw = float(math.atan2(delta[1], delta[0]))

    return best_yaw


def _estimate_box_world_edge_lengths(
    box: np.ndarray,
    intrinsics: CameraIntrinsics,
    transform: TransformSnapshot,
    plane_z: float,
) -> list[float]:
    world_pts: list[np.ndarray] = []
    for pt in box:
        hit = _intersect_ray_with_horizontal_plane(
            intrinsics, transform, float(pt[0]), float(pt[1]), plane_z
        )
        if hit is None:
            return []
        world_pts.append(hit)

    lengths: list[float] = []
    for idx in range(4):
        delta = world_pts[(idx + 1) % 4][:2] - world_pts[idx][:2]
        lengths.append(float(np.linalg.norm(delta)))
    return lengths


def _contour_shape_metrics(contour: np.ndarray) -> tuple[int, float]:
    peri = cv2.arcLength(contour, True)
    if peri < 1e-6:
        return 0, 0.0

    approx = cv2.approxPolyDP(contour, 0.03 * peri, True)
    area = float(cv2.contourArea(contour))
    circularity = float((4.0 * math.pi * area) / max(peri * peri, 1e-6))
    return len(approx), circularity


def _contour_color_purity(
    contour: np.ndarray,
    strict_mask: np.ndarray,
) -> float:
    contour_mask = np.zeros_like(strict_mask)
    cv2.drawContours(contour_mask, [contour], -1, 255, thickness=-1)
    contour_px = float(np.count_nonzero(contour_mask))
    if contour_px < 1.0:
        return 0.0
    overlap = cv2.bitwise_and(strict_mask, contour_mask)
    return float(np.count_nonzero(overlap) / contour_px)


def _rect_metrics(contour: np.ndarray, rect: tuple) -> tuple[float, float, float]:
    hull = cv2.convexHull(contour)
    area = float(cv2.contourArea(contour))
    hull_area = float(cv2.contourArea(hull))
    rect_w, rect_h = rect[1]
    rect_area = float(max(rect_w * rect_h, 1.0))
    solidity = area / max(hull_area, 1.0)
    fill = area / rect_area
    return area, solidity, fill


def _passes_box_geometry(
    contour: np.ndarray,
    rect: tuple,
    box: np.ndarray,
    intrinsics: CameraIntrinsics,
    transform: TransformSnapshot | None,
    center_plane_z: float,
    verifier: dict[str, float],
    color_purity: float,
    color_margin: float,
    side_metrics: dict[str, float],
) -> tuple[bool, dict[str, float], str]:
    rect_w, rect_h = rect[1]
    long_px = float(max(rect_w, rect_h))
    short_px = float(max(min(rect_w, rect_h), 1e-6))
    aspect = long_px / short_px

    vertices, circularity = _contour_shape_metrics(contour)
    diagnostics = {
        "aspect": aspect,
        "vertices": float(vertices),
        "circularity": circularity,
        "color_purity": color_purity,
        "color_margin": color_margin,
        "support_ratio": float(side_metrics["support_ratio"]),
        "side_area_ratio": float(side_metrics["side_area_ratio"]),
        "side_value_drop": float(side_metrics["side_value_drop"]),
        "world_long": -1.0,
        "world_short": -1.0,
    }

    if aspect > verifier["max_aspect_ratio"]:
        return False, diagnostics, "reject:aspect"
    if vertices < verifier["min_vertices"] or vertices > verifier["max_vertices"]:
        return False, diagnostics, "reject:verts"
    if circularity > verifier["max_circularity"]:
        return False, diagnostics, "reject:circ"
    if color_purity < verifier["min_color_purity"]:
        return False, diagnostics, "reject:purity"
    if color_margin < verifier["min_color_margin"]:
        return False, diagnostics, "reject:color_margin"
    if side_metrics["side_area_ratio"] < verifier["min_side_area_ratio"]:
        return False, diagnostics, "reject:flat"
    if side_metrics["side_value_drop"] < verifier["min_side_value_drop"]:
        return False, diagnostics, "reject:flat"

    if transform is None:
        return True, diagnostics, ""

    edge_lengths = _estimate_box_world_edge_lengths(box, intrinsics, transform, center_plane_z)
    if not edge_lengths:
        return False, diagnostics, "reject:world_proj"

    world_long = float(max(edge_lengths))
    world_short = float(min(edge_lengths))
    diagnostics["world_long"] = world_long
    diagnostics["world_short"] = world_short

    if world_short < verifier["min_world_edge_m"]:
        return False, diagnostics, "reject:world_small"
    if world_long > verifier["max_world_edge_m"]:
        return False, diagnostics, "reject:world_large"

    return True, diagnostics, ""


class HSVBlockDetector:
    def __init__(self, color_ranges: dict, min_area: int, max_area: int) -> None:
        self._ranges = color_ranges
        self._min_area = min_area
        self._max_area = max_area

    def set_color_ranges(self, color_ranges: dict) -> None:
        self._ranges = color_ranges

    def detect(
        self,
        bgr: np.ndarray,
        intrinsics: CameraIntrinsics,
        transform: TransformSnapshot | None,
        plane: Plane | None,
        block_half_height: float,
        max_reproj_err: float,
        roi_norm: tuple[float, float, float, float] | None,
        verifier: dict[str, float],
        debug_mask_color: str,
        mask_policy: dict[str, int],
    ) -> tuple[list[dict], np.ndarray, np.ndarray | None]:
        enhanced, hsv, lab = _prepare_image(bgr)
        roi_rect = _normalized_roi_to_pixels(enhanced.shape, roi_norm)
        debug = _draw_roi_overlay(enhanced.copy(), roi_rect)
        results: list[dict] = []
        mask_debug: np.ndarray | None = None
        strict_masks: dict[str, np.ndarray] = {}
        dominance_masks: dict[str, np.ndarray] = {}
        combined_masks: dict[str, np.ndarray] = {}

        K = intrinsics.camera_matrix()
        D = intrinsics.dist_array()
        R_c2w = (
            quaternion_to_rotation_matrix(transform.rotation_xyzw)
            if transform is not None
            else None
        )
        center_plane_z = (
            float(plane.point_xyz[2]) + block_half_height if plane is not None else block_half_height
        )

        for color, ranges in self._ranges.items():
            strict_raw = _build_hsv_mask(hsv, ranges)
            dominance_raw = _build_dominance_mask(enhanced, hsv, lab, color)
            mask = _build_mask(
                strict_raw,
                dominance_raw,
                expand_kernel_size=mask_policy["dominance_expand_kernel_size"],
                expand_iterations=mask_policy["dominance_expand_iterations"],
                min_strict_pixels=mask_policy["min_strict_pixels_for_expansion"],
            )
            strict_mask = _apply_roi_to_mask(_stabilize_mask(strict_raw), roi_rect)
            dominance_mask = _apply_roi_to_mask(_stabilize_mask(dominance_raw), roi_rect)
            mask = _apply_roi_to_mask(mask, roi_rect)
            strict_masks[color] = strict_mask
            dominance_masks[color] = dominance_mask
            combined_masks[color] = mask
            if color == debug_mask_color:
                mask_debug = _make_mask_debug_image(enhanced, strict_mask, dominance_mask, mask, color)

        for color in self._ranges:
            strict_mask = strict_masks[color]
            mask = combined_masks[color]
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            candidate_contours: list[np.ndarray] = []

            for cnt in contours:
                box, rect = _extract_oriented_box(cnt)
                if _component_needs_split(
                    cnt,
                    rect,
                    self._min_area,
                    self._max_area,
                    verifier["max_aspect_ratio"],
                ):
                    candidate_contours.extend(
                        _split_component_contour(mask, strict_mask, cnt, self._min_area)
                    )
                else:
                    candidate_contours.append(cnt)

            for cnt in candidate_contours:
                box, rect = _extract_oriented_box(cnt)
                area, solidity, fill = _rect_metrics(cnt, rect)
                if area < self._min_area or area > self._max_area:
                    _draw_reject(debug, cnt, "reject:area")
                    continue
                if solidity < 0.72 or fill < 0.45:
                    _draw_reject(debug, cnt, "reject:shape")
                    continue

                px = float(rect[0][0])
                py = float(rect[0][1])
                if not _point_in_roi(px, py, roi_rect):
                    _draw_reject(debug, cnt, "reject:roi")
                    continue

                core_contour, core_mask, core_area = _extract_candidate_core(cnt, strict_mask, hsv)
                if core_contour is None or core_area < 20.0:
                    _draw_reject(debug, cnt, "reject:core")
                    continue
                classified_color, color_purity, color_margin, _ = _classify_core_color(
                    core_mask,
                    strict_masks,
                    dominance_masks,
                )
                if classified_color != color:
                    _draw_reject(debug, cnt, "reject:class")
                    continue
                contour_mask = _contour_to_mask(strict_mask.shape, cnt)
                side_metrics = _compute_side_support_metrics(contour_mask, core_mask, hsv)

                geometry_ok, geometry_diag, reject_reason = _passes_box_geometry(
                    cnt,
                    rect,
                    box,
                    intrinsics,
                    transform,
                    center_plane_z,
                    verifier,
                    color_purity,
                    color_margin,
                    side_metrics,
                )
                if not geometry_ok:
                    _draw_reject(debug, cnt, reject_reason)
                    continue

                center_w: np.ndarray | None = None
                yaw = 0.0
                conf = float(
                    np.clip(
                        0.15
                        + 0.30 * min(solidity, 1.0)
                        + 0.15 * min(fill, 1.0)
                        + 0.20 * min(color_purity, 1.0)
                        + 0.10 * min(max(color_margin, 0.0) * 4.0, 1.0)
                        + 0.10 * min(geometry_diag["side_area_ratio"], 1.0),
                        0.1,
                        0.90,
                    )
                )
                source = "rect_center"
                reproj_err = float("inf")

                top_face_quad = _extract_top_face_quad(core_contour)
                if top_face_quad is not None and transform is not None and R_c2w is not None:
                    rvec, tvec, reproj_err = _solve_pnp(top_face_quad, K, D)
                    if rvec is not None and reproj_err <= max_reproj_err:
                        R_obj, _ = cv2.Rodrigues(rvec)
                        center_cam = tvec.flatten() + R_obj @ np.array([0.0, 0.0, block_half_height])
                        center_w = R_c2w @ center_cam + transform.translation_m
                        R_obj_w = R_c2w @ R_obj
                        yaw = float(np.arctan2(R_obj_w[1, 0], R_obj_w[0, 0]))
                        conf = float(np.clip(1.0 - reproj_err / max_reproj_err, 0.2, 1.0))
                        source = "pnp_top_face"

                if center_w is None and transform is not None and plane is not None:
                    center_hit = _intersect_ray_with_horizontal_plane(
                        intrinsics, transform, px, py, center_plane_z
                    )
                    if center_hit is None:
                        continue
                    center_w = center_hit
                    yaw = _estimate_yaw_from_box(box, intrinsics, transform, center_plane_z)

                if center_w is None:
                    continue

                candidate = {
                    "color": classified_color,
                    "x": float(center_w[0]),
                    "y": float(center_w[1]),
                    "z": float(center_w[2]),
                    "yaw": yaw,
                    "confidence": conf,
                    "pixel_u": px,
                    "pixel_v": py,
                    "source": source,
                    "reproj_err": round(reproj_err, 2) if np.isfinite(reproj_err) else -1.0,
                    "solidity": round(solidity, 3),
                    "fill_ratio": round(fill, 3),
                    "aspect_ratio": round(float(geometry_diag["aspect"]), 3),
                    "color_purity": round(float(geometry_diag["color_purity"]), 3),
                    "color_margin": round(float(geometry_diag["color_margin"]), 3),
                    "side_area_ratio": round(float(geometry_diag["side_area_ratio"]), 3),
                    "side_value_drop": round(float(geometry_diag["side_value_drop"]), 2),
                    "world_long_edge_m": round(float(geometry_diag["world_long"]), 4),
                    "world_short_edge_m": round(float(geometry_diag["world_short"]), 4),
                }
                results.append(candidate)

                c = _DEBUG_BGR.get(classified_color, (200, 200, 200))
                hull = cv2.convexHull(cnt)
                cv2.drawContours(debug, [hull], -1, c, 2)
                cv2.polylines(debug, [box.astype(int)], True, (255, 255, 255), 1)
                cv2.circle(debug, (int(px), int(py)), 4, c, -1)
                if top_face_quad is not None and source == "pnp_top_face":
                    for corner in top_face_quad.astype(int):
                        cv2.circle(debug, tuple(corner), 4, c, -1)

                label = f"{classified_color}|{source} {conf:.2f}"
                if np.isfinite(reproj_err):
                    label += f" e={reproj_err:.1f}"
                label += f" s={solidity:.2f}"
                label += f" p={geometry_diag['color_purity']:.2f}"
                label += f" d={geometry_diag['side_value_drop']:.0f}"
                cv2.putText(
                    debug,
                    label,
                    (int(box[:, 0].min()), max(int(box[:, 1].min()) - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    c,
                    1,
                )

        return results, debug, mask_debug


class DetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("detector_node")

        self.declare_parameter("reference_frame", "world")
        self.declare_parameter("camera_frame", "default_cam")
        self.declare_parameter("table_z_m", 0.0)
        self.declare_parameter("block_half_height_m", 0.015)
        self.declare_parameter("min_contour_area", 500)
        self.declare_parameter("max_contour_area", 30000)
        self.declare_parameter("max_reproj_error_px", 5.0)
        self.declare_parameter("color_ranges_path", "")
        self.declare_parameter("roi_x_min", 0.05)
        self.declare_parameter("roi_y_min", 0.05)
        self.declare_parameter("roi_x_max", 0.95)
        self.declare_parameter("roi_y_max", 0.95)
        self.declare_parameter("max_box_aspect_ratio", 1.9)
        self.declare_parameter("min_polygon_vertices", 4)
        self.declare_parameter("max_polygon_vertices", 8)
        self.declare_parameter("max_circularity", 0.94)
        self.declare_parameter("min_color_purity", 0.32)
        self.declare_parameter("min_color_margin", 0.08)
        self.declare_parameter("min_side_area_ratio", 0.18)
        self.declare_parameter("min_side_value_drop", 8.0)
        self.declare_parameter("min_world_edge_m", 0.018)
        self.declare_parameter("max_world_edge_m", 0.12)
        self.declare_parameter("debug_mask_color", "green")
        self.declare_parameter("dominance_expand_kernel_size", 35)
        self.declare_parameter("dominance_expand_iterations", 3)
        self.declare_parameter("min_strict_pixels_for_expansion", 25)

        ranges_path = self.get_parameter("color_ranges_path").value
        color_ranges = _load_color_ranges(ranges_path)
        self._declare_hsv_parameters(color_ranges)
        self.get_logger().info(f"Loaded color ranges: {list(color_ranges.keys())} from '{ranges_path}'")

        self._detector = HSVBlockDetector(
            color_ranges=color_ranges,
            min_area=int(self.get_parameter("min_contour_area").value),
            max_area=int(self.get_parameter("max_contour_area").value),
        )

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._intrinsics: CameraIntrinsics | None = None

        self.create_subscription(CameraInfo, "/camera_info", self._on_camera_info, 10)
        self.create_subscription(Image, "/image/raw", self._on_image, 10)
        self._blocks_pub = self.create_publisher(String, "/omx/perception/blocks", 10)
        self._debug_pub = self.create_publisher(Image, "/omx/perception/debug_image", 10)
        self._debug_mask_pub = self.create_publisher(Image, "/omx/perception/debug_mask", 10)

    def _declare_hsv_parameters(self, color_ranges: dict) -> None:
        green = _first_range_or_default(color_ranges, "green", 0)
        blue = _first_range_or_default(color_ranges, "blue", 0)
        red_1 = _first_range_or_default(color_ranges, "red", 0)
        red_2 = _first_range_or_default(color_ranges, "red", 1)

        for color, entry in (("green", green), ("blue", blue)):
            self.declare_parameter(f"{color}_h_min", int(entry["h"][0]))
            self.declare_parameter(f"{color}_h_max", int(entry["h"][1]))
            self.declare_parameter(f"{color}_s_min", int(entry["s"][0]))
            self.declare_parameter(f"{color}_s_max", int(entry["s"][1]))
            self.declare_parameter(f"{color}_v_min", int(entry["v"][0]))
            self.declare_parameter(f"{color}_v_max", int(entry["v"][1]))

        for idx, entry in ((1, red_1), (2, red_2)):
            self.declare_parameter(f"red_h{idx}_min", int(entry["h"][0]))
            self.declare_parameter(f"red_h{idx}_max", int(entry["h"][1]))
            self.declare_parameter(f"red_s{idx}_min", int(entry["s"][0]))
            self.declare_parameter(f"red_s{idx}_max", int(entry["s"][1]))
            self.declare_parameter(f"red_v{idx}_min", int(entry["v"][0]))
            self.declare_parameter(f"red_v{idx}_max", int(entry["v"][1]))

    def _current_color_ranges(self) -> dict:
        def clip_h(value: float) -> int:
            return int(np.clip(int(value), 0, 179))

        def clip_sv(value: float) -> int:
            return int(np.clip(int(value), 0, 255))

        color_ranges = {
            "green": [
                {
                    "h": [
                        clip_h(self.get_parameter("green_h_min").value),
                        clip_h(self.get_parameter("green_h_max").value),
                    ],
                    "s": [
                        clip_sv(self.get_parameter("green_s_min").value),
                        clip_sv(self.get_parameter("green_s_max").value),
                    ],
                    "v": [
                        clip_sv(self.get_parameter("green_v_min").value),
                        clip_sv(self.get_parameter("green_v_max").value),
                    ],
                }
            ],
            "blue": [
                {
                    "h": [
                        clip_h(self.get_parameter("blue_h_min").value),
                        clip_h(self.get_parameter("blue_h_max").value),
                    ],
                    "s": [
                        clip_sv(self.get_parameter("blue_s_min").value),
                        clip_sv(self.get_parameter("blue_s_max").value),
                    ],
                    "v": [
                        clip_sv(self.get_parameter("blue_v_min").value),
                        clip_sv(self.get_parameter("blue_v_max").value),
                    ],
                }
            ],
            "red": [],
        }

        for idx in (1, 2):
            color_ranges["red"].append(
                {
                    "h": [
                        clip_h(self.get_parameter(f"red_h{idx}_min").value),
                        clip_h(self.get_parameter(f"red_h{idx}_max").value),
                    ],
                    "s": [
                        clip_sv(self.get_parameter(f"red_s{idx}_min").value),
                        clip_sv(self.get_parameter(f"red_s{idx}_max").value),
                    ],
                    "v": [
                        clip_sv(self.get_parameter(f"red_v{idx}_min").value),
                        clip_sv(self.get_parameter(f"red_v{idx}_max").value),
                    ],
                }
            )

        return color_ranges

    def _on_camera_info(self, msg: CameraInfo) -> None:
        k = msg.k
        if k[0] == 0.0:
            return
        dist = tuple(float(v) for v in msg.d) if msg.d else (0.0, 0.0, 0.0, 0.0, 0.0)
        self._intrinsics = CameraIntrinsics(
            fx=float(k[0]),
            fy=float(k[4]),
            cx=float(k[2]),
            cy=float(k[5]),
            frame_id=msg.header.frame_id,
            dist_coeffs=dist,
        )

    def _on_image(self, msg: Image) -> None:
        if self._intrinsics is None:
            return

        ref_frame = self.get_parameter("reference_frame").value
        cam_frame = self.get_parameter("camera_frame").value
        table_z = float(self.get_parameter("table_z_m").value)
        half_h = float(self.get_parameter("block_half_height_m").value)
        max_err = float(self.get_parameter("max_reproj_error_px").value)
        roi_norm = (
            float(self.get_parameter("roi_x_min").value),
            float(self.get_parameter("roi_y_min").value),
            float(self.get_parameter("roi_x_max").value),
            float(self.get_parameter("roi_y_max").value),
        )
        verifier = {
            "max_aspect_ratio": float(self.get_parameter("max_box_aspect_ratio").value),
            "min_vertices": int(self.get_parameter("min_polygon_vertices").value),
            "max_vertices": int(self.get_parameter("max_polygon_vertices").value),
            "max_circularity": float(self.get_parameter("max_circularity").value),
            "min_color_purity": float(self.get_parameter("min_color_purity").value),
            "min_color_margin": float(self.get_parameter("min_color_margin").value),
            "min_side_area_ratio": float(self.get_parameter("min_side_area_ratio").value),
            "min_side_value_drop": float(self.get_parameter("min_side_value_drop").value),
            "min_world_edge_m": float(self.get_parameter("min_world_edge_m").value),
            "max_world_edge_m": float(self.get_parameter("max_world_edge_m").value),
        }
        debug_mask_color = str(self.get_parameter("debug_mask_color").value).strip().lower()
        mask_policy = {
            "dominance_expand_kernel_size": int(self.get_parameter("dominance_expand_kernel_size").value),
            "dominance_expand_iterations": int(self.get_parameter("dominance_expand_iterations").value),
            "min_strict_pixels_for_expansion": int(self.get_parameter("min_strict_pixels_for_expansion").value),
        }
        self._detector.set_color_ranges(self._current_color_ranges())

        transform: TransformSnapshot | None = None
        try:
            tf = self._tf_buffer.lookup_transform(ref_frame, cam_frame, rclpy.time.Time())
            t = tf.transform.translation
            r = tf.transform.rotation
            transform = TransformSnapshot(
                reference_frame=ref_frame,
                translation_m=np.array([t.x, t.y, t.z]),
                rotation_xyzw=np.array([r.x, r.y, r.z, r.w]),
            )
        except Exception:
            pass

        plane = Plane(
            frame_id=ref_frame,
            normal_xyz=np.array([0.0, 0.0, 1.0]),
            point_xyz=np.array([0.0, 0.0, table_z]),
        )

        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        blocks, debug, mask_debug = self._detector.detect(
            bgr,
            self._intrinsics,
            transform,
            plane,
            half_h,
            max_err,
            roi_norm,
            verifier,
            debug_mask_color,
            mask_policy,
        )

        stamp = msg.header.stamp
        self._blocks_pub.publish(
            String(
                data=json.dumps(
                    {
                        "stamp_sec": stamp.sec,
                        "stamp_nanosec": stamp.nanosec,
                        "frame_id": ref_frame,
                        "blocks": blocks,
                    }
                )
            )
        )
        debug_msg = self._bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_msg.header = msg.header
        self._debug_pub.publish(debug_msg)
        if mask_debug is not None:
            mask_msg = self._bridge.cv2_to_imgmsg(mask_debug, encoding="bgr8")
            mask_msg.header = msg.header
            self._debug_mask_pub.publish(mask_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
