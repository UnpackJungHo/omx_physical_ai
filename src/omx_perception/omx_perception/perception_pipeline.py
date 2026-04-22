"""Pure functions for OMX's current color-block perception fast path.

Pipeline per frame:
    1. Undistort
    2. Optional gray-world white balance
    3. Saturation-based foreground segmentation
    4. Optional large-blob split for calibration workflows
    5. minAreaRect + ray/plane pose estimate
    6. a*b* majority-vote color labeling
    7. Confidence scoring and debug overlay

ROS-free. All functions take explicit dependencies; no global state.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from omx_perception.camera_geometry import (
    CameraIntrinsics,
    Plane,
    TransformSnapshot,
    intersect_ray_with_plane,
    ray_in_reference_frame,
)


@dataclass(frozen=True)
class WorkspaceRect:
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    plane_z_m: float


@dataclass(frozen=True)
class DetectorSettings:
    block_size_m: float = 0.030
    color_chroma_min: float = 15.0
    color_majority_min: float = 0.60
    canny_sigma: float = 0.33
    min_contour_area_px: float = 200.0
    max_contour_area_px: float = 20000.0
    saturation_min: int = 40
    rect_fill_min: float = 0.75
    aspect_ratio_min: float = 0.60
    aspect_ratio_max: float = 1.66


@dataclass(frozen=True)
class ColorPrototype:
    name: str
    a_star: float
    b_star: float


def gray_world_white_balance(bgr: np.ndarray) -> np.ndarray:
    """Channel-wise gain so per-channel mean matches the overall mean."""
    bgr_f = bgr.astype(np.float32) + 1.0
    means = bgr_f.reshape(-1, 3).mean(axis=0)
    scale = np.clip(means.mean() / means, 0.75, 1.35)
    balanced = np.clip(bgr_f * scale.reshape(1, 1, 3), 0.0, 255.0)
    return balanced.astype(np.uint8)


def build_undistort_maps(
    intrinsics: CameraIntrinsics,
    image_size_wh: tuple[int, int],
) -> tuple[np.ndarray, np.ndarray]:
    return cv2.initUndistortRectifyMap(
        intrinsics.camera_matrix(),
        intrinsics.dist_array(),
        None,
        intrinsics.camera_matrix(),
        image_size_wh,
        cv2.CV_16SC2,
    )


def auto_canny(gray: np.ndarray, sigma: float = 0.33) -> np.ndarray:
    median = float(np.median(gray))
    lower = int(max(0, (1.0 - sigma) * median))
    upper = int(min(255, (1.0 + sigma) * median))
    return cv2.Canny(gray, lower, upper)


def segment_foreground(
    bgr: np.ndarray,
    canny_sigma: float = 0.33,
    saturation_min: int = 40,
    compute_edges: bool = False,
) -> tuple[list[np.ndarray], np.ndarray | None, np.ndarray]:
    """Return (external contours, optional canny edge map, saturation mask)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    _, sat_mask = cv2.threshold(saturation, saturation_min, 255, cv2.THRESH_BINARY)
    close_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    sat_mask = cv2.morphologyEx(sat_mask, cv2.MORPH_CLOSE, close_k, iterations=2)
    open_k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    sat_mask = cv2.morphologyEx(sat_mask, cv2.MORPH_OPEN, open_k, iterations=1)
    contours, _ = cv2.findContours(sat_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    edges = None
    if compute_edges:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        edges = auto_canny(cv2.GaussianBlur(gray, (3, 3), 0), canny_sigma)
    return list(contours), edges, sat_mask


@dataclass(frozen=True)
class BlockEstimate:
    """Ray-plane + minAreaRect based 4-DOF block pose."""

    x_world: float
    y_world: float
    z_world: float
    yaw_world: float
    pixel_u: float
    pixel_v: float
    rect_fill: float
    aspect_ratio: float
    rect_corners_px: np.ndarray
    center_px: tuple[int, int]


def estimate_block_pose(
    contour: np.ndarray,
    intrinsics: CameraIntrinsics,
    transform: TransformSnapshot,
    block_size_m: float,
    table_z_m: float,
    rect_fill_min: float,
    aspect_ratio_min: float,
    aspect_ratio_max: float,
) -> tuple[BlockEstimate | None, str]:
    """Estimate (x, y, z, yaw) from contour by ray-plane + minAreaRect."""
    area = float(cv2.contourArea(contour))
    if area < 1.0:
        return None, "degenerate"

    rect = cv2.minAreaRect(contour)
    (u, v), (rw, rh), angle_deg = rect
    if rw < 1.0 or rh < 1.0:
        return None, "rect_degenerate"

    long_side = max(rw, rh)
    short_side = min(rw, rh)
    aspect = long_side / short_side
    if aspect < aspect_ratio_min or aspect > aspect_ratio_max:
        return None, "aspect_bad"

    rect_area = rw * rh
    fill = area / rect_area if rect_area > 0 else 0.0
    if fill < rect_fill_min:
        return None, "fill_low"

    half = block_size_m / 2.0
    center_plane = Plane(
        frame_id=transform.reference_frame,
        normal_xyz=np.array([0.0, 0.0, 1.0], dtype=np.float64),
        point_xyz=np.array([0.0, 0.0, table_z_m + half], dtype=np.float64),
    )
    origin, direction = ray_in_reference_frame(intrinsics, transform, float(u), float(v))
    hit = intersect_ray_with_plane(origin, direction, center_plane)
    if hit is None:
        return None, "ray_miss"

    box_pts = cv2.boxPoints(rect).astype(np.float32)
    angle_rad = math.radians(float(angle_deg))
    if rw < rh:
        angle_rad += math.pi / 2.0
    yaw = _image_angle_to_world_yaw(
        intrinsics,
        transform,
        (float(u), float(v)),
        angle_rad,
    )

    return (
        BlockEstimate(
            x_world=float(hit[0]),
            y_world=float(hit[1]),
            z_world=float(hit[2]),
            yaw_world=yaw,
            pixel_u=float(u),
            pixel_v=float(v),
            rect_fill=float(fill),
            aspect_ratio=float(aspect),
            rect_corners_px=box_pts,
            center_px=(int(round(u)), int(round(v))),
        ),
        "ok",
    )


def _image_angle_to_world_yaw(
    intrinsics: CameraIntrinsics,
    transform: TransformSnapshot,
    center_uv: tuple[float, float],
    image_angle_rad: float,
) -> float:
    """Map an image-plane orientation at (u, v) to a world yaw around +Z."""
    u, v = center_uv
    du = math.cos(image_angle_rad) * 20.0
    dv = math.sin(image_angle_rad) * 20.0
    origin_a, dir_a = ray_in_reference_frame(intrinsics, transform, u, v)
    origin_b, dir_b = ray_in_reference_frame(intrinsics, transform, u + du, v + dv)
    plane = Plane(
        frame_id=transform.reference_frame,
        normal_xyz=np.array([0.0, 0.0, 1.0], dtype=np.float64),
        point_xyz=np.array([0.0, 0.0, 0.0], dtype=np.float64),
    )
    pt_a = intersect_ray_with_plane(origin_a, dir_a, plane)
    pt_b = intersect_ray_with_plane(origin_b, dir_b, plane)
    if pt_a is None or pt_b is None:
        return 0.0
    dx = float(pt_b[0] - pt_a[0])
    dy = float(pt_b[1] - pt_a[1])
    yaw = math.atan2(dy, dx)
    while yaw > math.pi / 4.0:
        yaw -= math.pi / 2.0
    while yaw < -math.pi / 4.0:
        yaw += math.pi / 2.0
    return yaw


def contour_center_in_workspace(
    contour: np.ndarray,
    intrinsics: CameraIntrinsics,
    transform: TransformSnapshot,
    workspace: WorkspaceRect,
) -> tuple[bool, np.ndarray | None]:
    """Check if the contour center projects inside the configured workspace."""
    rect = cv2.minAreaRect(contour)
    u, v = float(rect[0][0]), float(rect[0][1])
    plane = Plane(
        frame_id=transform.reference_frame,
        normal_xyz=np.array([0.0, 0.0, 1.0], dtype=np.float64),
        point_xyz=np.array([0.0, 0.0, workspace.plane_z_m], dtype=np.float64),
    )
    origin, direction = ray_in_reference_frame(intrinsics, transform, u, v)
    hit = intersect_ray_with_plane(origin, direction, plane)
    if hit is None:
        return False, None
    if not (workspace.x_min_m <= hit[0] <= workspace.x_max_m):
        return False, hit
    if not (workspace.y_min_m <= hit[1] <= workspace.y_max_m):
        return False, hit
    return True, hit


def split_large_blob(
    contour: np.ndarray,
    max_single_area_px: float,
    min_area_px: float,
) -> list[np.ndarray]:
    """Split contours much larger than one cube for calibration workflows."""
    area = float(cv2.contourArea(contour))
    if area < max_single_area_px * 1.30:
        return [contour]

    x, y, w, h = cv2.boundingRect(contour)
    pad = 4
    x0 = max(0, x - pad)
    y0 = max(0, y - pad)
    bw = w + 2 * pad
    bh = h + 2 * pad
    mask = np.zeros((bh, bw), dtype=np.uint8)
    cnt_local = contour.copy()
    cnt_local[:, 0, 0] -= x0
    cnt_local[:, 0, 1] -= y0
    cv2.drawContours(mask, [cnt_local], -1, 255, thickness=-1)

    eroded = cv2.erode(
        mask,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
        iterations=2,
    )
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(eroded)
    seeds: list[np.ndarray] = []
    for label in range(1, num_labels):
        if int(stats[label, cv2.CC_STAT_AREA]) < max(20, int(min_area_px * 0.1)):
            continue
        seed = np.zeros_like(eroded)
        seed[labels == label] = 255
        seeds.append(seed)

    if len(seeds) < 2:
        return [contour]

    dist_stack: list[np.ndarray] = []
    for seed in seeds:
        dist_input = np.full_like(seed, 255)
        dist_input[seed > 0] = 0
        dist_stack.append(cv2.distanceTransform(dist_input, cv2.DIST_L2, 5))
    distances = np.stack(dist_stack, axis=0)
    nearest = np.argmin(distances, axis=0) + 1
    nearest[mask == 0] = 0

    split_contours: list[np.ndarray] = []
    for label in range(1, len(seeds) + 1):
        sub = np.zeros_like(mask)
        sub[nearest == label] = 255
        sub = cv2.morphologyEx(
            sub,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
            iterations=1,
        )
        sub_contours, _ = cv2.findContours(sub, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not sub_contours:
            continue
        largest = max(sub_contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < min_area_px:
            continue
        largest[:, 0, 0] += x0
        largest[:, 0, 1] += y0
        split_contours.append(largest)

    return split_contours if len(split_contours) >= 2 else [contour]


def _warp_rect_patch(
    bgr: np.ndarray,
    rect_corners_px: np.ndarray,
    shrink: float = 0.50,
) -> np.ndarray:
    """Warp a minAreaRect interior into a fixed 40x40 patch."""
    center = rect_corners_px.mean(axis=0, keepdims=True)
    pulled = center + (rect_corners_px - center) * shrink
    src = pulled.astype(np.float32)
    dst = np.array([[0, 0], [40, 0], [40, 40], [0, 40]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(bgr, homography, (40, 40), flags=cv2.INTER_LINEAR)


def _chromatic_pixels_from_patch(
    patch_bgr: np.ndarray,
    chroma_min: float,
) -> tuple[np.ndarray, np.ndarray] | None:
    lab = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2LAB)
    luminance = lab[:, :, 0].astype(np.float32)
    a_star = lab[:, :, 1].astype(np.float32) - 128.0
    b_star = lab[:, :, 2].astype(np.float32) - 128.0
    chroma = np.sqrt(a_star * a_star + b_star * b_star)
    achromatic = chroma < chroma_min
    l_low = np.percentile(luminance, 20.0)
    l_high = np.percentile(luminance, 80.0)
    extreme = (luminance < l_low) | (luminance > l_high)
    valid = ~(achromatic | extreme)
    if np.count_nonzero(valid) < 10:
        return None
    return a_star[valid], b_star[valid]


def extract_rect_patch_ab(
    bgr: np.ndarray,
    rect_corners_px: np.ndarray,
    chroma_min: float = 15.0,
) -> tuple[float, float, int] | None:
    """Return (median a*, median b*, sample_count) from a rect interior patch."""
    patch = _warp_rect_patch(bgr, rect_corners_px)
    filtered = _chromatic_pixels_from_patch(patch, chroma_min)
    if filtered is None:
        return None
    a_valid, b_valid = filtered
    return float(np.median(a_valid)), float(np.median(b_valid)), int(a_valid.size)


def label_color_rect(
    bgr: np.ndarray,
    rect_corners_px: np.ndarray,
    prototypes: list[ColorPrototype],
    chroma_min: float = 15.0,
    majority_min: float = 0.60,
) -> tuple[str | None, float]:
    """Classify block color by a*b* 1-NN majority vote over rect interior."""
    if not prototypes:
        return None, 0.0

    patch = _warp_rect_patch(bgr, rect_corners_px)
    filtered = _chromatic_pixels_from_patch(patch, chroma_min)
    if filtered is None:
        return None, 0.0
    a_valid, b_valid = filtered

    proto_a = np.array([p.a_star for p in prototypes], dtype=np.float32)
    proto_b = np.array([p.b_star for p in prototypes], dtype=np.float32)
    diff_a = a_valid[:, None] - proto_a[None, :]
    diff_b = b_valid[:, None] - proto_b[None, :]
    distances = np.sqrt(diff_a * diff_a + diff_b * diff_b)
    labels = np.argmin(distances, axis=1)
    counts = np.bincount(labels, minlength=len(prototypes)).astype(np.float64)
    total = float(counts.sum())
    if total < 1.0:
        return None, 0.0
    winner = int(np.argmax(counts))
    ratio = float(counts[winner] / total)
    if ratio < majority_min:
        return None, ratio
    return prototypes[winner].name, ratio


def compute_confidence(
    rect_fill: float,
    rect_fill_min: float,
    aspect_ratio: float,
    color_ratio: float,
    color_ratio_min: float,
) -> float:
    del rect_fill_min, color_ratio_min
    fill_score = float(np.clip(rect_fill, 0.0, 1.0))
    aspect_score = float(np.clip(1.0 - abs(aspect_ratio - 1.0) / 0.66, 0.0, 1.0))
    color_score = float(np.clip(color_ratio, 0.0, 1.0))
    return float(min(fill_score, aspect_score, color_score))


_DEBUG_BGR: dict[str, tuple[int, int, int]] = {
    "red": (0, 0, 255),
    "green": (0, 200, 0),
    "blue": (255, 80, 0),
}


def draw_debug(
    image: np.ndarray,
    contour: np.ndarray,
    rect_corners_px: np.ndarray | None,
    color_name: str | None,
    label_text: str,
    center_px: tuple[int, int] | None = None,
) -> None:
    draw_color = _DEBUG_BGR.get(color_name or "", (200, 200, 200))
    cv2.drawContours(image, [contour], -1, draw_color, 2)
    if rect_corners_px is not None:
        box = rect_corners_px.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(image, [box], True, draw_color, 2)
    if center_px is not None:
        cu, cv_ = center_px
        cv2.line(image, (cu - 14, cv_), (cu + 14, cv_), (0, 0, 0), 3)
        cv2.line(image, (cu, cv_ - 14), (cu, cv_ + 14), (0, 0, 0), 3)
        cv2.line(image, (cu - 14, cv_), (cu + 14, cv_), (255, 255, 255), 1)
        cv2.line(image, (cu, cv_ - 14), (cu, cv_ + 14), (255, 255, 255), 1)
        cv2.circle(image, center_px, 7, draw_color, -1)
        cv2.circle(image, center_px, 9, (255, 255, 255), 1)
        cv2.circle(image, center_px, 10, (0, 0, 0), 1)
    if label_text:
        x, y, _, _ = cv2.boundingRect(contour)
        cv2.putText(
            image,
            label_text,
            (x, max(y - 6, 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 0, 0),
            3,
        )
        cv2.putText(
            image,
            label_text,
            (x, max(y - 6, 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            draw_color,
            1,
        )


def draw_reject(
    image: np.ndarray,
    contour: np.ndarray,
    reason: str,
    info_text: str = "",
) -> None:
    """Draw a gray outline + reject reason label for a rejected contour."""
    gray = (120, 120, 120)
    x, y, w, h = cv2.boundingRect(contour)
    cv2.rectangle(image, (x, y), (x + w, y + h), gray, 1)
    cv2.drawContours(image, [contour], -1, gray, 1)
    line1 = f"X {reason}"
    cv2.putText(
        image,
        line1,
        (x, max(y - 18, 12)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (0, 0, 0),
        3,
    )
    cv2.putText(
        image,
        line1,
        (x, max(y - 18, 12)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        gray,
        1,
    )
    if info_text:
        cv2.putText(
            image,
            info_text,
            (x, max(y - 4, 24)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (0, 0, 0),
            3,
        )
        cv2.putText(
            image,
            info_text,
            (x, max(y - 4, 24)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            gray,
            1,
        )


def load_color_prototypes(data: dict | None) -> list[ColorPrototype]:
    if not isinstance(data, dict):
        return []
    prototypes: list[ColorPrototype] = []
    for name, entry in data.items():
        if not isinstance(entry, dict):
            continue
        if "a_star" not in entry or "b_star" not in entry:
            continue
        prototypes.append(
            ColorPrototype(
                name=str(name),
                a_star=float(entry["a_star"]),
                b_star=float(entry["b_star"]),
            )
        )
    return prototypes
