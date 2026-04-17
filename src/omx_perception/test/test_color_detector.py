from pathlib import Path
import sys

import cv2
import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = Path(__file__).resolve().parents[3]

if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from omx_perception.color_detector import detect_blocks, detect_blocks_and_masks


def _bgr_from_hsv(h: int, s: int, v: int) -> tuple[int, int, int]:
    hsv = np.uint8([[[h, s, v]]])
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
    return tuple(int(x) for x in bgr)


def _find_nearest(blocks, color: str, expected_cx: int, expected_cy: int):
    candidates = [block for block in blocks if block.color == color]
    assert candidates, f"No detections for {color}"
    return min(candidates, key=lambda block: abs(block.cx - expected_cx) + abs(block.cy - expected_cy))


def test_detect_blocks_on_synthetic_scene() -> None:
    img = np.full((480, 640, 3), 255, np.uint8)

    cv2.rectangle(img, (80, 120), (220, 260), _bgr_from_hsv(169, 80, 180), -1)
    cv2.rectangle(img, (260, 120), (400, 260), _bgr_from_hsv(83, 140, 180), -1)
    cv2.rectangle(img, (440, 120), (580, 260), _bgr_from_hsv(99, 160, 180), -1)

    blocks = detect_blocks(img)
    assert len(blocks) == 3

    red = _find_nearest(blocks, "red", 150, 190)
    green = _find_nearest(blocks, "green", 330, 190)
    blue = _find_nearest(blocks, "blue", 510, 190)

    assert abs(red.cx - 150) <= 3 and abs(red.cy - 190) <= 3
    assert abs(green.cx - 330) <= 3 and abs(green.cy - 190) <= 3
    assert abs(blue.cx - 510) <= 3 and abs(blue.cy - 190) <= 3
    assert all(block.area > 5_000 for block in (red, green, blue))


def test_detect_blocks_on_reference_image() -> None:
    image_path = WORKSPACE_ROOT / "test_perception.png"
    img = cv2.imread(str(image_path))

    assert img is not None, f"Failed to load {image_path}"

    blocks = detect_blocks(img)
    assert len(blocks) >= 3

    red = _find_nearest(blocks, "red", 75, 221)
    green = _find_nearest(blocks, "green", 550, 171)
    blue = _find_nearest(blocks, "blue", 354, 120)

    assert red.confidence >= 0.70
    assert green.confidence >= 0.85
    assert blue.confidence >= 0.85
    assert abs(red.cx - 75) <= 15 and abs(red.cy - 221) <= 20
    assert abs(green.cx - 550) <= 20 and abs(green.cy - 171) <= 20
    assert abs(blue.cx - 354) <= 20 and abs(blue.cy - 120) <= 20


def test_final_masks_match_classified_blocks() -> None:
    image_path = WORKSPACE_ROOT / "test_perception.png"
    img = cv2.imread(str(image_path))

    assert img is not None, f"Failed to load {image_path}"

    blocks, final_masks = detect_blocks_and_masks(img)

    red = _find_nearest(blocks, "red", 75, 221)
    green = _find_nearest(blocks, "green", 550, 171)
    blue = _find_nearest(blocks, "blue", 354, 120)

    assert final_masks["red"][red.cy, red.cx] > 0
    assert final_masks["green"][green.cy, green.cx] > 0
    assert final_masks["blue"][blue.cy, blue.cx] > 0

    assert final_masks["blue"][green.cy, green.cx] == 0
    assert final_masks["green"][blue.cy, blue.cx] == 0
