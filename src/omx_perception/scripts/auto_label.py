#!/usr/bin/env python3
"""Auto-label images using HSV+contour detector → YOLO detection format.

Output: labels/train/<image_stem>.txt
Format: class_id cx cy w h  (normalized 0-1)
Classes: 0=red  1=green  2=blue

Review output with:
  labelImg <image_dir> <label_dir>
and correct false positives / missed detections.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

CLASS_ID = {"red": 0, "green": 1, "blue": 2}

_COLOR_RANGES: dict[str, list[tuple[np.ndarray, np.ndarray]]] = {
    "red": [
        (np.array([0, 100, 80]), np.array([10, 255, 255])),
        (np.array([160, 100, 80]), np.array([180, 255, 255])),
    ],
    "green": [
        (np.array([40, 80, 80]), np.array([85, 255, 255])),
    ],
    "blue": [
        (np.array([100, 80, 80]), np.array([135, 255, 255])),
    ],
}


def detect(bgr: np.ndarray, min_area: int) -> list[tuple[str, float, float, float, float]]:
    """Return list of (color, cx_norm, cy_norm, w_norm, h_norm)."""
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    results = []

    for color, ranges in _COLOR_RANGES.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) < min_area:
                continue
            x, y, bw, bh = cv2.boundingRect(cnt)
            cx_n = (x + bw / 2) / w
            cy_n = (y + bh / 2) / h
            w_n = bw / w
            h_n = bh / h
            results.append((color, cx_n, cy_n, w_n, h_n))

    return results


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", default="/home/kjhz/omx_ws/datasets/block_detection/images/train")
    parser.add_argument("--labels", default="/home/kjhz/omx_ws/datasets/block_detection/labels/train")
    parser.add_argument("--min-area", type=int, default=300)
    parser.add_argument("--preview", action="store_true", help="show detection preview per image")
    args = parser.parse_args()

    img_dir = Path(args.images)
    lbl_dir = Path(args.labels)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    images = sorted(img_dir.glob("*.jpg")) + sorted(img_dir.glob("*.png"))
    print(f"Processing {len(images)} images  →  {lbl_dir}")

    skipped = 0
    for img_path in images:
        bgr = cv2.imread(str(img_path))
        if bgr is None:
            print(f"  [WARN] cannot read {img_path.name}")
            continue

        detections = detect(bgr, args.min_area)
        lbl_path = lbl_dir / (img_path.stem + ".txt")

        with lbl_path.open("w") as f:
            for color, cx, cy, w, h in detections:
                f.write(f"{CLASS_ID[color]} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n")

        if not detections:
            skipped += 1

        if args.preview:
            vis = bgr.copy()
            ih, iw = bgr.shape[:2]
            for color, cx, cy, bw, bh in detections:
                x1 = int((cx - bw / 2) * iw)
                y1 = int((cy - bh / 2) * ih)
                x2 = int((cx + bw / 2) * iw)
                y2 = int((cy + bh / 2) * ih)
                c = {"red": (0,0,255), "green": (0,200,0), "blue": (255,80,0)}[color]
                cv2.rectangle(vis, (x1, y1), (x2, y2), c, 2)
                cv2.putText(vis, color, (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1)
            cv2.imshow("preview — any key: next  q: quit", vis)
            if cv2.waitKey(0) & 0xFF == ord("q"):
                break

    cv2.destroyAllWindows()
    print(f"Done. labeled={len(images)-skipped}  empty={skipped}")
    print("Review with: labelImg <images_dir> <labels_dir>")


if __name__ == "__main__":
    main()
