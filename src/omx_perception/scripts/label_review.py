#!/usr/bin/env python3
"""Simple YOLO label reviewer / editor using OpenCV only.

Controls
--------
D / Right  : next image
A / Left   : prev image
Z          : undo last delete
S          : save current labels
R          : reload original labels
Q / ESC    : quit

Left click drag     : add new bbox
Right click on bbox : delete it
"""
from __future__ import annotations

import copy
from pathlib import Path

import cv2
import numpy as np

CLASSES = ["red", "green", "blue"]
COLORS = [(0, 0, 255), (0, 200, 0), (255, 80, 0)]
IMG_DIR = Path("/home/kjhz/omx_ws/datasets/block_detection/images/train")
LBL_DIR = Path("/home/kjhz/omx_ws/datasets/block_detection/labels/train")
WIN = "Label Review  D=next A=prev S=save Z=undo click=delete"


def load_labels(txt: Path) -> list[list[float]]:
    if not txt.exists():
        return []
    rows = []
    for line in txt.read_text().strip().splitlines():
        parts = line.split()
        if len(parts) == 5:
            rows.append([float(p) for p in parts])
    return rows


def save_labels(txt: Path, labels: list[list[float]]) -> None:
    txt.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"{int(r[0])} {r[1]:.6f} {r[2]:.6f} {r[3]:.6f} {r[4]:.6f}" for r in labels]
    txt.write_text("\n".join(lines) + ("\n" if lines else ""))


def yolo_to_pixel(label: list[float], w: int, h: int) -> tuple[int, int, int, int]:
    _, cx, cy, bw, bh = label
    x1 = int((cx - bw / 2) * w)
    y1 = int((cy - bh / 2) * h)
    x2 = int((cx + bw / 2) * w)
    y2 = int((cy + bh / 2) * h)
    return x1, y1, x2, y2


def pixel_to_yolo(cls: int, x1: int, y1: int, x2: int, y2: int, w: int, h: int) -> list[float]:
    cx = ((x1 + x2) / 2) / w
    cy = ((y1 + y2) / 2) / h
    bw = abs(x2 - x1) / w
    bh = abs(y2 - y1) / h
    return [float(cls), cx, cy, bw, bh]


def draw(bgr: np.ndarray, labels: list[list[float]]) -> np.ndarray:
    vis = bgr.copy()
    h, w = vis.shape[:2]
    for lbl in labels:
        cls = int(lbl[0])
        x1, y1, x2, y2 = yolo_to_pixel(lbl, w, h)
        c = COLORS[cls % len(COLORS)]
        cv2.rectangle(vis, (x1, y1), (x2, y2), c, 2)
        cv2.putText(vis, CLASSES[cls], (x1, max(y1 - 4, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 1)
    return vis


class State:
    draw_start: tuple[int, int] | None = None
    draw_cls: int = 0
    drawing: bool = False


_state = State()
_new_box_preview: tuple[int, int, int, int] | None = None


def mouse_cb(event, x, y, flags, param):
    global _new_box_preview
    labels, bgr, h, w, undo_stack = param

    if event == cv2.EVENT_RBUTTONDOWN:
        # delete box under cursor
        for i, lbl in enumerate(labels):
            x1, y1, x2, y2 = yolo_to_pixel(lbl, w, h)
            if x1 <= x <= x2 and y1 <= y <= y2:
                undo_stack.append(copy.deepcopy(labels))
                labels.pop(i)
                break

    elif event == cv2.EVENT_LBUTTONDOWN:
        _state.drawing = True
        _state.draw_start = (x, y)

    elif event == cv2.EVENT_MOUSEMOVE and _state.drawing:
        _new_box_preview = (_state.draw_start[0], _state.draw_start[1], x, y)

    elif event == cv2.EVENT_LBUTTONUP and _state.drawing:
        _state.drawing = False
        _new_box_preview = None
        sx, sy = _state.draw_start
        if abs(x - sx) > 5 and abs(y - sy) > 5:
            undo_stack.append(copy.deepcopy(labels))
            print(f"  Class? 0=red 1=green 2=blue [enter number]: ", end="", flush=True)
            try:
                cls = int(input().strip())
            except ValueError:
                cls = 0
            cls = max(0, min(2, cls))
            labels.append(pixel_to_yolo(cls, sx, sy, x, y, w, h))


def main() -> None:
    images = sorted(IMG_DIR.glob("*.jpg")) + sorted(IMG_DIR.glob("*.png"))
    if not images:
        print(f"No images found in {IMG_DIR}")
        return

    print(f"Loaded {len(images)} images. D=next A=prev S=save Z=undo click=delete")

    idx = 0
    labels: list[list[float]] = []
    undo_stack: list[list[list[float]]] = []
    bgr = None

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 720)

    def load(i: int) -> None:
        nonlocal bgr, labels, undo_stack
        bgr = cv2.imread(str(images[i]))
        if bgr is None:
            bgr = np.zeros((720, 1280, 3), dtype=np.uint8)
        labels = load_labels(LBL_DIR / (images[i].stem + ".txt"))
        undo_stack = []
        h, w = bgr.shape[:2]
        cv2.setMouseCallback(WIN, mouse_cb, [labels, bgr, h, w, undo_stack])

    load(idx)

    while True:
        h, w = bgr.shape[:2]
        vis = draw(bgr, labels)

        if _new_box_preview:
            sx, sy, ex, ey = _new_box_preview
            cv2.rectangle(vis, (sx, sy), (ex, ey), (200, 200, 200), 1)

        info = f"[{idx+1}/{len(images)}] {images[idx].name}  boxes={len(labels)}"
        cv2.putText(vis, info, (8, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow(WIN, vis)

        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), 27):
            save_labels(LBL_DIR / (images[idx].stem + ".txt"), labels)
            break
        elif key in (ord("d"), 83):  # D or Right arrow
            save_labels(LBL_DIR / (images[idx].stem + ".txt"), labels)
            idx = min(idx + 1, len(images) - 1)
            load(idx)
        elif key in (ord("a"), 81):  # A or Left arrow
            save_labels(LBL_DIR / (images[idx].stem + ".txt"), labels)
            idx = max(idx - 1, 0)
            load(idx)
        elif key == ord("s"):
            save_labels(LBL_DIR / (images[idx].stem + ".txt"), labels)
            print(f"Saved {images[idx].name}")
        elif key == ord("z") and undo_stack:
            labels[:] = undo_stack.pop()
        elif key == ord("r"):
            load(idx)

    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
