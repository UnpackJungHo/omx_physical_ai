#!/usr/bin/env python3
"""Split train images/labels into train 80% / val 20%."""
from __future__ import annotations

import random
import shutil
from pathlib import Path

DATASET = Path("/home/kjhz/omx_ws/datasets/block_detection")
VAL_RATIO = 0.2
SEED = 42

src_img = DATASET / "images" / "train"
src_lbl = DATASET / "labels" / "train"
val_img = DATASET / "images" / "val"
val_lbl = DATASET / "labels" / "val"
val_img.mkdir(parents=True, exist_ok=True)
val_lbl.mkdir(parents=True, exist_ok=True)

images = sorted(src_img.glob("*.jpg")) + sorted(src_img.glob("*.png"))
random.seed(SEED)
random.shuffle(images)

n_val = int(len(images) * VAL_RATIO)
val_set = images[:n_val]

for img_path in val_set:
    lbl_path = src_lbl / (img_path.stem + ".txt")
    shutil.move(str(img_path), val_img / img_path.name)
    if lbl_path.exists():
        shutil.move(str(lbl_path), val_lbl / lbl_path.name)

print(f"train: {len(images)-n_val}  val: {n_val}")
