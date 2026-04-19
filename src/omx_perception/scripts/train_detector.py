#!/usr/bin/env python3
"""Train YOLOv8n detector for block detection."""
from ultralytics import YOLO

model = YOLO("/home/kjhz/omx_ws/src/omx_perception/models/yolov8n.pt")

model.train(
    data="/home/kjhz/omx_ws/datasets/block_detection/data.yaml",
    epochs=150,
    imgsz=640,
    batch=16,
    project="/home/kjhz/omx_ws/datasets/block_detection",
    name="train_v2",
    device=0,
    hsv_h=0.03,   # hue shift
    hsv_s=0.9,    # saturation 강하게 변화
    hsv_v=0.6,    # brightness 강하게 변화
    degrees=10,
    translate=0.1,
    scale=0.3,
    fliplr=0.5,
)
