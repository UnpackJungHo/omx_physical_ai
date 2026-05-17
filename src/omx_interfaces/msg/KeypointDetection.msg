# YOLO pose detection with class info.
# class_id: 0=Box, 1=Cup (matches model class index).
# class_name: "box" | "cup" (lowercased convenience field).
# detection_confidence: YOLO box score (0..1).
# keypoints: fixed layout for four YOLOv8-Pose keypoints,
#   class_id == CLASS_BOX → 큐브 윗면 4 corners (top-left, top-right, bottom-right, bottom-left)
#   class_id == CLASS_CUP → 림(rim) 4 cardinal pixel 위치
#   [k0_x, k0_y, k0_conf,
#    k1_x, k1_y, k1_conf,
#    k2_x, k2_y, k2_conf,
#    k3_x, k3_y, k3_conf]

uint8 CLASS_BOX = 0
uint8 CLASS_CUP = 1

uint8 class_id
string class_name
float32 detection_confidence
float32[12] keypoints
string color              # "red"|"green"|"blue"|"unknown"|"" (Cup이면 "")
float32 color_confidence  # 0.0~1.0; color="" 또는 "unknown"이면 0.0
