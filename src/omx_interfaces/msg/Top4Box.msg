float32 detection_confidence

# Fixed layout for four YOLOv8-Pose keypoints:
# [k0_x, k0_y, k0_conf,
#  k1_x, k1_y, k1_conf,
#  k2_x, k2_y, k2_conf,
#  k3_x, k3_y, k3_conf]
float32[12] keypoints
