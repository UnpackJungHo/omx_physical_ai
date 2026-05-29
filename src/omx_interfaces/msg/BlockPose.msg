std_msgs/Header header
string color             # "red"|"green"|"blue"|"unknown"|"cup"
geometry_msgs/PoseStamped pose
geometry_msgs/PoseStamped grasp_pose
float32 confidence       # 0.0 ~ 1.0
float32 yaw_confidence   # 0.0 ~ 1.0; box yaw 신뢰도. cup 또는 산출 실패 시 0.0
# cup 의 윗면(rim) 4 모서리 world XY (header.frame_id 와 동일 frame, z 미사용).
# box 또는 polygon 산출 실패 시 빈 배열. skill 측 cup-내부 판정에 사용.
geometry_msgs/Point[] polygon
