std_msgs/Header header
string color             # "red"|"green"|"blue"|"unknown"|"cup"
geometry_msgs/PoseStamped pose
geometry_msgs/PoseStamped grasp_pose
float32 confidence       # 0.0 ~ 1.0
float32 yaw_confidence   # 0.0 ~ 1.0; box yaw 신뢰도. cup 또는 산출 실패 시 0.0
