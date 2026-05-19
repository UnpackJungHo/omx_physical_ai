# box_cup_world_pose + PnP 기하 C++ 이식 설계

- 날짜: 2026-05-19
- 브랜치: feat/box-cup-perception
- 대상: `box_cup_world_pose_node` 와 PnP 기하 헬퍼를 Python -> C++ 로 이식

## 배경 / 동기

`omx_perception` 의 perception 노드는 모두 Python 으로 작성돼 있다. 그중
`box_cup_world_pose_node` 는 YOLO keypoint 를 받아 solvePnP 와 TF 변환으로
world 좌표 `BlockPose` 를 만드는 노드로, 다음 이유에서 C++ 가 더 적합하다.

- solvePnP / Rodrigues / projectPoints 는 OpenCV C++ 가 본토이고 Python 은 바인딩이다.
- tf2 도 C++ API 가 본체다.
- 실시간 제어 인접 경로는 C++ 우선이라는 프로젝트 규칙에 부합한다.
- 개발자(JH)의 C++ 역량/포트폴리오 방향과 일치한다.

YOLO 추론 노드(`box_cup_pose_node`)는 ultralytics 의존성 때문에 Python 으로 남는다.

## 범위

이식 대상:
- `box_cup_world_pose_node` (서비스 노드)
- PnP 기하 헬퍼: `object_points_for_class`, `solve_pnp_planar_disambiguated`,
  `wrap_yaw_to_pm45`, `load_intrinsics`
- world 노드 안에 있던 헬퍼: `box_yaw_world`, `quaternion_to_rotation_matrix`,
  `quaternion_from_yaw`

이식 제외 (Python 유지):
- `box_cup_pose_node` (YOLO 추론)
- `color_classifier.py`, `box_color_calibrate.py`, `camera_control_node.py`
- `pnp_geometry.py` / `test_pnp_geometry.py` — `box_cup_pose_node` 의 디버그
  축 그리기(`camera_frame_yaw` 포함)가 계속 사용하므로 그대로 둔다.

## 아키텍처

### 새 패키지 `omx_perception_cpp` (ament_cmake)

기존 ament_python `omx_perception` 은 손대지 않는다. `omx_motion_server` 와
동일한 ament_cmake 패턴으로 새 패키지를 만든다.

```
omx_perception_cpp/
  CMakeLists.txt
  package.xml
  include/omx_perception_cpp/pnp_geometry.hpp
  src/pnp_geometry.cpp
  src/box_cup_world_pose_node.cpp
  test/test_pnp_geometry.cpp
```

의존성: `rclcpp`, `omx_interfaces`, `geometry_msgs`, `tf2`, `tf2_ros`,
`OpenCV`(core, calib3d), `yaml-cpp`. 테스트: `ament_cmake_gtest`,
`ament_lint_auto`.

### pnp_geometry (순수 함수)

`pnp_geometry.hpp` / `pnp_geometry.cpp` 에 순수 기하 함수를 모은다. rclcpp
의존 없이 OpenCV/Eigen 수준만 쓰므로 gtest 로 독립 검증 가능하다.

- `objectPointsForClass(class_id, cube_size_m, cup_radius_m) -> cv::Mat(4x3)`
  지원하지 않는 class_id 는 `std::invalid_argument`.
- `solvePnpPlanarDisambiguated(object_points, image_points, K, dist)
  -> {ok, rvec, tvec}` — `SOLVEPNP_IPPE` 로 두 해를 구해 object +z 가
  카메라를 향하는 해만 남기고 reprojection error 최소 해 선택, IPPE 불가 시
  `SOLVEPNP_ITERATIVE` fallback.
- `wrapYawToPm45(yaw) -> double` — 정육면체 90° 대칭으로 (-pi/4, pi/4] 접기.
- `loadIntrinsics(path) -> {camera_matrix(3x3), dist_coeffs}` — yaml-cpp 로
  `camera_matrix.data` / `distortion_coefficients.data` 파싱.
- `boxYawWorld(rvec, rotation_world_cam) -> double` — box +x 축을 world 로
  옮겨 world z 기준 yaw 산출.
- `quaternionToRotationMatrix(x,y,z,w) -> cv::Matx33d`
- `quaternionFromYaw(yaw) -> {x,y,z,w}`

Python `camera_frame_yaw` 는 이식하지 않는다 (디버그 viz 전용, Python 잔존).

### box_cup_world_pose_node.cpp

Python 노드와 동일한 ROS2 인터페이스/파라미터/동작을 유지한다.

- 서비스 서버: `GetBlockPoses` (기본 `/perception/get_box_cup_world_poses`)
- 서비스 클라이언트: `GetKeypointDetections`
  (기본 `/perception/get_box_cup_keypoints`)
- `tf2_ros::Buffer` + `tf2_ros::TransformListener`
- 파라미터: `camera_intrinsics_path`, `keypoints_service_name`,
  `world_service_name`, `target_frame`, `camera_frame`, `cube_size_m`,
  `box_output_z_m`, `cup_radius_m`, `cup_height_m`, `cup_output_z_m`,
  `min_keypoint_confidence`, `keypoints_timeout_sec`, `keypoint_order`
  (Python 노드와 1:1 동일).

동작 흐름 (Python 노드와 동일):
1. world 서비스 요청 수신 -> keypoint 서비스 호출.
2. 응답 header 의 frame_id / stamp 로 `target_frame <- source_frame` TF lookup
   (stamp 가 0 이면 latest fallback).
3. 각 detection 에 `keypoint_order` 적용, confidence < 임계값이면 drop.
4. `solvePnpPlanarDisambiguated` 로 카메라 좌표 pose 산출.
5. 카메라 좌표 -> world 변환. Box 는 `boxYawWorld` + `wrapYawToPm45` 로 yaw
   quaternion, Cup 은 identity orientation + `yaw_confidence=0`.
6. `pose.position.z` 는 클래스별 `*_output_z_m` 로 덮어쓴다.
7. `BlockPose` 리스트로 응답.

### 동시성 설계

서비스 콜백 안에서 다른 서비스를 동기 호출한다. C++ rclcpp 에서 단일 스레드
executor 로 이를 하면 deadlock 이므로:

- 서버 콜백 그룹과 클라이언트 콜백 그룹을 별도 `MutuallyExclusive` 로 생성.
- `MultiThreadedExecutor` 로 spin.
- 서버 콜백은 클라이언트 future 를 `wait_for(timeout)` 로 대기. 콜백 그룹이
  분리돼 있어 다른 스레드가 클라이언트 응답을 처리한다.

Python 노드의 `ReentrantCallbackGroup` + `MultiThreadedExecutor` 설계와 동등.

## 에러 처리

- intrinsics 파일 없음/파싱 실패: 노드 생성자에서 예외 -> 노드 기동 실패.
- keypoint 서비스 unavailable / 타임아웃 / 호출 실패: 경고 로그 후 빈
  `blocks` 응답.
- TF lookup 실패: 경고 로그 후 빈 `blocks` 응답.
- keypoint confidence 미달 / solvePnP 실패 / 미지원 class_id: 해당 detection
  만 drop, 나머지는 계속 처리.
- `keypoint_order` 가 0,1,2,3 순열이 아니면 생성자에서 예외.

## 기존 자산 처리

- `src/omx_perception/omx_perception/box_cup_world_pose_node.py` 삭제.
- `setup.py` 의 `box_cup_world_pose_node` entry point 제거.
- `test/test_box_cup_world_pose_helpers.py` 삭제 (테스트하던 헬퍼가 C++ 로
  이동). C++ gtest 로 대체.
- `pnp_geometry.py`, `test_pnp_geometry.py` 는 유지.

## launch 변경

`perception.launch.py` 의 `box_cup_world_pose` Node 를
`package="omx_perception_cpp"`, `executable="box_cup_world_pose_node"` 로
변경. 파라미터 블록은 그대로 둔다 (이름/타입 동일).

## 테스트

`test/test_pnp_geometry.cpp` (`ament_add_gtest`) — Python
`test_pnp_geometry.py` / `test_box_cup_world_pose_helpers.py` 를 포팅:
- `objectPointsForClass` box/cup 좌표, 미지원 class 예외.
- `solvePnpPlanarDisambiguated` 가 알려진 pose 복원, object +z 가 카메라
  방향, 기울어진 view 에서 reprojection error / yaw 복원, near-degenerate
  view 에서 유효 fit.
- `wrapYawToPm45` 범위 유지 / 대칭 접기.
- `quaternionToRotationMatrix` identity / z-180.
- `boxYawWorld` identity / object 회전 / world_cam 회전 / 합성.
- `quaternionFromYaw` roundtrip.

노드 레벨은 하드웨어/카메라 의존이라 빌드 + 순수 함수 gtest 까지 검증하고,
런타임 검증은 launch 로 별도 수행한다.

## 검증

- `colcon build --symlink-install --packages-select omx_perception_cpp`
- `colcon test --packages-select omx_perception_cpp` (gtest + lint 통과)
- `omx_perception` 재빌드로 Python 쪽 entry point 제거 정상 반영 확인.
