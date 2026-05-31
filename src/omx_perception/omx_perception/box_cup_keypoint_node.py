from __future__ import annotations

from datetime import datetime
from pathlib import Path
import select
import sys
import termios
import threading
import tty
from typing import Any

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time as RclpyTime
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

import math

from omx_interfaces.msg import KeypointDetection, KeypointSample
from omx_interfaces.srv import GetBlockPoses, GetKeypointDetections


CLASS_NAMES = {
    0: "Box",
    1: "Cup",
}

CLASS_COLORS = {
    0: (0, 255, 255),
    1: (0, 165, 255),
}


class BoxCupKeypointNode(Node):
    def __init__(self) -> None:
        super().__init__("box_cup_keypoint")

        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "image/raw")
        self.declare_parameter("output_dir", "tmp_kjh/box_cup_pose_image")
        self.declare_parameter(
            "extra_pythonpath",
            "/home/kjhz/miniconda3/envs/driving/lib/python3.12/site-packages",
        )
        self.declare_parameter("device", "0")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("max_det", 20)
        self.declare_parameter("publish_debug", False)
        # snapshot 시 block (world x,y,z) -> image pixel reprojection 에 사용.
        # 기본값은 omx_perception_world_pose 와 일치시킨다.
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("camera_frame", "default_cam")
        self.declare_parameter("camera_info_topic", "camera/info")
        self.declare_parameter("tf_lookup_timeout_sec", 0.5)
        # GetKeypointDetections 처리 시 새 image frame 도착을 기다리는 단위 timeout.
        # 시간 sleep 이 아니라 condition signal 의 fallback 상한값이다 (카메라 hang
        # 같은 비정상 상황에서만 영향). 정상이면 fps 주기에 맞춰 즉시 wake.
        self.declare_parameter("frame_wait_timeout_sec", 2.0)

        self._bridge = CvBridge()
        # Condition 은 RLock 위에 wait/notify 를 얹는다. `with self._lock:` 구문은
        # 기존 그대로 동작하며, 신규 _on_image 시 notify_all() 로 대기중인 서비스
        # 핸들러를 즉시 깨운다 (시간 sleep 회피).
        self._lock = threading.Condition()
        self._latest_frame: np.ndarray | None = None
        self._latest_stamp = None
        # _on_image 가 새 frame 을 받을 때마다 증가하는 monotonic counter.
        # 서비스 핸들러가 N 개의 서로 다른 frame 을 가져갈 때 동일 frame 중복
        # 사용을 막는 trigger 로 쓴다.
        self._frame_counter = 0
        self._stop_event = threading.Event()
        self._keyboard_thread: threading.Thread | None = None
        self._model_lock = threading.Lock()

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._camera_frame = str(self.get_parameter("camera_frame").value)
        self._tf_lookup_timeout_sec = float(self.get_parameter("tf_lookup_timeout_sec").value)
        self._frame_wait_timeout_sec = float(
            self.get_parameter("frame_wait_timeout_sec").value
        )

        # CameraInfo 가 도착하면 K, D 를 채워 둔다. snapshot 시 cv2.projectPoints 에 사용.
        self._cam_K: np.ndarray | None = None
        self._cam_D: np.ndarray | None = None

        self._model_path = self._resolve_path(str(self.get_parameter("model_path").value))
        if not self._model_path.exists():
            raise FileNotFoundError(
                f"YOLO pose model not found: {self._model_path}. Set model_path to best.pt."
            )

        self._output_dir = self._resolve_path(str(self.get_parameter("output_dir").value))
        self._output_dir.mkdir(parents=True, exist_ok=True)

        YOLO = self._load_yolo_class()
        self._model = YOLO(str(self._model_path))
        self._device = str(self.get_parameter("device").value)
        self._imgsz = int(self.get_parameter("imgsz").value)
        self._conf = float(self.get_parameter("conf").value)
        self._max_det = int(self.get_parameter("max_det").value)
        self._publish_debug = bool(self.get_parameter("publish_debug").value)

        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        image_topic = str(self.get_parameter("image_topic").value)
        self._image_sub = self.create_subscription(Image, image_topic, self._on_image, image_qos)

        self._debug_pub = None
        if self._publish_debug:
            self._debug_pub = self.create_publisher(Image, "perception/debug_image", 1)

        self._service_cb_group = MutuallyExclusiveCallbackGroup()
        self._client_cb_group = ReentrantCallbackGroup()
        self._srv = self.create_service(
            GetKeypointDetections,
            "perception/get_box_cup_keypoints",
            self._handle_get_keypoints,
            callback_group=self._service_cb_group,
        )

        self._world_pose_client = self.create_client(
            GetBlockPoses,
            "perception/get_box_cup_world_poses",
            callback_group=self._client_cb_group,
        )

        # snapshot 라벨을 정확한 detection 위에 그리려면 block 의 world 좌표를
        # 이미지 픽셀로 reproject 해야 한다. TF + CameraInfo 가 필요하다.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        camera_info_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self._cam_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self._on_camera_info, camera_info_qos,
        )

        self._start_keyboard_listener()

        self.get_logger().info(
            "box/cup keypoint node ready: services=perception/get_box_cup_keypoints, "
            "press 'p' to save snapshot "
            f"(model={self._model_path}, image_topic={image_topic})"
        )

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._keyboard_thread is not None and self._keyboard_thread.is_alive():
            self._keyboard_thread.join(timeout=1.0)
        return super().destroy_node()

    def _resolve_path(self, value: str) -> Path:
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = Path.cwd() / path
        return path.resolve()

    def _load_yolo_class(self):
        extra_pythonpath = Path(str(self.get_parameter("extra_pythonpath").value)).expanduser()
        try:
            from ultralytics import YOLO
            return YOLO
        except ModuleNotFoundError:
            if extra_pythonpath.exists() and str(extra_pythonpath) not in sys.path:
                sys.path.insert(0, str(extra_pythonpath))
                self.get_logger().info(
                    f"added extra_pythonpath for YOLO dependencies: {extra_pythonpath}"
                )

            try:
                from ultralytics import YOLO
                return YOLO
            except ModuleNotFoundError as exc:
                raise ModuleNotFoundError(
                    "Missing Python package: ultralytics. Install it for /usr/bin/python3 "
                    "or set extra_pythonpath to the site-packages directory that contains ultralytics and torch."
                ) from exc

    def _on_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"failed to convert image: {exc}")
            return

        with self._lock:
            self._latest_frame = frame.copy()
            self._latest_stamp = msg.header.stamp
            self._frame_counter += 1
            # GetKeypointDetections 핸들러가 새 frame 도착을 기다리고 있다면
            # 즉시 깨워서 sample 추론을 진행하게 한다.
            self._lock.notify_all()

    def _on_camera_info(self, msg: CameraInfo) -> None:
        # K (3x3), D (k1..k5) 캐싱. usb_cam 은 항상 동일 intrinsics 를 publish 하므로
        # 매 메시지 갱신해도 동일. msg.d 가 비어 있으면 0 distortion 로 처리.
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64).flatten() if len(msg.d) else np.zeros(5, dtype=np.float64)
        with self._lock:
            self._cam_K = K
            self._cam_D = D

    def _handle_get_keypoints(
        self,
        request: GetKeypointDetections.Request,
        response: GetKeypointDetections.Response,
    ) -> GetKeypointDetections.Response:
        num_samples = max(1, int(request.num_samples))
        min_valid_samples = int(request.min_valid_samples)
        min_valid_samples = max(1, min(num_samples, min_valid_samples))

        with self._lock:
            last_seen_counter = self._frame_counter

        last_frame: np.ndarray | None = None
        last_stamp = None
        last_raw_detections: list[dict[str, Any]] = []

        for _ in range(num_samples):
            frame_data = self._wait_for_next_frame(last_seen_counter)
            if frame_data is None:
                break

            frame, stamp, last_seen_counter = frame_data
            try:
                result = self._predict_frame(frame)
            except Exception as exc:  # noqa: BLE001
                response.success = False
                response.message = f"YOLO inference error: {exc}"
                return response

            raw_detections = self._extract_detections(frame, result)
            sample = KeypointSample()
            if stamp is not None:
                sample.stamp = stamp
            sample.detections = self._to_keypoint_detection_msgs(raw_detections)
            response.samples.append(sample)

            last_frame = frame
            last_stamp = stamp
            last_raw_detections = raw_detections

        response.header.frame_id = self._camera_frame
        if last_stamp is not None:
            response.header.stamp = last_stamp

        if request.publish_debug and last_frame is not None:
            self._publish_debug_image(last_frame, last_stamp, last_raw_detections)

        collected = len(response.samples)
        response.success = collected >= min_valid_samples
        if response.success:
            if collected < num_samples:
                response.message = (
                    f"collected {collected}/{num_samples} sample(s); "
                    f"min_valid_samples={min_valid_samples}"
                )
            else:
                response.message = ""
        else:
            response.message = (
                f"collected {collected}/{num_samples} sample(s), "
                f"below min_valid_samples={min_valid_samples}"
            )
        return response

    def _predict_frame(self, frame: np.ndarray):
        with self._model_lock:
            return self._model.predict(
                source=frame,
                imgsz=self._imgsz,
                conf=self._conf,
                max_det=self._max_det,
                device=self._device,
                verbose=False,
            )[0]

    def _wait_for_next_frame(
        self,
        last_seen_counter: int,
    ) -> tuple[np.ndarray, Any, int] | None:
        with self._lock:
            got_frame = self._lock.wait_for(
                lambda: self._frame_counter > last_seen_counter or self._stop_event.is_set(),
                timeout=self._frame_wait_timeout_sec,
            )
            if not got_frame or self._latest_frame is None:
                return None

            return self._latest_frame.copy(), self._latest_stamp, self._frame_counter

    def _to_keypoint_detection_msgs(
        self,
        raw_detections: list[dict[str, Any]],
    ) -> list[KeypointDetection]:
        detections: list[KeypointDetection] = []
        for det in raw_detections:
            kd = KeypointDetection()
            kd.class_id = int(det["class_id"])
            kd.class_name = str(det["class_name"]).lower()
            kd.detection_confidence = float(det["confidence"])
            kd.color = str(det.get("color_name", kd.class_name)).lower()
            kd.color_confidence = 0.0
            flat: list[float] = []
            for kp_x, kp_y, kp_conf in det["keypoints"]:
                flat.extend([float(kp_x), float(kp_y), float(kp_conf)])
            kd.keypoints = flat
            detections.append(kd)
        return detections

    def _publish_debug_image(
        self,
        frame: np.ndarray,
        stamp: Any,
        raw_detections: list[dict[str, Any]],
    ) -> None:
        annotated = frame.copy()
        self._draw_detections(annotated, raw_detections)
        if self._debug_pub is not None:
            try:
                img_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                if stamp is not None:
                    img_msg.header.stamp = stamp
                img_msg.header.frame_id = self._camera_frame
                self._debug_pub.publish(img_msg)
            except CvBridgeError as exc:
                self.get_logger().warning(f"failed to publish debug image: {exc}")
        else:
            self.get_logger().warning(
                "publish_debug requested but publish_debug param is false; no publisher created"
            )

    def _start_keyboard_listener(self) -> None:
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()

    def _keyboard_loop(self) -> None:
        try:
            with open("/dev/tty", "r", encoding="utf-8") as tty_file:
                fd = tty_file.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setcbreak(fd)
                    while not self._stop_event.is_set() and rclpy.ok():
                        readable, _, _ = select.select([tty_file], [], [], 0.1)
                        if not readable:
                            continue
                        char = tty_file.read(1)
                        if char in ("p", "P"):
                            self._save_snapshot()
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except OSError as exc:
            self.get_logger().warning(f"keyboard listener disabled; cannot open /dev/tty: {exc}")

    def _save_snapshot(self) -> None:
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            stamp = self._latest_stamp

        if frame is None:
            self.get_logger().warning("cannot save YOLO snapshot: no image/raw frame received yet")
            return

        # 추론/저장 중 예외(예: CUDA out of memory)가 keyboard listener 스레드를
        # 죽이지 않도록 가드한다. 실패 시 로그만 남기고 스레드는 유지되어, 원인
        # (GPU 메모리 등)이 해소되면 다음 'p' 입력부터 다시 정상 동작한다.
        try:
            result = self._predict_frame(frame)
            detections = self._extract_detections(frame, result)
            annotated = frame.copy()
            self._draw_detections(annotated, detections)

            # world pose overlay (best-effort: world_pose node가 없으면 생략)
            block_poses = self._fetch_world_poses_sync()
            if block_poses is not None:
                self._draw_world_poses(annotated, detections, block_poses)

            filename = self._build_filename(stamp)
            output_path = self._output_dir / filename
            if not cv2.imwrite(str(output_path), annotated):
                self.get_logger().error(f"failed to write YOLO snapshot: {output_path}")
                return
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"YOLO snapshot failed: {exc}")
            return

        self.get_logger().info(
            f"saved YOLO snapshot: {output_path} "
            f"({len(detections)} detection(s), world_pose={'ok' if block_poses is not None else 'unavailable'})"
        )

    def _fetch_world_poses_sync(self) -> list | None:
        """키보드 스레드에서 world pose 서비스를 event 기반으로 호출한다."""
        if not self._world_pose_client.service_is_ready():
            return None

        req = GetBlockPoses.Request()
        req.color = ""

        future = self._world_pose_client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout=3.0):
            self.get_logger().warning("world pose service timed out during snapshot")
            return None

        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warning(f"world pose service error: {exc}")
            return None

        return resp.blocks

    def _quat_to_rotmat(self, x: float, y: float, z: float, w: float) -> np.ndarray:
        # 단위 quaternion 가정. TF2 가 정규화된 회전을 보장.
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
            [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
        ], dtype=np.float64)

    def _project_blocks_to_image(
        self,
        blocks: list,
        stamp: Any,
    ) -> list[tuple[float, float] | None]:
        """각 block 의 world (x,y,z) 를 카메라 이미지 픽셀 (u,v) 로 reprojection.

        반환: blocks 길이와 동일한 list. 투영 실패(intrinsics 미수신, TF 실패,
        Z<=0 등) 인 경우 해당 entry 는 None.
        """
        n = len(blocks)
        if n == 0:
            return []

        with self._lock:
            K = None if self._cam_K is None else self._cam_K.copy()
            D = None if self._cam_D is None else self._cam_D.copy()

        if K is None:
            self.get_logger().debug(
                "snapshot reprojection skipped: CameraInfo not received yet"
            )
            return [None] * n

        # camera_frame 에서 본 world 좌표가 필요하므로 TF: world -> camera_frame.
        # stamp 가 있을 때 그 시점 transform 을 우선, 없으면 latest available.
        try:
            tf_time = RclpyTime() if stamp is None else RclpyTime.from_msg(stamp)
            tf_msg = self._tf_buffer.lookup_transform(
                self._camera_frame,
                self._target_frame,
                tf_time,
                timeout=Duration(seconds=self._tf_lookup_timeout_sec),
            )
        except Exception as exc:
            self.get_logger().debug(
                f"snapshot reprojection skipped: TF lookup failed: {exc}"
            )
            return [None] * n

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        R = self._quat_to_rotmat(q.x, q.y, q.z, q.w)
        rvec, _ = cv2.Rodrigues(R)
        tvec = np.array([t.x, t.y, t.z], dtype=np.float64).reshape(3, 1)

        object_pts = np.array(
            [
                [
                    float(b.pose.pose.position.x),
                    float(b.pose.pose.position.y),
                    float(b.pose.pose.position.z),
                ]
                for b in blocks
            ],
            dtype=np.float64,
        ).reshape(-1, 1, 3)

        if D is None:
            D = np.zeros(5, dtype=np.float64)
        img_pts, _ = cv2.projectPoints(object_pts, rvec, tvec, K, D)
        img_pts = img_pts.reshape(-1, 2)

        # 카메라 뒤(Z<=0) 또는 NaN 인 경우는 None 으로 표시. cv2.projectPoints 가
        # Z<=0 일 때 부호가 뒤집힌 좌표를 돌려주므로 직접 검사한다.
        cam_pts = (R @ object_pts.reshape(-1, 3).T + tvec).T  # (n, 3)
        results: list[tuple[float, float] | None] = []
        for i in range(n):
            u, v = float(img_pts[i, 0]), float(img_pts[i, 1])
            z_cam = float(cam_pts[i, 2])
            if not (math.isfinite(u) and math.isfinite(v)) or z_cam <= 0.0:
                results.append(None)
            else:
                results.append((u, v))
        return results

    @staticmethod
    def _target_class_id_for_block(block: Any) -> int:
        # box_cup_world_pose_node 는 cup -> color="cup", box -> color="box"|"unknown"|색이름.
        # 매칭은 class_id 1=Cup, 0=Box 로만 충분하다.
        color = (block.color or "").strip().lower()
        return 1 if color == "cup" else 0

    def _match_blocks_to_detections(
        self,
        blocks: list,
        detections: list[dict[str, Any]],
        projected: list[tuple[float, float] | None],
    ) -> dict[int, int]:
        """block_idx -> detection_idx greedy 매칭.

        - block 의 target class_id 와 동일한 unassigned detection 만 후보.
        - projected (u,v) 와 detection keypoint centroid 의 픽셀 거리 최소를 선택.
        - block 은 입력 순서(=server confidence 내림차순)대로 우선 배정.
        - projected 가 None 인 block 은 매칭 시도하지 않음(미매칭 처리).
        """
        det_centroids: dict[int, tuple[float, float]] = {}
        det_class: dict[int, int] = {}
        for di, det in enumerate(detections):
            kps = [
                (float(x), float(y))
                for x, y, _ in det["keypoints"]
                if x > 0 or y > 0
            ]
            if not kps:
                continue
            cx = sum(p[0] for p in kps) / len(kps)
            cy = sum(p[1] for p in kps) / len(kps)
            det_centroids[di] = (cx, cy)
            det_class[di] = int(det["class_id"])

        assigned: set[int] = set()
        matches: dict[int, int] = {}
        for bi, block in enumerate(blocks):
            uv = projected[bi]
            if uv is None:
                continue
            target_class = self._target_class_id_for_block(block)
            best_di = -1
            best_d2 = float("inf")
            for di, (cx, cy) in det_centroids.items():
                if di in assigned:
                    continue
                if det_class.get(di) != target_class:
                    continue
                d2 = (cx - uv[0]) ** 2 + (cy - uv[1]) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best_di = di
            if best_di >= 0:
                matches[bi] = best_di
                assigned.add(best_di)
        return matches

    def _draw_world_poses(
        self,
        image: np.ndarray,
        detections: list[dict[str, Any]],
        blocks: list,
    ) -> None:
        """fused world pose 를 해당 detection 위에 라벨로 그린다.

        매칭은 (1) block.color->class_id 일치 + (2) world->image reprojection 후
        픽셀 최근접 detection 으로, confidence 순 greedy 배정한다. 이는 multi-frame
        fusion 으로 block list size/order 가 snapshot frame 의 detections 와
        달라질 수 있는 경우에도 라벨이 엉뚱한 객체에 붙는 문제를 막는다.
        """
        if not blocks:
            return

        with self._lock:
            stamp = self._latest_stamp
        projected = self._project_blocks_to_image(blocks, stamp)
        matches = self._match_blocks_to_detections(blocks, detections, projected)

        h, w = image.shape[:2]
        fallback_count = 0
        for idx, block in enumerate(blocks):
            color_name = block.color
            p = block.pose.pose.position
            q = block.pose.pose.orientation

            yaw_deg: float | None = None
            if block.yaw_confidence > 0.0:
                yaw_rad = 2.0 * math.atan2(q.z, q.w)
                yaw_deg = math.degrees(yaw_rad)

            lines = [
                f"x={p.x:.3f}m  y={p.y:.3f}m",
                f"z={p.z:.3f}m  [{color_name}]",
            ]
            if yaw_deg is not None:
                lines.append(f"yaw={yaw_deg:.1f}deg")

            if idx in matches:
                anchor_x, anchor_y = self._detection_centroid(detections, matches[idx])
            else:
                uv = projected[idx]
                if uv is not None and 0 <= uv[0] < w and 0 <= uv[1] < h:
                    anchor_x, anchor_y = int(round(uv[0])), int(round(uv[1]))
                else:
                    # 매칭/투영 모두 실패: 영상 왼쪽 상단에 순서대로 fallback 표기.
                    anchor_x, anchor_y = 10, 60 + fallback_count * 60
                    fallback_count += 1

            bg_x = anchor_x
            bg_y = anchor_y + 12
            line_h = 16
            box_w = 180
            box_h = line_h * len(lines) + 6
            cv2.rectangle(
                image,
                (bg_x - 2, bg_y - line_h),
                (bg_x + box_w, bg_y + box_h - line_h),
                (20, 20, 20),
                -1,
            )
            for li, line in enumerate(lines):
                cv2.putText(
                    image,
                    line,
                    (bg_x, bg_y + li * line_h),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.40,
                    (220, 220, 220),
                    1,
                    cv2.LINE_AA,
                )

    def _detection_centroid(
        self,
        detections: list[dict[str, Any]],
        idx: int,
    ) -> tuple[int, int]:
        if idx < len(detections):
            kps = [
                (int(round(x)), int(round(y)))
                for x, y, _ in detections[idx]["keypoints"]
                if x > 0 or y > 0
            ]
            if kps:
                cx = int(sum(p[0] for p in kps) / len(kps))
                cy = int(sum(p[1] for p in kps) / len(kps))
                return cx, cy
        # fallback: 이미지 왼쪽 상단에 순서대로 나열
        return 10, 60 + idx * 60

    def _build_filename(self, stamp: Any) -> str:
        now = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        if stamp is None:
            return f"box_cup_pose_{now}.jpg"
        return f"box_cup_pose_{stamp.sec}_{stamp.nanosec}_{now}.jpg"

    def _determine_box_color(self, frame: np.ndarray, points: list[tuple[float, float, float]]) -> str:
        """HSV 색상 공간을 이용하여 중심 영역의 색상을 판별한다."""
        kps = [(x, y) for x, y, conf in points if x > 0 or y > 0]
        if not kps:
            return "box"
            
        cx = int(round(sum(p[0] for p in kps) / len(kps)))
        cy = int(round(sum(p[1] for p in kps) / len(kps)))
        
        h, w = frame.shape[:2]
        if cx < 0 or cx >= w or cy < 0 or cy >= h:
            return "box"
            
        roi_size = 10
        x1 = max(0, cx - roi_size // 2)
        x2 = min(w, cx + roi_size // 2)
        y1 = max(0, cy - roi_size // 2)
        y2 = min(h, cy + roi_size // 2)
        
        if x1 >= x2 or y1 >= y2:
            return "box"
            
        roi = frame[y1:y2, x1:x2]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        hue = np.median(hsv_roi[:, :, 0])
        sat = np.median(hsv_roi[:, :, 1])
        val = np.median(hsv_roi[:, :, 2])
        
        if sat > 50 and val > 50:
            if hue <= 10 or hue >= 160:
                return "red"
            elif 35 <= hue <= 85:
                return "green"
            elif 100 <= hue <= 140:
                return "blue"
            elif 15 <= hue <= 35:
                return "yellow"
                
        return "box"

    def _extract_detections(self, frame: np.ndarray, result) -> list[dict[str, Any]]:
        if result.boxes is None or len(result.boxes) == 0 or result.keypoints is None:
            return []

        boxes_conf = result.boxes.conf.detach().cpu().numpy()
        boxes_cls = result.boxes.cls.detach().cpu().numpy().astype(int)
        keypoints_xy = result.keypoints.xy.detach().cpu().numpy()
        keypoints_conf = None
        if result.keypoints.conf is not None:
            keypoints_conf = result.keypoints.conf.detach().cpu().numpy()

        detections: list[dict[str, Any]] = []
        for detection_index in np.argsort(-boxes_conf):
            idx = int(detection_index)
            points = keypoints_xy[idx]
            if len(points) < 4:
                continue

            class_id = int(boxes_cls[idx])
            keypoints = []
            for keypoint_index in range(4):
                point = points[keypoint_index]
                confidence = 1.0
                if keypoints_conf is not None:
                    confidence = float(keypoints_conf[idx][keypoint_index])
                keypoints.append((float(point[0]), float(point[1]), confidence))

            class_name = CLASS_NAMES.get(class_id, f"class_{class_id}")
            color_name = class_name.lower()
            if class_id == 0:  # Box
                color_name = self._determine_box_color(frame, keypoints)

            detections.append(
                {
                    "class_id": class_id,
                    "class_name": class_name,
                    "color_name": color_name,
                    "confidence": float(boxes_conf[idx]),
                    "keypoints": keypoints,
                }
            )

        return detections

    def _draw_detections(self, image: np.ndarray, detections: list[dict[str, Any]]) -> None:
        cv2.putText(
            image,
            f"YOLO box/cup pose: {len(detections)}",
            (16, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        for det_index, detection in enumerate(detections):
            class_id = int(detection["class_id"])
            class_name = str(detection["class_name"])
            color = CLASS_COLORS.get(class_id, (180, 180, 180))
            points: list[tuple[int, int]] = []

            for keypoint_index, (x_value, y_value, keypoint_conf) in enumerate(detection["keypoints"]):
                x = int(round(x_value))
                y = int(round(y_value))
                if x <= 0 and y <= 0:
                    continue

                points.append((x, y))
                cv2.circle(image, (x, y), 5, color, -1, lineType=cv2.LINE_AA)
                cv2.circle(image, (x, y), 7, (0, 0, 0), 1, lineType=cv2.LINE_AA)
                cv2.putText(
                    image,
                    f"{class_name}{det_index}.p{keypoint_index}:{keypoint_conf:.2f}",
                    (x + 7, y - 7),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    color,
                    1,
                    cv2.LINE_AA,
                )

            if len(points) == 4:
                for start, end in ((0, 1), (1, 2), (2, 3), (3, 0)):
                    cv2.line(image, points[start], points[end], color, 2, lineType=cv2.LINE_AA)

            if points:
                color_name = str(detection.get("color_name", class_name))
                label = f"{color_name} {det_index}: {float(detection['confidence']):.2f}"
                cv2.putText(
                    image,
                    label,
                    (points[0][0], max(18, points[0][1] - 24)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                    cv2.LINE_AA,
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BoxCupKeypointNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
