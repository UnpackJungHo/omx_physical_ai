"""omx_perception — BlockDetectionNode

Subscribes : /image_raw, /camera_info
Service     : /omx/get_block_poses  (omx_interfaces/srv/GetBlockPoses)
Publishes   : /omx/perception/debug_image
              /omx/perception/mask_final/red
              /omx/perception/mask_final/green
              /omx/perception/mask_final/blue

HSV tuning via rqt_reconfigure
-------------------------------
rqt → Plugins → Configuration → Dynamic Reconfigure → block_detection_node

Parameters exposed (all integers, 0-255 / 0-180 for hue):
  red_h1_lo / red_h1_hi   — first hue band  (0~10 wraps near 0)
  red_h2_lo / red_h2_hi   — second hue band (170~180 wraps near 180)
  red_s_lo  / red_v_lo    — shared saturation / value floor for red
  green_h_lo / green_h_hi / green_s_lo / green_v_lo
  blue_h_lo  / blue_h_hi  / blue_s_lo  / blue_v_lo
  min_contour_area        — minimum detection area in px²
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from omx_interfaces.msg import BlockPose
from omx_interfaces.srv import GetBlockPoses

from .color_detector import (
    HsvRangeMap,
    detect_blocks_and_masks_from_hsv,
    draw_detections,
    preprocess_hsv,
)
from .pixel_to_3d import PixelTo3D


def _int_desc(description: str, lo: int, hi: int) -> ParameterDescriptor:
    r = IntegerRange(from_value=lo, to_value=hi, step=1)
    d = ParameterDescriptor(description=description)
    d.integer_range = [r]
    return d


class BlockDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__("block_detection_node")

        # ── static parameters ───────────────────────────────────────
        self.declare_parameter("camera_frame",      "default_cam")
        self.declare_parameter("image_topic",       "/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("table_z_in_camera", 0.30)
        self.declare_parameter("publish_debug_image", True)

        self._camera_frame = self.get_parameter("camera_frame").value
        self._publish_debug: bool = self.get_parameter("publish_debug_image").value
        table_z: float = self.get_parameter("table_z_in_camera").value
        image_topic: str = self.get_parameter("image_topic").value
        camera_info_topic: str = self.get_parameter("camera_info_topic").value

        # ── HSV tuning parameters (rqt_reconfigure) ─────────────────
        H = lambda d, l, h: _int_desc(d, l, h)  # noqa: E731

        self.declare_parameter("red_h1_lo",  0,   H("Red hue-band1 lower (0-180)",    0, 180))
        self.declare_parameter("red_h1_hi",  10,  H("Red hue-band1 upper (0-180)",    0, 180))
        self.declare_parameter("red_h2_lo",  155, H("Red hue-band2 lower (0-180)",    0, 180))
        self.declare_parameter("red_h2_hi",  180, H("Red hue-band2 upper (0-180)",    0, 180))
        self.declare_parameter("red_s_lo",   35,  H("Red saturation lower (0-255)",   0, 255))
        self.declare_parameter("red_v_lo",   80,  H("Red value lower (0-255)",        0, 255))

        self.declare_parameter("green_h_lo", 75,  H("Green hue lower (0-180)",        0, 180))
        self.declare_parameter("green_h_hi", 88,  H("Green hue upper (0-180)",        0, 180))
        self.declare_parameter("green_s_lo", 80,  H("Green saturation lower (0-255)", 0, 255))
        self.declare_parameter("green_v_lo", 70,  H("Green value lower (0-255)",      0, 255))

        self.declare_parameter("blue_h_lo",  90,  H("Blue hue lower (0-180)",         0, 180))
        self.declare_parameter("blue_h_hi",  118, H("Blue hue upper (0-180)",         0, 180))
        self.declare_parameter("blue_s_lo",  175, H("Blue saturation lower (0-255) — applied to S-BOOSTED image",  0, 255))
        self.declare_parameter("blue_v_lo",  70,  H("Blue value lower (0-255)",       0, 255))

        self.declare_parameter("min_contour_area", 2000,
                               H("Minimum contour area (px²)", 100, 50000))

        self._hsv_ranges: HsvRangeMap = self._build_hsv_ranges()
        self._min_area: int = self.get_parameter("min_contour_area").value

        self.add_on_set_parameters_callback(self._on_params_changed)

        # ── internal state ───────────────────────────────────────────
        self._bridge = CvBridge()
        self._pixel_to_3d = PixelTo3D(table_z_in_camera=table_z)
        self._latest_image: np.ndarray | None = None
        self._latest_blocks: list = []   # cache — refreshed every frame

        # ── QoS ─────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── subscriptions ────────────────────────────────────────────
        self.create_subscription(Image,      image_topic,       self._cb_image,       sensor_qos)
        self.create_subscription(CameraInfo, camera_info_topic, self._cb_camera_info, sensor_qos)

        # ── service ──────────────────────────────────────────────────
        self._srv = self.create_service(
            GetBlockPoses, "/omx/get_block_poses", self._handle_get_block_poses
        )

        # ── publishers ───────────────────────────────────────────────
        self._debug_pub = (
            self.create_publisher(Image, "/omx/perception/debug_image", 1)
            if self._publish_debug else None
        )
        self._mask_pubs = {
            color: self.create_publisher(Image, f"/omx/perception/mask_final/{color}", 1)
            for color in ("red", "green", "blue")
        }

        self.get_logger().info(
            f"BlockDetectionNode ready. "
            f"image={image_topic}, camera_info={camera_info_topic}, "
            f"camera_frame={self._camera_frame}, table_z={table_z:.3f}m\n"
            f"  HSV tuning: ros2 run rqt_reconfigure rqt_reconfigure"
        )

    # ---------------------------------------------------------------- #
    # Parameter callback (rqt_reconfigure)
    # ---------------------------------------------------------------- #

    def _on_params_changed(self, params: list) -> SetParametersResult:
        hsv_keys = {
            "red_h1_lo", "red_h1_hi", "red_h2_lo", "red_h2_hi",
            "red_s_lo", "red_v_lo",
            "green_h_lo", "green_h_hi", "green_s_lo", "green_v_lo",
            "blue_h_lo",  "blue_h_hi",  "blue_s_lo",  "blue_v_lo",
            "min_contour_area",
        }
        touched = {p.name for p in params} & hsv_keys
        if touched:
            overrides = {p.name: p.value for p in params if p.name in hsv_keys}
            error = self._validate_hsv_overrides(overrides)
            if error is not None:
                return SetParametersResult(successful=False, reason=error)

            self._hsv_ranges = self._build_hsv_ranges(overrides)
            if "min_contour_area" in overrides:
                self._min_area = int(overrides["min_contour_area"])
            self.get_logger().info(f"HSV params updated: {touched}")
        return SetParametersResult(successful=True)

    def _validate_hsv_overrides(self, overrides: dict) -> str | None:
        def g(name: str) -> int:
            if name in overrides:
                return int(overrides[name])
            return int(self.get_parameter(name).value)

        ordered_pairs = (
            ("red_h1_lo", "red_h1_hi"),
            ("red_h2_lo", "red_h2_hi"),
            ("green_h_lo", "green_h_hi"),
            ("blue_h_lo", "blue_h_hi"),
        )
        for lo_name, hi_name in ordered_pairs:
            lo = g(lo_name)
            hi = g(hi_name)
            if lo > hi:
                return f"Invalid HSV range: {lo_name} ({lo}) must be <= {hi_name} ({hi})"

        min_area = g("min_contour_area")
        if min_area <= 0:
            return f"Invalid min_contour_area: {min_area}"

        return None

    def _build_hsv_ranges(self, overrides: dict | None = None) -> HsvRangeMap:
        def g(name: str) -> int:
            if overrides and name in overrides:
                return int(overrides[name])
            return int(self.get_parameter(name).value)

        return {
            "red": [
                (np.array([g("red_h1_lo"), g("red_s_lo"), g("red_v_lo")]),
                 np.array([g("red_h1_hi"), 255,           255])),
                (np.array([g("red_h2_lo"), g("red_s_lo"), g("red_v_lo")]),
                 np.array([g("red_h2_hi"), 255,           255])),
            ],
            "green": [
                (np.array([g("green_h_lo"), g("green_s_lo"), g("green_v_lo")]),
                 np.array([g("green_h_hi"), 255,             255])),
            ],
            "blue": [
                (np.array([g("blue_h_lo"), g("blue_s_lo"), g("blue_v_lo")]),
                 np.array([g("blue_h_hi"), 255,            255])),
            ],
        }

    # ---------------------------------------------------------------- #
    # Subscriptions
    # ---------------------------------------------------------------- #

    def _cb_image(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn(f"cv_bridge error: {e}", throttle_duration_sec=5.0)
            return

        self._latest_image = frame
        hsv = preprocess_hsv(frame)

        # ── run detection every frame ────────────────────────────────
        self._latest_blocks, final_masks = detect_blocks_and_masks_from_hsv(
            hsv,
            hsv_ranges=self._hsv_ranges,
            min_area=self._min_area,
        )

        # ── publish final masks for all three colors ─────────────────
        for color, pub in self._mask_pubs.items():
            try:
                mask = final_masks.get(color)
                if mask is None:
                    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
                mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                pub.publish(self._bridge.cv2_to_imgmsg(mask_bgr, "bgr8"))
            except CvBridgeError:
                pass

        # ── publish debug image (raw frame + detections drawn) ───────
        if self._debug_pub is not None:
            try:
                debug = draw_detections(frame, self._latest_blocks)
                self._debug_pub.publish(self._bridge.cv2_to_imgmsg(debug, "bgr8"))
            except CvBridgeError:
                pass

    def _cb_camera_info(self, msg: CameraInfo) -> None:
        if not self._pixel_to_3d.is_ready:
            self._pixel_to_3d.update_camera_info(msg)
            self.get_logger().info("CameraInfo received — projection ready.")

    # ---------------------------------------------------------------- #
    # Service handler
    # ---------------------------------------------------------------- #

    def _handle_get_block_poses(
        self,
        request: GetBlockPoses.Request,
        response: GetBlockPoses.Response,
    ) -> GetBlockPoses.Response:
        if self._latest_image is None:
            self.get_logger().warn("No image received yet.")
            return response

        if not self._pixel_to_3d.is_ready:
            self.get_logger().warn("CameraInfo not received yet.")
            return response

        target_color = request.color.strip().lower() if request.color else ""

        # Use cached detection results; filter by color if requested
        blocks = [
            b for b in self._latest_blocks
            if not target_color or b.color == target_color
        ]

        stamp = self.get_clock().now().to_msg()
        for b in blocks:
            try:
                x, y, z = self._pixel_to_3d.pixel_to_camera_point(b.cx, b.cy)
            except RuntimeError:
                continue

            pose_stamped = PoseStamped(
                header=Header(frame_id=self._camera_frame, stamp=stamp),
            )
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = z
            pose_stamped.pose.orientation.w = 1.0

            response.blocks.append(BlockPose(
                header=Header(frame_id=self._camera_frame, stamp=stamp),
                color=b.color,
                pose=pose_stamped,
                confidence=b.confidence,
            ))

        self.get_logger().info(
            f"GetBlockPoses: color='{target_color or 'all'}' → {len(response.blocks)} blocks"
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlockDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
