"""camera_supervisor - usb_cam 을 자식 프로세스로 감독한다.

arm(gripper) 에 장착된 USB 카메라(Innomaker UVC)는 모션 중 전기 노이즈/케이블
영향으로 재enumeration 되며, 그때 /dev/videoN 번호가 바뀌거나 장치가 잠시
사라진다. usb_cam 노드는 reconnect/reopen 을 지원하지 않으므로 한 번 끊기면
영구적으로 image_topic 발행이 멈춘다.

본 노드는 다음으로 그 실패 모드를 복구한다:
  1. 안정 by-id 경로(/dev/v4l/by-id/...)로 usb_cam 을 실행한다.
     -> 재enumeration 후 재오픈 시 항상 올바른 capture 노드를 잡는다.
  2. image_topic 의 frame staleness 를 감시한다.
     -> usb_cam 이 죽든(exit) 살아서 멈추든(hang) 둘 다 감지한다.
  3. staleness 또는 프로세스 종료를 감지하면 usb_cam 을 재시작하고
     v4l2 control 을 재적용한다 (재enumeration 시 control 이 초기화되므로).

모든 임계값/타임아웃은 ROS2 파라미터로 노출한다 (하드코딩 금지). 재시작은
restart_min_interval_sec 로 rate-limit 하여 crash-loop 를 방지한다.
"""
from __future__ import annotations

import os
import shlex
import signal
import subprocess
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

from omx_perception.camera_device import (
    apply_v4l2_controls,
    load_camera_parameters,
    resolve_video_device,
)


class CameraSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("camera_supervisor")

        # ── 장치 탐색 / usb_cam 실행 파라미터 ────────────────────────────
        self.declare_parameter("camera_name_match", "Innomaker")
        self.declare_parameter("video_device", "")  # explicit override (auto-detect 우선 무시)
        self.declare_parameter("video_sysfs_dir", "/sys/class/video4linux")
        self.declare_parameter("camera_params_path", "")
        self.declare_parameter("image_topic", "image/raw")
        self.declare_parameter("enable_pub_plugins", ["image_transport/raw"])

        # ── 감시 / 복구 임계값 ────────────────────────────────────────────
        self.declare_parameter("monitor_period_sec", 1.0)
        self.declare_parameter("stale_timeout_sec", 3.0)
        self.declare_parameter("startup_grace_sec", 10.0)
        self.declare_parameter("restart_min_interval_sec", 5.0)
        self.declare_parameter("kill_grace_sec", 2.0)

        self._camera_name_match = str(self.get_parameter("camera_name_match").value)
        self._explicit_device = str(self.get_parameter("video_device").value)
        self._sysfs_dir = str(self.get_parameter("video_sysfs_dir").value)
        self._camera_params_path = str(self.get_parameter("camera_params_path").value)
        self._image_topic = str(self.get_parameter("image_topic").value)
        self._enable_pub_plugins = [
            str(v) for v in self.get_parameter("enable_pub_plugins").value
        ]

        self._monitor_period = float(self.get_parameter("monitor_period_sec").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout_sec").value)
        self._startup_grace = float(self.get_parameter("startup_grace_sec").value)
        self._restart_min_interval = float(
            self.get_parameter("restart_min_interval_sec").value
        )
        self._kill_grace = float(self.get_parameter("kill_grace_sec").value)

        if not self._camera_params_path:
            raise RuntimeError("camera_params_path parameter is required")

        # `ros2 run` 래퍼는 노드를 자식으로 또 fork 해서 Popen.poll() 이 실제 노드의
        # 생존을 반영하지 못한다. 실행 파일을 직접 띄워 Popen 이 노드를 직접 추적한다.
        self._usb_cam_exe = os.path.join(
            get_package_prefix("usb_cam"), "lib", "usb_cam", "usb_cam_node_exe"
        )
        if not os.path.exists(self._usb_cam_exe):
            raise RuntimeError(f"usb_cam executable not found: {self._usb_cam_exe}")

        self._proc: Optional[subprocess.Popen] = None
        self._last_frame_sec: Optional[float] = None
        self._last_start_sec: float = 0.0

        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._image_sub = self.create_subscription(
            Image, self._image_topic, self._on_image, image_qos
        )

        # 첫 기동
        self._start_usb_cam(reason="initial start")
        self._timer = self.create_timer(self._monitor_period, self._monitor)

    # ──────────────────────────────────────────────────────────────────
    # 시간 헬퍼 (rclpy clock 기반, 단순 sleep/time 하드코딩 회피)
    # ──────────────────────────────────────────────────────────────────
    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ──────────────────────────────────────────────────────────────────
    # frame 수신
    # ──────────────────────────────────────────────────────────────────
    def _on_image(self, _msg: Image) -> None:
        self._last_frame_sec = self._now_sec()

    # ──────────────────────────────────────────────────────────────────
    # usb_cam 자식 프로세스 실행
    # ──────────────────────────────────────────────────────────────────
    def _resolve_device_path(self) -> Optional[str]:
        try:
            camera = resolve_video_device(
                self._camera_name_match,
                explicit_device=self._explicit_device,
                sysfs_dir=self._sysfs_dir,
            )
        except RuntimeError as exc:
            self.get_logger().error(f"camera resolve failed: {exc}")
            return None
        # by-id 안정 별칭으로 올바른 카메라를 "식별" 하되, usb_cam 은 relative
        # symlink 를 잘못 정규화(/dev/../../videoN)하므로 실제 노드 경로를 넘긴다.
        # 매 (재)시작마다 재해석하므로 재enumeration 으로 번호가 바뀌어도 흡수된다.
        real_path = os.path.realpath(camera.path)
        self.get_logger().info(
            f"selected camera: {camera.name} -> {camera.path} (real: {real_path})"
        )
        return real_path

    def _build_usb_cam_command(self, device_path: str) -> list[str]:
        ns = self.get_namespace()
        plugins = "[" + ",".join(self._enable_pub_plugins) + "]"
        cmd = [
            self._usb_cam_exe,
            "--ros-args",
            "--params-file", self._camera_params_path,
            "-p", f"video_device:={device_path}",
            "-p", f"image_raw.enable_pub_plugins:={plugins}",
            "-r", "__node:=usb_cam",
            "-r", "image_raw:=image/raw",
            "-r", "camera_info:=camera/info",
        ]
        if ns and ns != "/":
            cmd += ["-r", f"__ns:={ns}"]
        return cmd

    def _start_usb_cam(self, *, reason: str) -> None:
        device_path = self._resolve_device_path()
        if device_path is None:
            # 장치가 아직 안 보이면 다음 monitor tick 에서 재시도한다.
            self._last_start_sec = self._now_sec()
            return

        # 재enumeration 후 v4l2 control 은 기본값으로 초기화되므로 매 기동마다 재적용.
        try:
            applied = apply_v4l2_controls(
                device_path, load_camera_parameters(self._camera_params_path)
            )
            if applied:
                self.get_logger().info(f"applied camera controls: {', '.join(applied)}")
        except RuntimeError as exc:
            self.get_logger().warning(f"failed to apply v4l2 controls: {exc}")

        cmd = self._build_usb_cam_command(device_path)
        self.get_logger().info(f"starting usb_cam ({reason}): {shlex.join(cmd)}")
        # 자체 세션 그룹으로 띄워 종료 시 그룹 전체를 안전하게 signal 한다.
        self._proc = subprocess.Popen(cmd, start_new_session=True)
        self._last_start_sec = self._now_sec()
        # 기동 직후 grace 동안은 staleness 판정을 보류한다.
        self._last_frame_sec = None

    def _stop_usb_cam(self) -> None:
        proc = self._proc
        self._proc = None
        if proc is None or proc.poll() is not None:
            return
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=self._kill_grace)
            return
        except subprocess.TimeoutExpired:
            self.get_logger().warning("usb_cam did not exit on SIGINT, sending SIGKILL")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=self._kill_grace)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass

    def _restart_usb_cam(self, *, reason: str) -> None:
        self._stop_usb_cam()
        self._start_usb_cam(reason=reason)

    # ──────────────────────────────────────────────────────────────────
    # 주기적 감시
    # ──────────────────────────────────────────────────────────────────
    def _monitor(self) -> None:
        now = self._now_sec()

        # crash-loop 방지: 직전 기동 후 최소 간격은 재시작하지 않는다.
        since_start = now - self._last_start_sec
        if since_start < self._restart_min_interval:
            return

        # 1) 프로세스가 죽었는가? (exit case)
        if self._proc is None or self._proc.poll() is not None:
            self._restart_usb_cam(reason="usb_cam process not running")
            return

        # 2) 기동 grace 이내면 staleness 판정 보류 (부팅 + 첫 프레임 대기).
        if since_start < self._startup_grace:
            return

        # 3) frame staleness (hang case): 한 번도 못 받았거나 너무 오래 끊김.
        if self._last_frame_sec is None:
            self._restart_usb_cam(reason="no frame received within startup grace")
            return
        if (now - self._last_frame_sec) > self._stale_timeout:
            self._restart_usb_cam(
                reason=f"image stale for {now - self._last_frame_sec:.1f}s"
            )

    # ──────────────────────────────────────────────────────────────────
    # 종료 경로
    # ──────────────────────────────────────────────────────────────────
    def destroy_node(self) -> bool:
        self._stop_usb_cam()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
