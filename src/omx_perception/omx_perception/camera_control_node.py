from __future__ import annotations

import shutil
import subprocess
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node


@dataclass(frozen=True)
class CameraControlState:
    video_device: str = "/dev/video0"
    autoexposure: bool = False
    exposure: int = 100
    auto_white_balance: bool = False
    white_balance: int = 4500
    gain: int = 4
    brightness: int = 128
    contrast: int = 32
    saturation: int = 32
    sharpness: int = 24


def build_v4l2_command(
    state: CameraControlState,
    executable: str = "v4l2-ctl",
) -> list[str]:
    command = [
        executable,
        "-d",
        state.video_device,
        "-c",
        f"auto_exposure={3 if state.autoexposure else 1}",
        "-c",
        "exposure_dynamic_framerate=0",
        "-c",
        f"white_balance_automatic={1 if state.auto_white_balance else 0}",
        "-c",
        f"gain={int(state.gain)}",
        "-c",
        f"brightness={int(state.brightness)}",
        "-c",
        f"contrast={int(state.contrast)}",
        "-c",
        f"saturation={int(state.saturation)}",
        "-c",
        f"sharpness={int(state.sharpness)}",
    ]

    if not state.autoexposure:
        command.extend(["-c", f"exposure_time_absolute={int(state.exposure)}"])

    if not state.auto_white_balance:
        command.extend(["-c", f"white_balance_temperature={int(state.white_balance)}"])

    return command


class CameraControlNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_control")

        defaults = CameraControlState()
        self.declare_parameter(
            "video_device",
            defaults.video_device,
            ParameterDescriptor(description="V4L2 video device path"),
        )
        self.declare_parameter(
            "autoexposure",
            defaults.autoexposure,
            ParameterDescriptor(description="Enable camera auto exposure"),
        )
        self.declare_parameter(
            "exposure",
            defaults.exposure,
            ParameterDescriptor(description="Manual exposure time for V4L2 exposure_time_absolute"),
        )
        self.declare_parameter(
            "auto_white_balance",
            defaults.auto_white_balance,
            ParameterDescriptor(description="Enable camera auto white balance"),
        )
        self.declare_parameter(
            "white_balance",
            defaults.white_balance,
            ParameterDescriptor(description="Manual white balance temperature in Kelvin"),
        )
        self.declare_parameter(
            "gain",
            defaults.gain,
            ParameterDescriptor(description="Camera analog/digital gain"),
        )
        self.declare_parameter(
            "brightness",
            defaults.brightness,
            ParameterDescriptor(description="Camera brightness control"),
        )
        self.declare_parameter(
            "contrast",
            defaults.contrast,
            ParameterDescriptor(description="Camera contrast control"),
        )
        self.declare_parameter(
            "saturation",
            defaults.saturation,
            ParameterDescriptor(description="Camera saturation control"),
        )
        self.declare_parameter(
            "sharpness",
            defaults.sharpness,
            ParameterDescriptor(description="Camera sharpness control"),
        )

        self._v4l2_ctl = shutil.which("v4l2-ctl")
        self.add_on_set_parameters_callback(self._on_set_parameters)

        if self._v4l2_ctl is None:
            self.get_logger().error("v4l2-ctl not found; runtime camera tuning is unavailable")
            return

        ok, reason = self._apply_state(self._current_state())
        if ok:
            self.get_logger().info(
                "camera controls ready; tune /camera/camera_control parameters from rqt"
            )
        else:
            self.get_logger().error(f"failed to apply initial camera controls: {reason}")

    def _current_state(self) -> CameraControlState:
        return CameraControlState(
            video_device=str(self.get_parameter("video_device").value),
            autoexposure=bool(self.get_parameter("autoexposure").value),
            exposure=int(self.get_parameter("exposure").value),
            auto_white_balance=bool(self.get_parameter("auto_white_balance").value),
            white_balance=int(self.get_parameter("white_balance").value),
            gain=int(self.get_parameter("gain").value),
            brightness=int(self.get_parameter("brightness").value),
            contrast=int(self.get_parameter("contrast").value),
            saturation=int(self.get_parameter("saturation").value),
            sharpness=int(self.get_parameter("sharpness").value),
        )

    def _on_set_parameters(self, params) -> SetParametersResult:
        state = self._current_state()
        updates = state.__dict__.copy()
        changed = False

        for param in params:
            if param.name not in updates:
                continue
            updates[param.name] = param.value
            changed = True

        if not changed:
            return SetParametersResult(successful=True)

        next_state = CameraControlState(**updates)
        ok, reason = self._apply_state(next_state)
        return SetParametersResult(successful=ok, reason=reason)

    def _apply_state(self, state: CameraControlState) -> tuple[bool, str]:
        if self._v4l2_ctl is None:
            return False, "v4l2-ctl not found"

        command = build_v4l2_command(state, executable=self._v4l2_ctl)
        try:
            completed = subprocess.run(
                command,
                capture_output=True,
                text=True,
                check=False,
            )
        except OSError as exc:
            return False, str(exc)

        if completed.returncode != 0:
            stderr = completed.stderr.strip()
            stdout = completed.stdout.strip()
            reason = stderr or stdout or f"v4l2-ctl exited with code {completed.returncode}"
            self.get_logger().error(f"camera control apply failed: {reason}")
            return False, reason

        self.get_logger().info(
            "applied camera controls "
            f"(exposure={state.exposure}, white_balance={state.white_balance}, "
            f"gain={state.gain}, brightness={state.brightness}, contrast={state.contrast}, "
            f"saturation={state.saturation}, sharpness={state.sharpness})"
        )
        return True, ""


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
