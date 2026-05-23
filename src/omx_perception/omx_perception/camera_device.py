from __future__ import annotations

from dataclasses import dataclass
import fcntl
from pathlib import Path
import re
import struct
import subprocess
from typing import Any

import yaml


VIDIOC_QUERYCAP = 0x80685600
V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_CAP_VIDEO_CAPTURE_MPLANE = 0x00001000
V4L2_CAP_DEVICE_CAPS = 0x80000000


@dataclass(frozen=True)
class CameraDevice:
    name: str
    path: str


def resolve_video_device(
    camera_name_match: str,
    *,
    explicit_device: str = "",
    sysfs_dir: str = "/sys/class/video4linux",
) -> CameraDevice:
    explicit_device = explicit_device.strip()
    if explicit_device:
        explicit_path = Path(explicit_device)
        if not explicit_path.exists():
            raise RuntimeError(f"Configured video_device does not exist: {explicit_device}")
        if not _is_capture_device(explicit_path):
            raise RuntimeError(
                f"Configured video_device is not a V4L2 capture device: {explicit_device}"
            )
        return CameraDevice(name="explicit launch argument", path=explicit_device)

    camera_name_match = camera_name_match.strip()
    if not camera_name_match:
        raise RuntimeError("camera_name_match must not be empty when video_device is not set")

    root = Path(sysfs_dir.strip())
    matches = _matching_video_devices(camera_name_match, root)
    if matches:
        return matches[0]

    available = []
    if root.exists():
        for video_dir in sorted(root.glob("video*"), key=_video_device_number):
            video_device = Path("/dev") / video_dir.name
            available.append(f"{video_device}: {_read_video_name(video_dir)}")

    raise RuntimeError(
        f"No capture-capable video device matched camera_name_match={camera_name_match!r}. "
        f"Available video devices: {available}"
    )


def load_camera_parameters(yaml_path: str) -> dict[str, Any]:
    data = yaml.safe_load(Path(yaml_path).read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        return {}

    for node_params in data.values():
        if isinstance(node_params, dict) and isinstance(node_params.get("ros__parameters"), dict):
            return node_params["ros__parameters"]

    return {}


def apply_v4l2_controls(video_device: str, parameters: dict[str, Any]) -> list[str]:
    controls: list[tuple[str, int]] = []

    for param_name in ("brightness", "contrast", "saturation", "sharpness", "gain"):
        value = parameters.get(param_name)
        if isinstance(value, int) and value >= 0:
            controls.append((param_name, value))

    auto_white_balance = parameters.get("auto_white_balance")
    if isinstance(auto_white_balance, bool):
        controls.append(("white_balance_automatic", int(auto_white_balance)))
        white_balance = parameters.get("white_balance")
        if not auto_white_balance and isinstance(white_balance, int) and white_balance > 0:
            controls.append(("white_balance_temperature", white_balance))

    autoexposure = parameters.get("autoexposure")
    if isinstance(autoexposure, bool):
        controls.append(("auto_exposure", 3 if autoexposure else 1))
        exposure = parameters.get("exposure")
        if not autoexposure and isinstance(exposure, int) and exposure > 0:
            controls.append(("exposure_time_absolute", exposure))

    applied: list[str] = []
    for control_name, control_value in controls:
        control = f"{control_name}={control_value}"
        try:
            subprocess.run(
                ["v4l2-ctl", "-d", video_device, f"--set-ctrl={control}"],
                check=True,
                capture_output=True,
                text=True,
            )
        except FileNotFoundError as exc:
            raise RuntimeError("v4l2-ctl is required to apply Innomaker camera controls") from exc
        except subprocess.CalledProcessError as exc:
            detail = (exc.stderr or exc.stdout or str(exc)).strip()
            raise RuntimeError(f"Failed to apply V4L2 control {control}: {detail}") from exc
        applied.append(control)

    return applied


def _matching_video_devices(match_text: str, sysfs_dir: Path) -> list[CameraDevice]:
    match = match_text.casefold()
    matches: list[CameraDevice] = []

    for video_dir in sorted(sysfs_dir.glob("video*"), key=_video_device_number):
        camera_name = _read_video_name(video_dir)
        if match not in camera_name.casefold():
            continue

        video_device = Path("/dev") / video_dir.name
        if video_device.exists() and _is_capture_device(video_device):
            matches.append(CameraDevice(name=camera_name, path=str(video_device)))

    return matches


def _video_device_number(path: Path) -> int:
    match = re.fullmatch(r"video(\d+)", path.name)
    return int(match.group(1)) if match else 10**9


def _read_video_name(video_dir: Path) -> str:
    try:
        return (video_dir / "name").read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def _is_capture_device(video_device: Path) -> bool:
    querycap = bytearray(104)
    try:
        with video_device.open("rb", buffering=0) as device_file:
            fcntl.ioctl(device_file, VIDIOC_QUERYCAP, querycap, True)
    except OSError:
        return False

    capabilities, device_caps = struct.unpack_from("II", querycap, 80)
    active_caps = device_caps if capabilities & V4L2_CAP_DEVICE_CAPS else capabilities
    capture_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE
    return bool(active_caps & capture_caps)
