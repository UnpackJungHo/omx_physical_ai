"""stable_by_id_path 의 동작 검증 - 실제 /dev 없이 임시 심볼릭으로 격리 테스트."""
from __future__ import annotations

import os
from pathlib import Path

from omx_perception.camera_device import stable_by_id_path


def _make_fake_devices(tmp_path: Path) -> tuple[Path, Path]:
    """가짜 /dev (videoN) 와 /dev/v4l/by-id (심볼릭) 트리를 만든다."""
    dev = tmp_path / "dev"
    dev.mkdir()
    video2 = dev / "video2"
    video3 = dev / "video3"
    video2.write_text("")  # capture node 대역
    video3.write_text("")  # metadata node 대역

    by_id = dev / "v4l" / "by-id"
    by_id.mkdir(parents=True)
    # udev 스타일 상대 심볼릭: by-id/<name> -> ../../video2
    os.symlink(os.path.relpath(video2, by_id), by_id / "usb-Innomaker-video-index0")
    os.symlink(os.path.relpath(video3, by_id), by_id / "usb-Innomaker-video-index1")
    return video2, by_id


def test_returns_stable_alias_for_capture_node(tmp_path: Path) -> None:
    video2, by_id = _make_fake_devices(tmp_path)
    result = stable_by_id_path(video2, by_id_dir=by_id)
    assert result == str(by_id / "usb-Innomaker-video-index0")


def test_returns_none_when_no_by_id_dir(tmp_path: Path) -> None:
    video2 = tmp_path / "video2"
    video2.write_text("")
    missing = tmp_path / "does-not-exist"
    assert stable_by_id_path(video2, by_id_dir=missing) is None


def test_returns_none_when_no_alias_matches(tmp_path: Path) -> None:
    _video2, by_id = _make_fake_devices(tmp_path)
    other = tmp_path / "dev" / "video9"
    other.write_text("")
    assert stable_by_id_path(other, by_id_dir=by_id) is None
