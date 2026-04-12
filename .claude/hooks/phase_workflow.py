#!/usr/bin/env python3

from __future__ import annotations

import json
import os
import re
from datetime import datetime, timezone, timedelta
from pathlib import Path
from typing import Any


KST = timezone(timedelta(hours=9))
DEFAULT_PHASES = [
    {
        "id": "A",
        "title": "레퍼런스 코드 읽기",
        "summary": "URDF, ros2_control, launch, MoveIt2 config 검토",
        "status": "done",
    },
    {
        "id": "B",
        "title": "로봇 모델 표시",
        "summary": "display_robot.launch.py 작성 및 RViz 표시 확인",
        "status": "done",
    },
    {
        "id": "C",
        "title": "ros2_control 붙이기 (mock 모드)",
        "summary": "controller_manager, spawner, 초기 포즈 연결",
        "status": "in_progress",
    },
    {
        "id": "D",
        "title": "MoveIt2 붙이기",
        "summary": "move_group, MotionPlanning, workspace 제약 연결",
        "status": "pending",
    },
    {
        "id": "E",
        "title": "통합 launch 파일 정리",
        "summary": "omx_demo.launch.py 통합",
        "status": "pending",
    },
]


def project_root() -> Path:
    env_root = os.environ.get("CLAUDE_PROJECT_DIR")
    if env_root:
        return Path(env_root)
    return Path(__file__).resolve().parents[2]


ROOT = project_root()
STATE_PATH = ROOT / ".claude" / "state" / "project_state.json"
CLAUDE_MD = ROOT / "CLAUDE.md"
SESSION_NEXT_MD = ROOT / "docs" / "SESSION_NEXT.md"
STATUS_MD = ROOT / "docs" / "STATUS.md"


def now_iso() -> str:
    return datetime.now(KST).isoformat(timespec="seconds")


def bootstrap_state() -> dict[str, Any]:
    return {
        "phases": DEFAULT_PHASES,
        "pending_approval": None,
        "history": [
            {
                "timestamp": now_iso(),
                "event": "bootstrap",
                "message": "초기 상태 생성",
            }
        ],
    }


def load_state() -> dict[str, Any]:
    if not STATE_PATH.exists():
        state = bootstrap_state()
        save_state(state)
        return state

    with STATE_PATH.open("r", encoding="utf-8") as f:
        return json.load(f)


def save_state(state: dict[str, Any]) -> None:
    STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
    with STATE_PATH.open("w", encoding="utf-8") as f:
        json.dump(state, f, ensure_ascii=False, indent=2)
        f.write("\n")


def find_phase(state: dict[str, Any], phase_id: str) -> dict[str, Any] | None:
    for phase in state["phases"]:
        if phase["id"] == phase_id:
            return phase
    return None


def current_phase(state: dict[str, Any]) -> dict[str, Any] | None:
    for phase in state["phases"]:
        if phase["status"] == "in_progress":
            return phase
    for phase in state["phases"]:
        if phase["status"] == "pending":
            return phase
    return None


def pending_label(state: dict[str, Any]) -> str:
    pending = state.get("pending_approval")
    if not pending:
        return "없음"
    return f"Phase {pending['phase']} 완료 승인 대기"


def status_label(status: str) -> str:
    return {
        "done": "완료",
        "in_progress": "진행 중",
        "pending": "대기",
    }[status]


def status_token(status: str) -> str:
    return {
        "done": "done",
        "in_progress": "in_progress",
        "pending": "pending",
    }[status]


def request_phase_completion(state: dict[str, Any], phase_id: str) -> bool:
    phase = find_phase(state, phase_id)
    if not phase:
        return False
    if phase["status"] != "in_progress":
        return False

    state["pending_approval"] = {
        "phase": phase_id,
        "requested_at": now_iso(),
    }
    state.setdefault("history", []).append(
        {
            "timestamp": now_iso(),
            "event": "completion_requested",
            "message": f"Phase {phase_id} 완료 승인 요청 등록",
        }
    )
    save_state(state)
    render_all(state)
    return True


def approve_phase(state: dict[str, Any], phase_id: str | None = None) -> tuple[bool, str]:
    pending = state.get("pending_approval")
    if not pending:
        return False, "승인 대기 중인 Phase가 없음"

    pending_phase_id = pending["phase"]
    if phase_id and phase_id != pending_phase_id:
        return False, f"승인 대기 Phase는 {pending_phase_id}인데, 입력은 {phase_id}"

    phase = find_phase(state, pending_phase_id)
    if not phase:
        return False, f"Phase {pending_phase_id}를 찾지 못함"

    phase["status"] = "done"

    in_progress_exists = any(p["status"] == "in_progress" for p in state["phases"])
    if not in_progress_exists:
        activate_next_pending(state, pending_phase_id)
    else:
        for p in state["phases"]:
            if p["id"] != pending_phase_id and p["status"] == "in_progress":
                break
        else:
            activate_next_pending(state, pending_phase_id)

    state["pending_approval"] = None
    state.setdefault("history", []).append(
        {
            "timestamp": now_iso(),
            "event": "completion_approved",
            "message": f"Phase {pending_phase_id} 완료 승인 및 문서 갱신",
        }
    )
    save_state(state)
    render_all(state)
    return True, pending_phase_id


def activate_next_pending(state: dict[str, Any], completed_phase_id: str) -> None:
    seen_completed = False
    for phase in state["phases"]:
        if phase["id"] == completed_phase_id:
            seen_completed = True
            continue
        if seen_completed and phase["status"] == "pending":
            phase["status"] = "in_progress"
            return


def parse_phase_request(message: str) -> str | None:
    match = re.search(r"\[PHASE_COMPLETE_REQUEST:\s*([A-Z])\s*\]", message or "")
    if not match:
        return None
    return match.group(1)


def parse_approval_prompt(prompt: str, fallback_phase: str | None = None) -> tuple[bool, str | None]:
    text = prompt.strip()
    upper = text.upper()
    lower = text.lower()

    reject_words = (
        "아직",
        "보류",
        "반려",
        "아니",
        "안 됐",
        "안됐",
        "안 끝",
        "미완료",
        "not yet",
        "hold",
    )
    if any(word in text or word in lower for word in reject_words):
        return False, None

    explicit = re.search(r"승인:\s*([A-Z])\s*완료", upper)
    if explicit:
        return True, explicit.group(1)

    phase_match = re.search(r"\b([A-Z])\b", upper)
    phase_id = phase_match.group(1) if phase_match else fallback_phase

    approval_words = ("승인", "오케이", "동의", "맞아", "완료", "끝난", "끝난거", "끝난 것", "complete", "approved")
    if phase_id and any(word in lower or word in text for word in approval_words):
        return True, phase_id

    return False, None


def render_all(state: dict[str, Any]) -> None:
    render_status_md(state)
    update_managed_block(CLAUDE_MD, "AUTO_PHASE_STATUS", render_claude_block(state))
    update_managed_block(SESSION_NEXT_MD, "AUTO_PHASE_SUMMARY", render_session_block(state))


def render_claude_block(state: dict[str, Any]) -> str:
    current = current_phase(state)
    if current:
        current_line = (
            f"- 현재 구현 우선순위는 `omx_bringup`의 Phase {current['id']}, "
            f"즉 `{current['title']}`다."
        )
    else:
        current_line = "- 현재 구현 우선순위는 없음. 모든 등록 Phase가 완료된 상태다."

    tokens = ", ".join(
        f"`{phase['id']} {status_token(phase['status'])}`" for phase in state["phases"]
    )
    lines = [
        current_line,
        f"- 단계 현황: {tokens}",
        f"- 승인 대기: {pending_label(state)}",
        "- 세부 상태 보드는 `docs/STATUS.md`, 실행 컨텍스트는 `docs/SESSION_NEXT.md`를 기준으로 본다.",
    ]
    return "\n".join(lines)


def render_session_block(state: dict[str, Any]) -> str:
    current = current_phase(state)
    lines = ["## 단계 상태"]
    for phase in state["phases"]:
        lines.append(
            f"- `{phase['id']}` {status_label(phase['status'])} — {phase['title']}"
        )
    lines.append("")
    lines.append("## 현재 작업 단계")
    if current:
        lines.append(f"- Phase {current['id']}, `{current['title']}`")
    else:
        lines.append("- 현재 활성 Phase 없음")
    lines.append("")
    lines.append("## 승인 대기")
    lines.append(f"- {pending_label(state)}")
    return "\n".join(lines)


def render_status_md(state: dict[str, Any]) -> None:
    current = current_phase(state)
    history = state.get("history", [])[-10:]
    lines = [
        "# OMX 상태 보드",
        "",
        "이 문서는 Hook이 자동 갱신한다. 수동 편집하지 않는다.",
        "",
        "## 단계 상태",
    ]
    for phase in state["phases"]:
        lines.append(
            f"- `{phase['id']}` {status_label(phase['status'])} — {phase['title']}"
        )
    lines.extend(["", "## 현재 작업 단계"])
    if current:
        lines.append(f"- Phase {current['id']}, `{current['title']}`")
    else:
        lines.append("- 현재 활성 Phase 없음")
    lines.extend(["", "## 승인 대기", f"- {pending_label(state)}", "", "## 최근 이력"])
    if history:
        for item in history:
            lines.append(f"- {item['timestamp']} | {item['message']}")
    else:
        lines.append("- 이력 없음")
    STATUS_MD.write_text("\n".join(lines) + "\n", encoding="utf-8")


def update_managed_block(path: Path, marker: str, new_body: str) -> None:
    text = path.read_text(encoding="utf-8")
    start = f"<!-- {marker}_START -->"
    end = f"<!-- {marker}_END -->"
    pattern = re.compile(
        rf"{re.escape(start)}.*?{re.escape(end)}",
        flags=re.DOTALL,
    )
    replacement = f"{start}\n{new_body}\n{end}"
    updated = pattern.sub(replacement, text, count=1)
    if updated == text:
        raise RuntimeError(f"{path}에서 관리 블록 {marker}를 찾지 못함")
    path.write_text(updated, encoding="utf-8")
