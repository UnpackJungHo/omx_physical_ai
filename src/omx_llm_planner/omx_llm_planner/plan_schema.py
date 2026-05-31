"""LLM plan JSON 의 파싱/검증/정규화.

ROS 의존이 전혀 없는 순수 모듈. 허용 action 과 enum 은 omx_interfaces 의
PickPlace / PickPlaceAll / MoveToNamed action 계약과 1:1 로 대응한다.
검증 실패는 모두 PlanError 로 표면화한다 (silent 금지). 모델이 모호/미지원
명령에 대해 빈 steps 또는 'unknown' action 을 내면 PlanError 로 처리되어,
상위 코드가 모델 재추론 없이 결정론적으로 실패를 반환한다.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

PICK_COLORS: tuple[str, ...] = ("red", "blue", "green")
NAMED_POSES: tuple[str, ...] = ("home", "init")

# PickPlaceAll.action 의 cap 정책과 동일: 1..10, 0/음수/과대는 default 10.
MAX_BOXES_DEFAULT = 10
MAX_BOXES_HI = 10

GRIPPER_STATES: tuple[str, ...] = ("open", "close")
ROTATE_DIRS: tuple[str, ...] = ("left", "right")
ANGLE_DEG_LO = 1
ANGLE_DEG_HI = 180


class PlanError(Exception):
    """plan 을 파싱할 수 없거나 action 스키마를 위반할 때 발생."""


@dataclass(frozen=True)
class PlanStep:
    action: str
    args: dict[str, Any]


@dataclass(frozen=True)
class Plan:
    steps: list[PlanStep]


def build_plan(raw: str | dict) -> Plan:
    """raw(JSON 문자열 또는 dict) 를 정규화된 Plan 으로 변환한다.

    실패 시 PlanError 를 던진다.
    """
    data = _load(raw)
    if not isinstance(data, dict) or "steps" not in data:
        raise PlanError("plan 에 'steps' 키가 없습니다")
    raw_steps = data["steps"]
    if not isinstance(raw_steps, list) or len(raw_steps) == 0:
        raise PlanError("plan steps 가 비어 있습니다 (명령을 이해하지 못함)")

    steps = [_build_step(i, s) for i, s in enumerate(raw_steps)]
    return Plan(steps=steps)


def _load(raw: str | dict) -> Any:
    if isinstance(raw, dict):
        return raw
    try:
        return json.loads(raw)
    except (json.JSONDecodeError, TypeError) as exc:
        raise PlanError(f"plan JSON 파싱 실패: {exc}") from exc


def _build_step(index: int, step: Any) -> PlanStep:
    if not isinstance(step, dict) or "action" not in step:
        raise PlanError(f"step[{index}] 에 'action' 이 없습니다")
    action = step["action"]
    args = step.get("args") or {}
    if not isinstance(args, dict):
        raise PlanError(f"step[{index}] args 가 dict 가 아닙니다")

    if action == "pick_place":
        return PlanStep(action, {"object_color": _enum(index, args, "object_color", PICK_COLORS)})
    if action == "move_to_named":
        return PlanStep(action, {"name": _enum(index, args, "name", NAMED_POSES)})
    if action == "pick_place_all":
        return PlanStep(action, {
            "max_boxes": _clamp_max_boxes(args.get("max_boxes")),
            "retry_on_fail": bool(args.get("retry_on_fail", False)),
        })
    if action == "gripper":
        return PlanStep(action, {"state": _enum(index, args, "state", GRIPPER_STATES)})
    if action == "rotate_base":
        return PlanStep(action, {
            "direction": _enum(index, args, "direction", ROTATE_DIRS),
            "angle_deg": _clamp_angle(args.get("angle_deg")),
        })
    raise PlanError(f"step[{index}] 미지원 action: '{action}'")


def _enum(index: int, args: dict, key: str, allowed: tuple[str, ...]) -> str:
    value = args.get(key)
    if value not in allowed:
        raise PlanError(f"step[{index}] {key}='{value}' 는 {allowed} 중 하나여야 합니다")
    return value


def _clamp_max_boxes(value: Any) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        return MAX_BOXES_DEFAULT
    return min(value, MAX_BOXES_HI)


def _clamp_angle(value: Any) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < ANGLE_DEG_LO:
        raise PlanError(f"angle_deg 가 정수(>= {ANGLE_DEG_LO}) 가 아닙니다: {value!r}")
    return min(value, ANGLE_DEG_HI)
