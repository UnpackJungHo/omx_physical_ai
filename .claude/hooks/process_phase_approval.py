#!/usr/bin/env python3

from __future__ import annotations

import json
import sys

from phase_workflow import approve_phase, load_state, parse_approval_prompt


def main() -> int:
    payload = json.load(sys.stdin)
    prompt = payload.get("prompt", "")

    state = load_state()
    pending = state.get("pending_approval")
    fallback_phase = pending["phase"] if pending else None
    approved, phase_id = parse_approval_prompt(prompt, fallback_phase=fallback_phase)

    if not approved:
        return 0

    ok, detail = approve_phase(state, phase_id)
    if not ok:
        return 0

    print(
        json.dumps(
            {
                "hookSpecificOutput": {
                    "hookEventName": "UserPromptSubmit",
                    "additionalContext": (
                        f"Hook가 Phase {detail} 완료 승인을 처리했다. "
                        "`.claude/state/project_state.json`, `CLAUDE.md`, "
                        "`docs/SESSION_NEXT.md`, `docs/STATUS.md`를 갱신했다."
                    ),
                }
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
