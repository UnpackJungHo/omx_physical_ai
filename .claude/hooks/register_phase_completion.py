#!/usr/bin/env python3

from __future__ import annotations

import json
import sys

from phase_workflow import load_state, parse_phase_request, request_phase_completion


def main() -> int:
    payload = json.load(sys.stdin)
    message = payload.get("last_assistant_message", "")
    phase_id = parse_phase_request(message)
    if not phase_id:
        return 0

    state = load_state()
    request_phase_completion(state, phase_id)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
