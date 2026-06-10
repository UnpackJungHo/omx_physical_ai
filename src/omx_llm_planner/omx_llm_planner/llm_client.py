"""LLM 클라이언트 추상화.

테스트는 MockLLMClient 를, 런타임은 OllamaLLMClient 를 주입한다. 두 구현 모두
generate_plan(command) -> Plan 을 제공한다. 출력 형식은 plan_schema.build_plan
으로 검증/정규화하여 항상 정규화된 Plan 또는 예외만 흘려보낸다.
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Protocol

from omx_llm_planner.plan_schema import Plan, build_plan


class LLMUnavailable(Exception):
    """LLM 엔드포인트 미연결/timeout/응답 없음."""


class LLMClient(Protocol):
    def generate_plan(self, command: str) -> Plan:
        """자연어 명령을 정규화된 Plan 으로 변환. 실패 시 LLMUnavailable/PlanError."""
        ...


class MockLLMClient:
    """command -> raw plan JSON 매핑 기반 테스트용 클라이언트."""

    def __init__(self, responses: dict[str, str]) -> None:
        self._responses = responses

    def generate_plan(self, command: str) -> Plan:
        if command not in self._responses:
            raise LLMUnavailable(f"mock 에 등록되지 않은 명령: {command!r}")
        return build_plan(self._responses[command])


class OllamaLLMClient:
    """Ollama /api/chat (format=json) 기반 런타임 클라이언트.

    bounded retry(max_retries) 후 실패하면 LLMUnavailable 를 던진다.
    무제한 retry/blocking 금지.
    """

    # -> 는 이 함수가 무엇을 반환하는지 알려주는 타입 힌트
    # -> None은 반환값이 없다는 의미
    def __init__(
        self,
        endpoint: str,
        model_name: str,
        system_prompt: str,
        request_timeout_sec: float,
        max_retries: int,
    ) -> None:
        self._endpoint = endpoint.rstrip("/")
        self._model_name = model_name
        self._system_prompt = system_prompt
        self._timeout = request_timeout_sec
        self._max_retries = max(0, max_retries)

    # 사용자가 omx_web_ws에서 보낸 채팅의 request를 처리해서 build_plan으로 넘겨주는 함수
    def generate_plan(self, command: str) -> Plan:
        payload = json.dumps({
            "model": self._model_name,
            "format": "json",
            "stream": False,
            "messages": [
                {"role": "system", "content": self._system_prompt},
                {"role": "user", "content": command},
            ],
        }).encode("utf-8")

        last_err: Exception | None = None
        for _ in range(self._max_retries + 1):
            try:
                req = urllib.request.Request(
                    f"{self._endpoint}/api/chat",
                    data=payload,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                    body = json.loads(resp.read().decode("utf-8"))
                content = body.get("message", {}).get("content", "")
                return build_plan(content)
            except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
                last_err = exc
        raise LLMUnavailable(f"Ollama 호출 실패: {last_err}")
