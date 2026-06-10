"""최신 /joint_states 를 캐시하고 joint 별 위치를 stale 검사와 함께 제공한다.

ROS 구독 콜백은 update() 만 호출하고, 순수 로직은 테스트 가능하도록 분리한다.
max_age_sec 보다 오래된 데이터는 None(추정 금지)으로 처리한다.
"""
from __future__ import annotations

import threading
from typing import Callable, Optional

from sensor_msgs.msg import JointState


class JointStateCache:
    # Callable 함수 타입을 의미함,
    def __init__(self, max_age_sec: float, clock_now: Callable[[], float]) -> None:
        self._max_age = max_age_sec
        self._now = clock_now
        self._lock = threading.Lock()
        self._pos: dict[str, float] = {}
        self._stamp: Optional[float] = None

    # get, update가 동시에 쓰일 수 없도록 Lock()으로 제어
    def update(self, msg: JointState, stamp: float) -> None:
        with self._lock:
            self._pos = {n: p for n, p in zip(msg.name, msg.position)}
            self._stamp = stamp

    def get(self, joint: str) -> Optional[float]:
        with self._lock:
            if self._stamp is None or (self._now() - self._stamp) > self._max_age:
                return None
            return self._pos.get(joint)
