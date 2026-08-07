"""Stable priority queue for simulation events."""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Any


@dataclass(order=True)
class ScheduledEvent:
    truth_time_s: float
    priority: int
    sequence: int
    kind: str = field(compare=False)
    subsystem: str = field(compare=False)
    payload: dict[str, Any] = field(compare=False, default_factory=dict)
    owner: str = field(compare=False, default="")


class EventQueue:
    def __init__(self) -> None:
        self._events: list[ScheduledEvent] = []
        self._sequence = 0
        self._cancelled_owners: set[str] = set()

    def __bool__(self) -> bool:
        self._discard_cancelled()
        return bool(self._events)

    def schedule(
        self,
        truth_time_s: float,
        priority: int,
        kind: str,
        subsystem: str,
        payload: dict[str, Any] | None = None,
        owner: str = "",
    ) -> ScheduledEvent:
        if not math.isfinite(truth_time_s):
            raise ValueError("event time must be finite")
        event = ScheduledEvent(
            truth_time_s,
            priority,
            self._sequence,
            kind,
            subsystem,
            payload or {},
            owner,
        )
        self._sequence += 1
        heapq.heappush(self._events, event)
        return event

    def cancel_owner(self, owner: str) -> None:
        self._cancelled_owners.add(owner)

    def pop(self) -> ScheduledEvent | None:
        self._discard_cancelled()
        return heapq.heappop(self._events) if self._events else None

    def peek_time(self) -> float | None:
        self._discard_cancelled()
        return self._events[0].truth_time_s if self._events else None

    def pop_due(self, truth_time_s: float) -> ScheduledEvent | None:
        self._discard_cancelled()
        if not self._events or self._events[0].truth_time_s > truth_time_s + 1e-12:
            return None
        return heapq.heappop(self._events)

    def _discard_cancelled(self) -> None:
        while self._events and self._events[0].owner in self._cancelled_owners:
            heapq.heappop(self._events)
