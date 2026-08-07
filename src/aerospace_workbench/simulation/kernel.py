"""Single-queue deterministic simulation kernel."""

from __future__ import annotations

from collections.abc import Callable
from enum import IntEnum

from .clock import SimulationClock
from .event_queue import EventQueue, ScheduledEvent


class EventPriority(IntEnum):
    TIMELINE = 10
    DEVICE_SAMPLE = 20
    DEVICE_COMPLETE = 21
    BUS_PUBLISH = 22
    BUS_RECEIVE = 23
    TASK_RELEASE = 30
    TASK_COMPLETE = 31
    TASK_PUBLISH = 32
    ACTUATION = 40
    TRUTH = 50
    TRUTH_FACT = 60
    EVIDENCE = 70
    LIFECYCLE = 80


EventHandler = Callable[[ScheduledEvent], None]


class SimulationKernel:
    def __init__(
        self,
        clock: SimulationClock | None = None,
        queue: EventQueue | None = None,
    ) -> None:
        self.clock = clock or SimulationClock()
        self.queue = queue if queue is not None else EventQueue()
        self._handlers: dict[str, EventHandler] = {}
        self.stopped = False

    def register(self, kind: str, handler: EventHandler) -> None:
        if kind in self._handlers:
            raise ValueError(f"event handler already registered for {kind!r}")
        self._handlers[kind] = handler

    def schedule(
        self,
        truth_time_s: float,
        priority: int,
        kind: str,
        subsystem: str = "",
        payload: dict | None = None,
        owner: str = "",
    ) -> ScheduledEvent:
        if truth_time_s + 1e-12 < self.clock.truth_time_s:
            raise ValueError("cannot schedule an event in the past")
        return self.queue.schedule(
            truth_time_s,
            priority,
            kind,
            subsystem,
            payload,
            owner,
        )

    def cancel_owner(self, owner: str) -> None:
        self.queue.cancel_owner(owner)

    def stop(self) -> None:
        self.stopped = True

    def run(
        self,
        until_s: float,
        should_cancel: Callable[[], bool] | None = None,
    ) -> None:
        while not self.stopped and self.queue:
            if self.queue.peek_time() > until_s + 1e-12:
                break
            if should_cancel is not None and should_cancel():
                break
            event = self.queue.pop()
            if event is None:
                break
            self.clock.advance_to(event.truth_time_s)
            try:
                handler = self._handlers[event.kind]
            except KeyError as error:
                raise ValueError(
                    f"no handler registered for event kind {event.kind!r}"
                ) from error
            handler(event)
