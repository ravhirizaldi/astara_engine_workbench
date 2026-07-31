"""Deterministic discrete-event scheduling for simulated avionics."""

from __future__ import annotations

import heapq
import math
import random
from dataclasses import dataclass, field
from typing import Any, Mapping


@dataclass(frozen=True)
class TimingProfile:
    sample_rate_hz: float
    clock_offset_s: float
    drift_ppm: float
    jitter_s: float
    processing_delay_s: float
    publication_delay_s: float
    deadline_s: float
    drop_on_deadline_miss: bool
    phase_offset_s: float
    reset_epoch_s: float

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> TimingProfile:
        return cls(
            sample_rate_hz=float(values["sample_rate_hz"]),
            clock_offset_s=float(values["clock_offset_s"]),
            drift_ppm=float(values["drift_ppm"]),
            jitter_s=float(values["jitter_s"]),
            processing_delay_s=float(values["processing_delay_s"]),
            publication_delay_s=float(values["publication_delay_s"]),
            deadline_s=float(values["deadline_s"]),
            drop_on_deadline_miss=bool(values["drop_on_deadline_miss"]),
            phase_offset_s=float(values["phase_offset_s"]),
            reset_epoch_s=float(values["reset_epoch_s"]),
        )


@dataclass
class SimulationClock:
    truth_time_s: float = 0.0
    reset_epoch_s: float = 0.0

    def advance_to(self, truth_time_s: float) -> None:
        if truth_time_s + 1e-12 < self.truth_time_s:
            raise ValueError("simulation clock cannot move backwards")
        self.truth_time_s = truth_time_s

    def reset(self, reset_epoch_s: float) -> None:
        self.truth_time_s = reset_epoch_s
        self.reset_epoch_s = reset_epoch_s


@dataclass(order=True)
class ScheduledEvent:
    truth_time_s: float
    priority: int
    sequence: int
    kind: str = field(compare=False)
    subsystem: str = field(compare=False)
    payload: dict[str, Any] = field(compare=False, default_factory=dict)


class EventQueue:
    def __init__(self) -> None:
        self._events: list[ScheduledEvent] = []
        self._sequence = 0

    def schedule(
        self,
        truth_time_s: float,
        priority: int,
        kind: str,
        subsystem: str,
        payload: dict[str, Any] | None = None,
    ) -> ScheduledEvent:
        event = ScheduledEvent(
            truth_time_s,
            priority,
            self._sequence,
            kind,
            subsystem,
            payload or {},
        )
        self._sequence += 1
        heapq.heappush(self._events, event)
        return event

    def pop_due(self, truth_time_s: float) -> ScheduledEvent | None:
        if not self._events or self._events[0].truth_time_s > truth_time_s + 1e-12:
            return None
        return heapq.heappop(self._events)


@dataclass(frozen=True)
class ClockTick:
    time_s: float
    dt_s: float


class _SampleClock:
    def __init__(self, profile: TimingProfile, seed: int) -> None:
        self.profile = profile
        self._rng = random.Random(seed)
        self._index = 0
        self._last_time_s: float | None = None

    def next_tick(self, not_before_s: float = -math.inf) -> ClockTick:
        rate_scale = 1.0 + self.profile.drift_ppm * 1e-6
        period_s = 1.0 / self.profile.sample_rate_hz
        while True:
            local_s = self.profile.phase_offset_s + self._index * period_s
            time_s = self.profile.reset_epoch_s + (
                local_s - self.profile.clock_offset_s
            ) / rate_scale
            if self.profile.jitter_s:
                time_s += self._rng.gauss(0.0, self.profile.jitter_s)
            self._index += 1
            if self._last_time_s is not None:
                time_s = max(time_s, self._last_time_s + 1e-12)
            if time_s + 1e-12 >= not_before_s:
                break
        dt_s = (
            time_s - self._last_time_s
            if self._last_time_s is not None
            else period_s / rate_scale
        )
        self._last_time_s = time_s
        return ClockTick(time_s, dt_s)


class _PeriodicScheduler:
    def __init__(self, queue: EventQueue, event_kind: str, priority: int) -> None:
        self.queue = queue
        self.event_kind = event_kind
        self.priority = priority
        self.profiles: dict[str, TimingProfile] = {}
        self.sample_clocks: dict[str, _SampleClock] = {}
        self.dropped_deadlines: dict[str, int] = {}

    def add(self, name: str, profile: TimingProfile, seed: int) -> None:
        self.profiles[name] = profile
        self.sample_clocks[name] = _SampleClock(profile, seed)
        self.dropped_deadlines[name] = 0

    def start(self, name: str, not_before_s: float) -> None:
        self._schedule_next(name, not_before_s)

    def _schedule_next(self, name: str, not_before_s: float = -math.inf) -> None:
        tick = self.sample_clocks[name].next_tick(not_before_s)
        self.queue.schedule(
            tick.time_s,
            self.priority,
            self.event_kind,
            name,
            {"tick": tick},
        )

    def released(self, event: ScheduledEvent) -> None:
        self._schedule_next(event.subsystem)


class DeviceScheduler(_PeriodicScheduler):
    def __init__(self, queue: EventQueue) -> None:
        super().__init__(queue, "device_sample", 0)

    def complete(
        self, event: ScheduledEvent, payload: dict[str, Any]
    ) -> ScheduledEvent | None:
        profile = self.profiles[event.subsystem]
        completion_s = event.truth_time_s + profile.processing_delay_s
        if completion_s > event.truth_time_s + profile.deadline_s + 1e-12:
            self.dropped_deadlines[event.subsystem] += 1
            if profile.drop_on_deadline_miss:
                return None
        payload["sensor_completion_time_s"] = completion_s
        return self.queue.schedule(
            completion_s,
            1,
            "device_complete",
            event.subsystem,
            payload,
        )


class TaskScheduler(_PeriodicScheduler):
    def __init__(self, queue: EventQueue) -> None:
        super().__init__(queue, "task_release", 4)

    def complete(
        self, event: ScheduledEvent, payload: dict[str, Any]
    ) -> ScheduledEvent | None:
        profile = self.profiles[event.subsystem]
        completion_s = event.truth_time_s + profile.processing_delay_s
        deadline_missed = (
            completion_s + profile.publication_delay_s
            > event.truth_time_s + profile.deadline_s + 1e-12
        )
        if deadline_missed:
            self.dropped_deadlines[event.subsystem] += 1
            if profile.drop_on_deadline_miss:
                return None
        payload["deadline_missed"] = deadline_missed
        payload["task_release_time_s"] = event.truth_time_s
        return self.queue.schedule(
            completion_s,
            5,
            "task_complete",
            event.subsystem,
            payload,
        )

    def publish(
        self, event: ScheduledEvent, payload: dict[str, Any]
    ) -> ScheduledEvent:
        publish_s = (
            event.truth_time_s
            + self.profiles[event.subsystem].publication_delay_s
        )
        return self.queue.schedule(
            publish_s,
            6,
            "task_publish",
            event.subsystem,
            payload,
        )


class BusScheduler:
    def __init__(
        self,
        queue: EventQueue,
        name: str,
        profile: TimingProfile,
        seed: int,
    ) -> None:
        self.queue = queue
        self.name = name
        self.profile = profile
        self.sample_clock = _SampleClock(profile, seed)
        self.dropped_deadlines = 0

    def submit(
        self,
        event: ScheduledEvent,
        publication_delay_s: float,
    ) -> ScheduledEvent | None:
        ready_s = event.truth_time_s + publication_delay_s
        slot = self.sample_clock.next_tick(ready_s)
        publish_s = slot.time_s + self.profile.processing_delay_s
        receive_s = publish_s + self.profile.publication_delay_s
        if receive_s > ready_s + self.profile.deadline_s + 1e-12:
            self.dropped_deadlines += 1
            if self.profile.drop_on_deadline_miss:
                return None
        payload = event.payload
        payload["bus_publish_time_s"] = publish_s
        payload["fsw_receive_time_s"] = receive_s
        return self.queue.schedule(
            publish_s,
            2,
            "bus_publish",
            self.name,
            payload,
        )

    def published(self, event: ScheduledEvent) -> ScheduledEvent:
        return self.queue.schedule(
            float(event.payload["fsw_receive_time_s"]),
            3,
            "bus_receive",
            self.name,
            event.payload,
        )
