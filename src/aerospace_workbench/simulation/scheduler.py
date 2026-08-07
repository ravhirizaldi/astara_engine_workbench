"""Avionics timing profiles and periodic schedulers."""

from __future__ import annotations

import math
import random
from collections import defaultdict
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Mapping

from .event_queue import EventQueue, ScheduledEvent
from .kernel import EventPriority
from .kernel import SimulationKernel


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
    def from_mapping(cls, values: Mapping[str, Any]) -> "TimingProfile":
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
    def __init__(
        self,
        queue: EventQueue,
        event_kind: str,
        priority: int,
        owner: str = "",
    ) -> None:
        self.queue = queue
        self.event_kind = event_kind
        self.priority = priority
        self.owner = owner
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
            self.owner,
        )

    def released(self, event: ScheduledEvent) -> None:
        self._schedule_next(event.subsystem)


class DeviceScheduler(_PeriodicScheduler):
    def __init__(self, queue: EventQueue, owner: str = "") -> None:
        super().__init__(
            queue,
            "device_sample",
            EventPriority.DEVICE_SAMPLE,
            owner,
        )

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
            EventPriority.DEVICE_COMPLETE,
            "device_complete",
            event.subsystem,
            payload,
            self.owner,
        )


class TaskScheduler(_PeriodicScheduler):
    def __init__(self, queue: EventQueue, owner: str = "") -> None:
        super().__init__(
            queue,
            "task_release",
            EventPriority.TASK_RELEASE,
            owner,
        )

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
            EventPriority.TASK_COMPLETE,
            "task_complete",
            event.subsystem,
            payload,
            self.owner,
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
            EventPriority.TASK_PUBLISH,
            "task_publish",
            event.subsystem,
            payload,
            self.owner,
        )


class BusScheduler:
    def __init__(
        self,
        queue: EventQueue,
        name: str,
        profile: TimingProfile,
        seed: int,
        owner: str = "",
    ) -> None:
        self.queue = queue
        self.name = name
        self.profile = profile
        self.owner = owner
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
            EventPriority.BUS_PUBLISH,
            "bus_publish",
            self.name,
            payload,
            self.owner,
        )

    def published(self, event: ScheduledEvent) -> ScheduledEvent:
        return self.queue.schedule(
            float(event.payload["fsw_receive_time_s"]),
            EventPriority.BUS_RECEIVE,
            "bus_receive",
            self.name,
            event.payload,
            self.owner,
        )


TimelineHandler = Callable[[dict[str, Any], ScheduledEvent], None]


class MissionScheduler:
    """Route typed timeline sources through the global event queue."""

    EVENT_KIND = "timeline_dispatch"

    def __init__(
        self,
        kernel: SimulationKernel,
        timeline: list[dict[str, Any]],
        handler: TimelineHandler,
    ) -> None:
        self.kernel = kernel
        self.timeline = timeline
        self.handler = handler
        self.by_source: dict[tuple[str, str], list[dict[str, Any]]] = (
            defaultdict(list)
        )
        self.after: dict[str, list[dict[str, Any]]] = defaultdict(list)
        self.fired: set[tuple[str, str]] = set()
        for entry in timeline:
            trigger = entry["trigger"]
            trigger_type = str(trigger["type"])
            if trigger_type == "fsw_fact":
                self.by_source[(trigger_type, str(trigger["fact"]))].append(
                    entry
                )
            elif trigger_type == "truth_detector":
                self.by_source[
                    (trigger_type, str(trigger["detector"]))
                ].append(entry)
            elif trigger_type == "after_event":
                self.after[str(trigger["event"])].append(entry)
        kernel.register(self.EVENT_KIND, self._handle)

    def start(self) -> None:
        for entry in self.timeline:
            trigger = entry["trigger"]
            if trigger["type"] != "time":
                continue
            target = str(entry["action"].get("target", "all"))
            self._schedule(
                entry,
                float(trigger["at_s"]),
                target,
                "timeline",
                {},
            )

    def publish(
        self,
        source_type: str,
        name: str,
        detected_time_s: float,
        body: str,
        payload: dict[str, Any] | None = None,
        effective_time_s: float | None = None,
    ) -> None:
        for entry in self.by_source.get((source_type, name), ()):
            self._schedule(
                entry,
                detected_time_s,
                body,
                "fsw" if source_type == "fsw_fact" else "truth",
                payload or {},
                effective_time_s,
            )

    def _schedule(
        self,
        entry: dict[str, Any],
        detected_time_s: float,
        body: str,
        source: str,
        payload: dict[str, Any],
        effective_time_s: float | None = None,
    ) -> None:
        self.kernel.schedule(
            detected_time_s,
            EventPriority.ACTUATION,
            self.EVENT_KIND,
            str(entry["id"]),
            {
                "entry": entry,
                "body": body,
                "source": source,
                "detail": payload,
                "effective_time_s": (
                    detected_time_s
                    if effective_time_s is None
                    else effective_time_s
                ),
            },
            f"timeline:{entry['id']}:{body}",
        )

    def _handle(self, event: ScheduledEvent) -> None:
        entry = event.payload["entry"]
        body = str(event.payload["body"])
        key = (str(entry["id"]), body)
        if key in self.fired:
            return
        self.fired.add(key)
        self.handler(entry, event)
        for dependent in self.after.get(str(entry["id"]), ()):
            trigger = dependent["trigger"]
            self._schedule(
                dependent,
                event.truth_time_s + float(trigger["delay_s"]),
                body,
                "timeline",
                {},
            )
