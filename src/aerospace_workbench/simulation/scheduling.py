"""Compatibility imports for the split scheduling modules."""

from .clock import SimulationClock
from .event_queue import EventQueue, ScheduledEvent
from .scheduler import (
    BusScheduler,
    ClockTick,
    DeviceScheduler,
    TaskScheduler,
    TimingProfile,
)

__all__ = (
    "BusScheduler",
    "ClockTick",
    "DeviceScheduler",
    "EventQueue",
    "ScheduledEvent",
    "SimulationClock",
    "TaskScheduler",
    "TimingProfile",
)
