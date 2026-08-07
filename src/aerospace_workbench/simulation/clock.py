"""Monotonic simulation time."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class SimulationClock:
    truth_time_s: float = 0.0
    reset_epoch_s: float = 0.0

    def advance_to(self, truth_time_s: float) -> None:
        if not math.isfinite(truth_time_s):
            raise ValueError("simulation time must be finite")
        if truth_time_s + 1e-12 < self.truth_time_s:
            raise ValueError("simulation clock cannot move backwards")
        self.truth_time_s = truth_time_s

    def reset(self, reset_epoch_s: float) -> None:
        if not math.isfinite(reset_epoch_s):
            raise ValueError("simulation epoch must be finite")
        self.truth_time_s = reset_epoch_s
        self.reset_epoch_s = reset_epoch_s
