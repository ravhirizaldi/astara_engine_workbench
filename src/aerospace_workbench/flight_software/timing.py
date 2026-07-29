"""Host timing modes for the flight-software bridge."""

from __future__ import annotations

import math
import time

FSW_TIMING_MODES = ("deterministic", "measured", "injected")


def validate_timing_options(
    mode: str, injected_execution_time_s: float | None
) -> None:
    if mode not in FSW_TIMING_MODES:
        raise ValueError(f"timing_mode must be one of {FSW_TIMING_MODES}")
    if mode == "injected":
        if (
            injected_execution_time_s is None
            or not math.isfinite(injected_execution_time_s)
            or injected_execution_time_s < 0.0
        ):
            raise ValueError(
                "injected timing requires a finite, nonnegative execution time"
            )
    elif injected_execution_time_s is not None:
        raise ValueError(
            "injected execution time requires timing_mode='injected'"
        )


def clock_ns() -> int:
    return time.perf_counter_ns()
