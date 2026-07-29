"""Background simulation process used by the live mission view."""

from __future__ import annotations

import queue
import time

from ...simulation.runner import run_simulation


def run_simulation_process(
    scenario: dict,
    seed: int,
    messages,
    cancel_event,
    pause_event,
    speed_factor,
    persist: bool = True,
) -> None:
    last_stream_time = 0.0
    last_stream_wall = time.perf_counter()

    def stream(time_s: float, rows: list[dict], events: list[dict]) -> None:
        nonlocal last_stream_time, last_stream_wall
        while pause_event.is_set() and not cancel_event.is_set():
            time.sleep(0.05)
            last_stream_wall = time.perf_counter()

        factor = float(speed_factor.value)
        if factor > 0.0 and not cancel_event.is_set():
            target = max(time_s - last_stream_time, 0.0) / factor
            delay = target - (time.perf_counter() - last_stream_wall)
            while delay > 0.0 and not cancel_event.is_set():
                chunk = min(delay, 0.05)
                time.sleep(chunk)
                delay -= chunk

        last_stream_time = time_s
        last_stream_wall = time.perf_counter()
        try:
            messages.put_nowait(("sample", (time_s, rows, events)))
        except queue.Full:
            pass

    try:
        result = run_simulation(
            scenario,
            seed=seed,
            create_report=persist,
            persist=persist,
            on_sample=stream,
            should_cancel=cancel_event.is_set,
        )
    except Exception as error:
        messages.put(("failed", f"{type(error).__name__}: {error}"))
    else:
        messages.put(("finished", result))
