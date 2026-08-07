"""Background simulation process used by the live mission view."""

from __future__ import annotations

import os
import queue
import time

from ...simulation.runner import run_simulation
from ...simulation.orchestration.mission_run import (
    LIVE_FAULT_BODIES,
    LIVE_FAULT_TYPES_BY_COMPONENT,
    LIVE_FAULT_VALUE_TYPES,
    normalize_live_fault_command,
)

SOLVER_NICE_INCREMENT = 5
CONTROL_WAIT_SLICE_S = 0.05
DISPLAY_ROW_FIELDS = (
    "body",
    "time_s",
    "position_ecef_x_m",
    "position_ecef_y_m",
    "position_ecef_z_m",
    "altitude_m",
    "speed_m_s",
    "mach",
    "thrust_n",
    "mode",
    "landed",
)


def run_simulation_process(
    scenario: dict,
    seed: int,
    messages,
    cancel_event,
    pause_event,
    speed_factor,
    persist: bool = True,
    controls=None,
) -> None:
    if hasattr(os, "nice"):
        try:
            os.nice(SOLVER_NICE_INCREMENT)
        except OSError:
            pass
    last_stream_time = 0.0
    last_stream_wall = time.perf_counter()

    def drain_controls(_time_s: float) -> list[dict]:
        if controls is None:
            return []
        commands = []
        while True:
            try:
                commands.append(controls.get_nowait())
            except queue.Empty:
                return commands

    def stream(time_s: float, rows: list[dict], events: list[dict]) -> None:
        nonlocal last_stream_time, last_stream_wall
        while pause_event.is_set() and not cancel_event.is_set():
            time.sleep(CONTROL_WAIT_SLICE_S)
            last_stream_wall = time.perf_counter()

        factor = float(speed_factor.value)
        if factor > 0.0 and not cancel_event.is_set():
            target = max(time_s - last_stream_time, 0.0) / factor
            delay = target - (time.perf_counter() - last_stream_wall)
            while delay > 0.0 and not cancel_event.is_set():
                chunk = min(delay, CONTROL_WAIT_SLICE_S)
                time.sleep(chunk)
                delay -= chunk

        last_stream_time = time_s
        last_stream_wall = time.perf_counter()
        display_rows = [
            {field: row[field] for field in DISPLAY_ROW_FIELDS}
            for row in rows
        ]
        try:
            messages.put_nowait(("sample", (time_s, display_rows, events)))
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
            control_source=drain_controls,
        )
    except Exception as error:
        messages.put(("failed", f"{type(error).__name__}: {error}"))
    else:
        messages.put(
            (
                "finished",
                {
                    "output_dir": str(result.output_dir),
                    "manifest": result.manifest,
                    "events": result.events,
                },
            )
        )
