"""Normalize runtime mission and flight-mode events."""

from __future__ import annotations

from ..flight_software.abi import MODE_NAMES


def event(
    time_s: float, body: str, name: str, detail: str = ""
) -> dict[str, object]:
    return {
        "time_s": time_s,
        "body": body,
        "event": name,
        "detail": detail,
    }


def flight_mode_events(
    time_s: float,
    body: str,
    previous_mode: int | None,
    current_mode: int,
) -> list[dict[str, object]]:
    current_name = MODE_NAMES[current_mode]
    events = [event(time_s, body, "flight_mode", current_name)]
    previous_name = (
        MODE_NAMES[previous_mode] if previous_mode is not None else None
    )
    if previous_name == "BOOST_1" and current_name == "SEPARATION":
        events.append(event(time_s, body, "meco"))
    if current_name == "BOOST_2":
        events.append(event(time_s, body, "stage2_ignition"))
    if previous_name == "BOOST_2" and current_name == "COAST":
        events.append(event(time_s, body, "stage2_first_cutoff"))
    return events
