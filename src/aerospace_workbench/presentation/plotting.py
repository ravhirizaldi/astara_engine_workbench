"""Live mission plot rendering."""

from __future__ import annotations

from typing import Any


def plot_live_telemetry(
    altitude_axis: Any,
    speed_axis: Any,
    thrust_axis: Any,
    trajectory_axis: Any,
    telemetry: list[dict],
    point_limit: int,
) -> None:
    for axis in (
        altitude_axis,
        speed_axis,
        thrust_axis,
        trajectory_axis,
    ):
        axis.clear()

    grouped: dict[str, list[dict]] = {}
    for row in telemetry:
        grouped.setdefault(row["body"], []).append(row)
    for body, body_rows in sorted(grouped.items()):
        stride = max(1, len(body_rows) // point_limit)
        rows = body_rows[::stride]
        altitude_axis.plot(
            [row["time_s"] for row in rows],
            [row["altitude_m"] for row in rows],
            label=body,
        )
        speed_axis.plot(
            [row["time_s"] for row in rows],
            [row["speed_m_s"] for row in rows],
            label=body,
        )
        thrust_axis.plot(
            [row["time_s"] for row in rows],
            [row["thrust_n"] / 1000.0 for row in rows],
            label=body,
        )
        origin = rows[0]
        trajectory_axis.plot(
            [
                (row["position_ecef_x_m"] - origin["position_ecef_x_m"])
                / 1000.0
                for row in rows
            ],
            [
                (row["position_ecef_y_m"] - origin["position_ecef_y_m"])
                / 1000.0
                for row in rows
            ],
            label=body,
        )

    altitude_axis.set_title("Altitude")
    altitude_axis.set_ylabel("Altitude (m)")
    altitude_axis.set_xlabel("Time (s)")
    speed_axis.set_title("Speed")
    speed_axis.set_ylabel("Speed (m/s)")
    speed_axis.set_xlabel("Time (s)")
    thrust_axis.set_title("Thrust")
    thrust_axis.set_ylabel("Thrust (kN)")
    thrust_axis.set_xlabel("Time (s)")
    trajectory_axis.set_title("Relative ECEF Ground Track")
    trajectory_axis.set_xlabel("ΔX km")
    trajectory_axis.set_ylabel("ΔY km")
    for axis in (
        altitude_axis,
        speed_axis,
        thrust_axis,
        trajectory_axis,
    ):
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=7)
