"""Live mission telemetry preparation independent of the desktop toolkit."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from ..mathematics.frames import geodetic_to_ecef, ned_basis


def trajectory_projection(scenario: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    """Return the launch origin and configured downrange axis."""
    environment = scenario["environment"]
    launch_position = geodetic_to_ecef(
        float(environment["latitude_deg"]),
        float(environment["longitude_deg"]),
        float(environment["launch_altitude_m"]),
    )
    north, east, _down = ned_basis(launch_position)
    azimuth = math.radians(float(environment["launch_azimuth_deg"]))
    downrange_axis = math.cos(azimuth) * north + math.sin(azimuth) * east
    return launch_position, downrange_axis


def live_telemetry_series(
    telemetry: list[dict[str, Any]],
    scenario: dict[str, Any],
    point_limit: int,
) -> dict[str, dict[str, Any]]:
    """Group and downsample telemetry into plot-ready body trajectories."""
    if point_limit < 1:
        raise ValueError("point_limit must be positive")

    launch_position, downrange_axis = trajectory_projection(scenario)

    grouped: dict[str, list[dict[str, Any]]] = {}
    for row in telemetry:
        grouped.setdefault(str(row["body"]), []).append(row)

    series: dict[str, dict[str, Any]] = {}
    for body, body_rows in grouped.items():
        stride = max(1, math.ceil(len(body_rows) / point_limit))
        rows = body_rows[::stride]
        if rows[-1] is not body_rows[-1]:
            rows.append(body_rows[-1])

        positions = np.array(
            [
                [
                    float(row["position_ecef_x_m"]),
                    float(row["position_ecef_y_m"]),
                    float(row["position_ecef_z_m"]),
                ]
                for row in rows
            ]
        )
        relative_positions = positions - launch_position
        series[body] = {
            "rows": rows,
            "time_s": [float(row["time_s"]) for row in rows],
            "altitude_km": [float(row["altitude_m"]) / 1000.0 for row in rows],
            "speed_m_s": [float(row["speed_m_s"]) for row in rows],
            "thrust_kn": [float(row["thrust_n"]) / 1000.0 for row in rows],
            "downrange_km": list(relative_positions @ downrange_axis / 1000.0),
        }
    return series
