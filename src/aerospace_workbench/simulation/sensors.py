"""Sensor-channel state and composable fault behavior."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class SensorChannelState:
    accelerometer_bias_m_s2: np.ndarray
    gyro_bias_rad_s: np.ndarray
    magnetometer_bias: np.ndarray
    barometer_bias_m: float
    gnss_position_bias_m: np.ndarray
    gnss_velocity_bias_m_s: np.ndarray
    acceleration_body_m_s2: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )
    gyro_body_rad_s: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )
    magnetic_body: np.ndarray = field(
        default_factory=lambda: np.array([1.0, 0.0, 0.0])
    )
    barometric_altitude_m: float = 0.0
    gnss_position_ecef_m: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )
    gnss_velocity_ecef_m_s: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )
    imu_sample_time_s: float = 0.0
    magnetometer_sample_time_s: float = 0.0
    barometer_sample_time_s: float = 0.0
    gnss_sample_time_s: float = 0.0
    next_imu_sample_s: float = 0.0
    next_magnetometer_sample_s: float = 0.0
    next_barometer_sample_s: float = 0.0
    next_gnss_sample_s: float = 0.0
    accel_valid: int = 0
    gyro_valid: int = 0
    magnetometer_valid: int = 0
    barometer_valid: int = 0
    gnss_valid: int = 0
    initialized: bool = False
    fault_state: tuple[str, ...] = ()


def fault_active(
    scenario: dict[str, Any],
    body: str,
    component: str,
    time_s: float,
    channel: int | None = None,
) -> dict[str, Any] | None:
    for fault in scenario.get("faults", []):
        start = float(fault.get("start_s", 0.0))
        end = start + float(fault.get("duration_s", math.inf))
        target = fault.get("component", fault.get("sensor"))
        if (
            fault.get("body", "all") in ("all", body)
            and target == component
            and (
                channel is None
                or "channel" not in fault
                or int(fault["channel"]) == channel
            )
            and start <= time_s <= end
        ):
            return fault
    return None


def sensor_faults(
    scenario: dict[str, Any],
    body: str,
    sensor: str,
    time_s: float,
    channel: int,
) -> list[dict[str, Any]]:
    return [
        fault
        for fault in scenario.get("faults", [])
        if (
            fault.get("body", "all") in ("all", body)
            and fault.get("component", fault.get("sensor")) == sensor
            and (
                "channel" not in fault
                or int(fault["channel"]) == channel
            )
            and float(fault.get("start_s", 0.0)) <= time_s
            <= float(fault.get("start_s", 0.0))
            + float(fault.get("duration_s", math.inf))
        )
    ]


def apply_sensor_faults(
    value: np.ndarray,
    previous: np.ndarray,
    sample_time_s: float,
    previous_time_s: float,
    faults: list[dict[str, Any]],
) -> tuple[np.ndarray, float, int]:
    result = value.copy()
    timestamp = sample_time_s
    valid = 1
    kinds = {str(fault.get("type")) for fault in faults}
    if "dropout" in kinds:
        result, timestamp, valid = previous.copy(), previous_time_s, 0
    elif "stale" in kinds:
        result, timestamp = previous.copy(), previous_time_s
    elif "freeze" in kinds:
        result = previous.copy()
    for fault in faults:
        if fault.get("type") == "scale_error":
            result *= float(fault.get("value", 1.0))
    for fault in faults:
        if fault.get("type") == "bias":
            result += float(fault.get("value", 0.0))
    if "stuck-valid" in kinds:
        valid = 1
    return result, timestamp, valid
