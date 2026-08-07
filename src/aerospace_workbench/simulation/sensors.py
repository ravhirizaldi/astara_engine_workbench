"""Sensor state, sampling models, and composable fault behavior."""

from __future__ import annotations

import ctypes
import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np

from ..flight_software.abi import SensorFrame
from ..mathematics.quaternions import quat_conjugate, quat_rotate
from ..mathematics.vectors import unit

if TYPE_CHECKING:
    from .truth_model import Body


@dataclass
class SensorChannelState:
    accelerometer_bias_m_s2: np.ndarray
    gyro_bias_rad_s: np.ndarray
    magnetometer_bias: np.ndarray
    barometer_bias_m: float
    gnss_position_bias_m: np.ndarray
    gnss_velocity_bias_m_s: np.ndarray
    previous_body_rates_rad_s: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )
    barometer_port_altitude_m: float = 0.0
    gnss_available_since_s: float | None = None
    gnss_has_lock: bool = False
    gnss_lost_lock: bool = False
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
        target = fault.get("component", fault.get("sensor"))
        if (
            bool(fault.get("_active", False))
            and
            fault.get("body", "all") in ("all", body)
            and target == component
            and (
                channel is None
                or "channel" not in fault
                or int(fault["channel"]) == channel
            )
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
            bool(fault.get("_active", False))
            and
            fault.get("body", "all") in ("all", body)
            and fault.get("component", fault.get("sensor")) == sensor
            and (
                "channel" not in fault
                or int(fault["channel"]) == channel
            )
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


def _channel_state(
    body: Body,
    sensors: dict[str, Any],
    rng: np.random.Generator,
    channel: int,
) -> SensorChannelState:
    while len(body.sensor_channels) <= channel:
        body.sensor_channels.append(
            SensorChannelState(
                rng.normal(
                    0.0,
                    float(sensors["accelerometer_noise_m_s2"]) * 0.1,
                    3,
                ),
                rng.normal(
                    0.0,
                    float(sensors["gyro_noise_rad_s"]) * 0.1,
                    3,
                ),
                rng.normal(
                    0.0, float(sensors["magnetometer_bias_sigma"]), 3
                ),
                float(
                    rng.normal(
                        0.0,
                        float(sensors["barometer_noise_m"]) * 0.1,
                    )
                ),
                rng.normal(
                    0.0,
                    float(sensors["gnss_position_noise_m"]) * 0.1,
                    3,
                ),
                rng.normal(
                    0.0,
                    float(sensors["gnss_velocity_noise_m_s"]) * 0.1,
                    3,
                ),
            )
        )
    return body.sensor_channels[channel]


def _temperature_delta_k(body: Body, sensors: dict[str, Any]) -> float:
    nominal_k = float(sensors["nominal_sensor_temperature_k"])
    temperature_k = nominal_k + float(
        sensors["engine_temperature_coupling"]
    ) * (float(body.last_engine_temperature_k) - nominal_k)
    return temperature_k - nominal_k


def _vector_model(
    value: np.ndarray,
    scale_error: list[float],
    misalignment: list[list[float]],
    temperature_coefficient: list[float],
    temperature_delta_k: float,
    quantization: float,
    saturation: float,
) -> np.ndarray:
    modeled = np.asarray(misalignment, dtype=float) @ value
    modeled *= 1.0 + np.asarray(scale_error, dtype=float)
    modeled += (
        np.asarray(temperature_coefficient, dtype=float)
        * temperature_delta_k
    )
    if quantization > 0.0:
        modeled = np.round(modeled / quantization) * quantization
    return np.clip(modeled, -saturation, saturation)


def _random_walk(
    value: np.ndarray,
    sigma_per_sqrt_s: float,
    dt_s: float,
    rng: np.random.Generator,
) -> None:
    if dt_s > 0.0 and sigma_per_sqrt_s > 0.0:
        value += rng.normal(0.0, sigma_per_sqrt_s * math.sqrt(dt_s), 3)


def _sensor_frame(
    body: Body,
    scenario: dict[str, Any],
    rng: np.random.Generator,
    time_s: float,
    dt_s: float,
    launch_position: np.ndarray,
    separated: bool,
    channel: int = 0,
) -> SensorFrame:
    sensors = scenario["sensors"]
    state = _channel_state(body, sensors, rng, channel)
    schedules = (
        ("imu", "next_imu_sample_s", "imu_rate_hz"),
        (
            "magnetometer",
            "next_magnetometer_sample_s",
            "magnetometer_rate_hz",
        ),
        ("barometer", "next_barometer_sample_s", "barometer_rate_hz"),
        ("gnss", "next_gnss_sample_s", "gnss_rate_hz"),
    )
    for device, next_field, rate_field in schedules:
        if not state.initialized or time_s + 1e-12 >= getattr(state, next_field):
            _sample_device(
                body,
                scenario,
                rng,
                time_s,
                launch_position,
                channel,
                device,
            )
            setattr(
                state,
                next_field,
                time_s + 1.0 / float(sensors[rate_field]),
            )
    state.initialized = True
    return _frame_from_values(
        body,
        scenario,
        rng,
        time_s,
        dt_s,
        separated,
        {
            "acceleration_body_m_s2": state.acceleration_body_m_s2,
            "gyro_body_rad_s": state.gyro_body_rad_s,
            "magnetic_body": state.magnetic_body,
            "barometric_altitude_m": state.barometric_altitude_m,
            "gnss_position_ecef_m": state.gnss_position_ecef_m,
            "gnss_velocity_ecef_m_s": state.gnss_velocity_ecef_m_s,
            "imu_sample_time_s": state.imu_sample_time_s,
            "magnetometer_sample_time_s": state.magnetometer_sample_time_s,
            "barometer_sample_time_s": state.barometer_sample_time_s,
            "gnss_sample_time_s": state.gnss_sample_time_s,
            "accel_valid": state.accel_valid,
            "gyro_valid": state.gyro_valid,
            "magnetometer_valid": state.magnetometer_valid,
            "barometer_valid": state.barometer_valid,
            "gnss_valid": state.gnss_valid,
        },
    )


def _sample_device(
    body: Body,
    scenario: dict[str, Any],
    rng: np.random.Generator,
    time_s: float,
    launch_position: np.ndarray,
    channel: int,
    device: str,
) -> dict[str, Any]:
    sensors = scenario["sensors"]
    state = _channel_state(body, sensors, rng, channel)
    faults = sensor_faults(scenario, body.name, device, time_s, channel)
    state.fault_state = tuple(str(fault.get("type")) for fault in faults)
    temperature_delta_k = _temperature_delta_k(body, sensors)
    if device == "imu":
        sample_dt_s = (
            max(time_s - state.imu_sample_time_s, 0.0)
            if state.initialized
            else 0.0
        )
        _random_walk(
            state.accelerometer_bias_m_s2,
            float(sensors["accelerometer_bias_random_walk_m_s2_sqrt_s"]),
            sample_dt_s,
            rng,
        )
        _random_walk(
            state.gyro_bias_rad_s,
            float(sensors["gyro_bias_random_walk_rad_s_sqrt_s"]),
            sample_dt_s,
            rng,
        )
        angular_acceleration = (
            (body.body_rates_rad_s - state.previous_body_rates_rad_s)
            / sample_dt_s
            if sample_dt_s > 0.0
            else np.zeros(3)
        )
        lever_arm = np.asarray(
            sensors["imu_lever_arm_body_m"], dtype=float
        )
        lever_acceleration = np.cross(angular_acceleration, lever_arm)
        lever_acceleration += np.cross(
            body.body_rates_rad_s,
            np.cross(body.body_rates_rad_s, lever_arm),
        )
        state.previous_body_rates_rad_s = body.body_rates_rad_s.copy()
        vibration = np.linalg.norm(body.last_specific_force_body_m_s2)
        acceleration_truth = (
            body.last_specific_force_body_m_s2
            + lever_acceleration
            + state.accelerometer_bias_m_s2
            + rng.normal(0.0, sensors["accelerometer_noise_m_s2"], 3)
            + rng.normal(
                0.0,
                float(sensors["accelerometer_vibration_sensitivity"])
                * vibration,
                3,
            )
        )
        gyro_truth = (
            body.body_rates_rad_s
            + state.gyro_bias_rad_s
            + rng.normal(0.0, sensors["gyro_noise_rad_s"], 3)
            + rng.normal(
                0.0,
                float(sensors["gyro_vibration_sensitivity"]) * vibration,
                3,
            )
        )
        acceleration, imu_time, state.accel_valid = apply_sensor_faults(
            _vector_model(
                acceleration_truth,
                sensors["accelerometer_scale_factor_error"],
                sensors["imu_cross_axis_misalignment"],
                sensors["accelerometer_temperature_coefficient_m_s2_k"],
                temperature_delta_k,
                float(sensors["accelerometer_quantization_m_s2"]),
                float(sensors["accelerometer_saturation_m_s2"]),
            ),
            state.acceleration_body_m_s2,
            time_s,
            state.imu_sample_time_s,
            faults,
        )
        gyro, imu_time, state.gyro_valid = apply_sensor_faults(
            _vector_model(
                gyro_truth,
                sensors["gyro_scale_factor_error"],
                sensors["imu_cross_axis_misalignment"],
                sensors["gyro_temperature_coefficient_rad_s_k"],
                temperature_delta_k,
                float(sensors["gyro_quantization_rad_s"]),
                float(sensors["gyro_saturation_rad_s"]),
            ),
            state.gyro_body_rad_s,
            imu_time,
            state.imu_sample_time_s,
            faults,
        )
        state.acceleration_body_m_s2 = acceleration
        state.gyro_body_rad_s = gyro
        state.imu_sample_time_s = imu_time
        return {
            "acceleration_body_m_s2": acceleration.copy(),
            "gyro_body_rad_s": gyro.copy(),
            "imu_sample_time_s": imu_time,
            "accel_valid": state.accel_valid,
            "gyro_valid": state.gyro_valid,
        }
    if device == "magnetometer":
        sample_dt_s = (
            max(time_s - state.magnetometer_sample_time_s, 0.0)
            if state.initialized
            else 0.0
        )
        _random_walk(
            state.magnetometer_bias,
            float(sensors["magnetometer_bias_random_walk_sqrt_s"]),
            sample_dt_s,
            rng,
        )
        magnetic_truth = (
            quat_rotate(
                quat_conjugate(body.attitude_wxyz),
                unit(np.array([0.28, 0.08, -0.52])),
            )
            + state.magnetometer_bias
            + rng.normal(0.0, float(sensors["magnetometer_noise"]), 3)
        )
        magnetic, sample_time_s, state.magnetometer_valid = apply_sensor_faults(
            _vector_model(
                magnetic_truth,
                sensors["magnetometer_scale_factor_error"],
                sensors["magnetometer_cross_axis_misalignment"],
                sensors["magnetometer_temperature_coefficient_k"],
                temperature_delta_k,
                float(sensors["magnetometer_quantization"]),
                float(sensors["magnetometer_saturation"]),
            ),
            state.magnetic_body,
            time_s,
            state.magnetometer_sample_time_s,
            faults,
        )
        state.magnetic_body = magnetic
        state.magnetometer_sample_time_s = sample_time_s
        return {
            "magnetic_body": magnetic.copy(),
            "magnetometer_sample_time_s": sample_time_s,
            "magnetometer_valid": state.magnetometer_valid,
        }
    if device == "barometer":
        altitude = float(
            np.linalg.norm(body.position_ecef_m) - np.linalg.norm(launch_position)
        )
        sample_dt_s = (
            max(time_s - state.barometer_sample_time_s, 0.0)
            if state.initialized
            else 0.0
        )
        if sample_dt_s > 0.0:
            state.barometer_bias_m += float(
                rng.normal(
                    0.0,
                    float(sensors["barometer_bias_random_walk_m_sqrt_s"])
                    * math.sqrt(sample_dt_s),
                )
            )
        port_alpha = (
            1.0
            - math.exp(
                -sample_dt_s / float(sensors["barometer_port_time_constant_s"])
            )
            if sample_dt_s > 0.0
            else 1.0
        )
        state.barometer_port_altitude_m += port_alpha * (
            altitude - state.barometer_port_altitude_m
        )
        sensor_alpha = (
            1.0
            - math.exp(
                -sample_dt_s / float(sensors["barometer_lag_time_constant_s"])
            )
            if sample_dt_s > 0.0
            else 1.0
        )
        raw_altitude = (
            state.barometer_port_altitude_m
            + state.barometer_bias_m
            + float(sensors["barometer_temperature_coefficient_m_k"])
            * temperature_delta_k
            + rng.normal(0.0, sensors["barometer_noise_m"])
        )
        lagged_altitude = state.barometric_altitude_m + sensor_alpha * (
            raw_altitude - state.barometric_altitude_m
        )
        quantum = float(sensors["barometer_quantization_m"])
        if quantum > 0.0:
            lagged_altitude = round(lagged_altitude / quantum) * quantum
        value, sample_time_s, state.barometer_valid = apply_sensor_faults(
            np.array(
                [
                    min(
                        max(
                            lagged_altitude,
                            float(sensors["barometer_min_altitude_m"]),
                        ),
                        float(sensors["barometer_max_altitude_m"]),
                    )
                ]
            ),
            np.array([state.barometric_altitude_m]),
            time_s,
            state.barometer_sample_time_s,
            faults,
        )
        state.barometric_altitude_m = float(value[0])
        state.barometer_sample_time_s = sample_time_s
        return {
            "barometric_altitude_m": state.barometric_altitude_m,
            "barometer_sample_time_s": sample_time_s,
            "barometer_valid": state.barometer_valid,
        }
    if device == "gnss":
        dropout = any(fault.get("type") == "dropout" for fault in faults)
        if dropout:
            state.gnss_has_lock = False
            state.gnss_lost_lock = True
            state.gnss_available_since_s = None
        elif not state.gnss_has_lock:
            if state.gnss_available_since_s is None:
                state.gnss_available_since_s = time_s
            acquisition_s = float(
                sensors[
                    "gnss_reacquisition_time_s"
                    if state.gnss_lost_lock
                    else "gnss_acquisition_time_s"
                ]
            )
            state.gnss_has_lock = (
                time_s - state.gnss_available_since_s + 1e-12
                >= acquisition_s
            )
        sample_dt_s = (
            max(time_s - state.gnss_sample_time_s, 0.0)
            if state.initialized
            else 0.0
        )
        _random_walk(
            state.gnss_position_bias_m,
            float(sensors["gnss_position_bias_random_walk_m_sqrt_s"]),
            sample_dt_s,
            rng,
        )
        _random_walk(
            state.gnss_velocity_bias_m_s,
            float(sensors["gnss_velocity_bias_random_walk_m_s_sqrt_s"]),
            sample_dt_s,
            rng,
        )
        position = (
            body.position_ecef_m
            + state.gnss_position_bias_m
            + rng.normal(0.0, sensors["gnss_position_noise_m"], 3)
        ) * (
            1.0
            + np.asarray(
                sensors["gnss_position_scale_factor_error"], dtype=float
            )
        )
        velocity = (
            body.velocity_ecef_m_s
            + state.gnss_velocity_bias_m_s
            + rng.normal(0.0, sensors["gnss_velocity_noise_m_s"], 3)
        ) * (
            1.0
            + np.asarray(
                sensors["gnss_velocity_scale_factor_error"], dtype=float
            )
        )
        position_quantum = float(sensors["gnss_position_quantization_m"])
        velocity_quantum = float(sensors["gnss_velocity_quantization_m_s"])
        if position_quantum > 0.0:
            position = np.round(position / position_quantum) * position_quantum
        if velocity_quantum > 0.0:
            velocity = np.round(velocity / velocity_quantum) * velocity_quantum
        value, sample_time_s, state.gnss_valid = apply_sensor_faults(
            np.concatenate((position, velocity)),
            np.concatenate(
                (state.gnss_position_ecef_m, state.gnss_velocity_ecef_m_s)
            ),
            time_s,
            state.gnss_sample_time_s,
            faults,
        )
        state.gnss_valid &= int(state.gnss_has_lock)
        state.gnss_position_ecef_m = value[:3].copy()
        state.gnss_velocity_ecef_m_s = value[3:].copy()
        state.gnss_sample_time_s = sample_time_s
        return {
            "gnss_position_ecef_m": value[:3].copy(),
            "gnss_velocity_ecef_m_s": value[3:].copy(),
            "gnss_sample_time_s": sample_time_s,
            "gnss_valid": state.gnss_valid,
        }
    raise ValueError(f"unknown sensor device {device!r}")


def _frame_from_values(
    body: Body,
    scenario: dict[str, Any],
    rng: np.random.Generator,
    time_s: float,
    dt_s: float,
    separated: bool,
    values: dict[str, Any],
) -> SensorFrame:
    gnss_position = np.asarray(
        values.get("gnss_position_ecef_m", np.zeros(3))
    )
    gnss_velocity = np.asarray(
        values.get("gnss_velocity_ecef_m_s", np.zeros(3))
    )
    up = unit(body.position_ecef_m)
    vertical_velocity = float(np.dot(gnss_velocity, up))
    return SensorFrame(
        time_s,
        dt_s,
        (ctypes.c_double * 3)(
            *values.get("acceleration_body_m_s2", np.zeros(3))
        ),
        (ctypes.c_double * 3)(*values.get("gyro_body_rad_s", np.zeros(3))),
        (ctypes.c_double * 3)(
            *values.get("magnetic_body", np.array([1.0, 0.0, 0.0]))
        ),
        float(values.get("barometric_altitude_m", 0.0)),
        (ctypes.c_double * 3)(*gnss_position),
        (ctypes.c_double * 3)(*gnss_velocity),
        vertical_velocity,
        0.0,
        0.0,
        int(values.get("gnss_valid", 0)),
        int(values.get("barometer_valid", 0)),
        0,
        float(values.get("barometer_sample_time_s", 0.0)),
        float(values.get("gnss_sample_time_s", 0.0)),
        0,
        0,
        0,
        0,
        float(values.get("imu_sample_time_s", 0.0)),
        float(values.get("magnetometer_sample_time_s", 0.0)),
        int(values.get("accel_valid", 0)),
        int(values.get("gyro_valid", 0)),
        int(values.get("magnetometer_valid", 0)),
    )
