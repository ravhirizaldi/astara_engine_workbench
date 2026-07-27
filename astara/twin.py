"""ASTARA deterministic 6-DOF software-in-the-loop mission simulation."""

from __future__ import annotations

import csv
import ctypes
import gzip
import hashlib
import json
import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import numpy as np

from . import __version__
from .aero import AeroResult, atmosphere, estimate
from .flight_core import (
    FSW_BODY_CORE,
    FSW_BODY_INTEGRATED,
    FSW_DISCRETE_ACTION_DEPLOY_DROGUE,
    FSW_DISCRETE_ACTION_DEPLOY_MAIN,
    FSW_DISCRETE_ACTION_STAGE_SEPARATE,
    MODE_NAMES,
    NAVIGATION_STATUS_NAMES,
    SENSOR_CSV_FIELDS,
    FlightCore,
    FswOutput,
    SensorFrame,
    decode_faults,
    sensor_frame_to_row,
)
from .math3d import (
    EARTH_MU,
    EARTH_RADIUS_M,
    EARTH_ROTATION_RAD_S,
    cross3,
    ecef_to_geodetic,
    ecef_to_ned,
    geodetic_to_ecef,
    initial_attitude,
    ned_to_ecef,
    quat_conjugate,
    quat_derivative,
    quat_normalize,
    quat_rotate,
    quat_to_euler,
    unit,
)
from .scenario import evidence_documents, model_source_hash, scenario_hash, validate_scenario


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
    initialized: bool = False
    fault_state: tuple[str, ...] = ()


@dataclass
class Body:
    name: str
    stage_index: int
    stage: dict[str, Any]
    position_ecef_m: np.ndarray
    velocity_ecef_m_s: np.ndarray
    attitude_wxyz: np.ndarray
    body_rates_rad_s: np.ndarray
    fuel_kg: float
    oxidizer_kg: float
    upper_mass_kg: float = 0.0
    stacked_length_m: float | None = None
    landed: bool = False
    engine_started_s: float | None = None
    engine_health_percent: float = 100.0
    drogue_deployed: bool = False
    main_deployed: bool = False
    parachute_deployed_s: float | None = None
    last_specific_force_body_m_s2: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )
    last_dynamic_pressure_pa: float = 0.0
    last_mach: float = 0.0
    last_angle_of_attack_deg: float = 0.0
    aero_valid: bool = True
    sensor_channels: list[SensorChannelState] = field(default_factory=list)
    next_fsw_sample_s: float = 0.0
    last_discrete_actuation_sequence: int = 0
    last_tvc_rad: np.ndarray = field(default_factory=lambda: np.zeros(2))
    last_fin_rad: np.ndarray = field(default_factory=lambda: np.zeros(3))
    last_engine_thrusts_n: dict[str, float] = field(default_factory=dict)

    @property
    def mass_kg(self) -> float:
        return (
            float(self.stage["dry_mass_kg"])
            + self.fuel_kg
            + self.oxidizer_kg
            + self.upper_mass_kg
        )

    @property
    def propellant_fraction(self) -> float:
        initial = float(self.stage["fuel_mass_kg"]) + float(
            self.stage["oxidizer_mass_kg"]
        )
        return min(max((self.fuel_kg + self.oxidizer_kg) / initial, 0.0), 1.0)

    @property
    def center_of_mass_m(self) -> float:
        table = self.stage.get("mass_properties")
        if not table:
            return float(self.stage["center_of_mass_m"])
        return float(
            np.interp(
                self.propellant_fraction,
                [row["propellant_fraction"] for row in table],
                [row["center_of_mass_m"] for row in table],
            )
        )

    @property
    def inertia_kg_m2(self) -> np.ndarray:
        table = self.stage.get("mass_properties")
        if table:
            inertia = np.array(
                [
                    np.interp(
                        self.propellant_fraction,
                        [row["propellant_fraction"] for row in table],
                        [row["inertia_kg_m2"][axis] for row in table],
                    )
                    for axis in range(3)
                ]
            )
        else:
            inertia = np.asarray(self.stage["inertia_kg_m2"], dtype=float)
        if self.upper_mass_kg > 0.0:
            inertia = inertia + np.array(
                [2.4, self.upper_mass_kg * 2.2**2, self.upper_mass_kg * 2.2**2]
            )
        return np.maximum(inertia, 1e-3)

    def aerodynamic_stage(self) -> dict[str, Any]:
        combined = dict(self.stage)
        combined["center_of_mass_m"] = self.center_of_mass_m
        if self.stacked_length_m is None:
            return combined
        combined["length_m"] = self.stacked_length_m
        combined["center_of_mass_m"] = self.stacked_length_m * 0.54
        combined["aerodynamics"] = dict(self.stage["aerodynamics"])
        combined["aerodynamics"]["center_of_pressure_m"] = self.stacked_length_m * 0.68
        return combined


@dataclass
class RunResult:
    output_dir: Path
    manifest: dict[str, Any]
    telemetry: list[dict[str, Any]]
    fsw_telemetry: list[dict[str, Any]]
    events: list[dict[str, Any]]


def _stage_total_mass(stage: dict[str, Any]) -> float:
    return stage["dry_mass_kg"] + stage["fuel_mass_kg"] + stage["oxidizer_mass_kg"]


def _stage_engines(stage: dict[str, Any]) -> list[dict[str, Any]]:
    return stage.get("engines") or [
        {
            "id": f"{stage.get('name', 'stage')}-engine-1",
            "position_body_m": [0.0, 0.0, 0.0],
            "direction_body": [1.0, 0.0, 0.0],
            "performance_scale": 1.0,
            "enabled": True,
            "gimbal_enabled": True,
        }
    ]


def _fault_active(
    scenario: dict[str, Any],
    body: str,
    sensor: str,
    time_s: float,
    channel: int | None = None,
) -> dict[str, Any] | None:
    for fault in scenario.get("faults", []):
        start = float(fault.get("start_s", 0.0))
        end = start + float(fault.get("duration_s", math.inf))
        target = fault.get("component", fault.get("sensor"))
        if (
            fault.get("body", "all") in ("all", body)
            and target == sensor
            and (
                channel is None
                or "channel" not in fault
                or int(fault["channel"]) == channel
            )
            and start <= time_s <= end
        ):
            return fault
    return None


def _sensor_faults(
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
                rng.normal(0.0, 0.002, 3),
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


def _apply_sensor_faults(
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


def _engine_fault_active(
    scenario: dict[str, Any], body: str, engine_id: str, time_s: float
) -> dict[str, Any] | None:
    for fault in scenario.get("faults", []):
        start = float(fault.get("start_s", 0.0))
        end = start + float(fault.get("duration_s", math.inf))
        target = fault.get("component", fault.get("sensor"))
        target_engine = fault.get("engine_id", "all")
        if (
            fault.get("body", "all") in ("all", body)
            and target == "engine"
            and target_engine in ("all", engine_id)
            and start <= time_s <= end
        ):
            return fault
    return None


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
    altitude = float(np.linalg.norm(body.position_ecef_m) - np.linalg.norm(launch_position))
    up = unit(body.position_ecef_m)
    magnetic_ecef = unit(np.array([0.28, 0.08, -0.52]))
    previous_acceleration = state.acceleration_body_m_s2.copy()
    previous_gyro = state.gyro_body_rad_s.copy()
    previous_magnetic = state.magnetic_body.copy()
    previous_barometer = np.array([state.barometric_altitude_m])
    previous_gnss = np.concatenate(
        (state.gnss_position_ecef_m, state.gnss_velocity_ecef_m_s)
    )
    previous_imu_time = state.imu_sample_time_s
    previous_magnetometer_time = state.magnetometer_sample_time_s
    previous_barometer_time = state.barometer_sample_time_s
    previous_gnss_time = state.gnss_sample_time_s
    if not state.initialized or time_s + 1e-12 >= state.next_imu_sample_s:
        state.acceleration_body_m_s2 = (
            body.last_specific_force_body_m_s2
            + state.accelerometer_bias_m_s2
            + rng.normal(
                0.0, sensors["accelerometer_noise_m_s2"], 3
            )
        )
        state.gyro_body_rad_s = (
            body.body_rates_rad_s
            + state.gyro_bias_rad_s
            + rng.normal(0.0, sensors["gyro_noise_rad_s"], 3)
        )
        state.imu_sample_time_s = time_s
        state.next_imu_sample_s = (
            time_s + 1.0 / float(sensors["imu_rate_hz"])
        )
    if (
        not state.initialized
        or time_s + 1e-12 >= state.next_magnetometer_sample_s
    ):
        state.magnetic_body = (
            quat_rotate(
                quat_conjugate(body.attitude_wxyz), magnetic_ecef
            )
            + state.magnetometer_bias
            + rng.normal(0.0, 0.002, 3)
        )
        state.magnetometer_sample_time_s = time_s
        state.next_magnetometer_sample_s = (
            time_s + 1.0 / float(sensors["imu_rate_hz"])
        )
    if (
        not state.initialized
        or time_s + 1e-12 >= state.next_barometer_sample_s
    ):
        state.barometric_altitude_m = float(
            altitude
            + state.barometer_bias_m
            + rng.normal(0.0, sensors["barometer_noise_m"])
        )
        state.barometer_sample_time_s = time_s
        state.next_barometer_sample_s = (
            time_s + 1.0 / float(sensors["barometer_rate_hz"])
        )
    if (
        not state.initialized
        or time_s + 1e-12 >= state.next_gnss_sample_s
    ):
        state.gnss_position_ecef_m = (
            body.position_ecef_m
            + state.gnss_position_bias_m
            + rng.normal(0.0, sensors["gnss_position_noise_m"], 3)
        )
        state.gnss_velocity_ecef_m_s = (
            body.velocity_ecef_m_s
            + state.gnss_velocity_bias_m_s
            + rng.normal(0.0, sensors["gnss_velocity_noise_m_s"], 3)
        )
        state.gnss_sample_time_s = time_s
        state.next_gnss_sample_s = (
            time_s + 1.0 / float(sensors["gnss_rate_hz"])
        )
    imu_faults = _sensor_faults(
        scenario, body.name, "imu", time_s, channel
    )
    magnetometer_faults = _sensor_faults(
        scenario, body.name, "magnetometer", time_s, channel
    )
    barometer_faults = _sensor_faults(
        scenario, body.name, "barometer", time_s, channel
    )
    gnss_faults = _sensor_faults(
        scenario, body.name, "gnss", time_s, channel
    )
    acceleration, imu_time, accel_valid = _apply_sensor_faults(
        state.acceleration_body_m_s2,
        previous_acceleration,
        state.imu_sample_time_s,
        previous_imu_time,
        imu_faults,
    )
    gyro, imu_time, gyro_valid = _apply_sensor_faults(
        state.gyro_body_rad_s,
        previous_gyro,
        imu_time,
        previous_imu_time,
        imu_faults,
    )
    magnetic_body, magnetometer_time, magnetometer_valid = (
        _apply_sensor_faults(
            state.magnetic_body,
            previous_magnetic,
            state.magnetometer_sample_time_s,
            previous_magnetometer_time,
            magnetometer_faults,
        )
    )
    barometer_value, barometer_time, barometer_valid = (
        _apply_sensor_faults(
            np.array([state.barometric_altitude_m]),
            previous_barometer,
            state.barometer_sample_time_s,
            previous_barometer_time,
            barometer_faults,
        )
    )
    gnss_value, gnss_time, gnss_valid = _apply_sensor_faults(
        np.concatenate(
            (state.gnss_position_ecef_m, state.gnss_velocity_ecef_m_s)
        ),
        previous_gnss,
        state.gnss_sample_time_s,
        previous_gnss_time,
        gnss_faults,
    )
    state.acceleration_body_m_s2 = acceleration.copy()
    state.gyro_body_rad_s = gyro.copy()
    state.magnetic_body = magnetic_body.copy()
    state.barometric_altitude_m = float(barometer_value[0])
    state.gnss_position_ecef_m = gnss_value[:3].copy()
    state.gnss_velocity_ecef_m_s = gnss_value[3:].copy()
    state.imu_sample_time_s = imu_time
    state.magnetometer_sample_time_s = magnetometer_time
    state.barometer_sample_time_s = barometer_time
    state.gnss_sample_time_s = gnss_time
    state.fault_state = tuple(
        str(fault.get("type"))
        for fault in (
            imu_faults
            + magnetometer_faults
            + barometer_faults
            + gnss_faults
        )
    )
    state.initialized = True
    barometer_altitude = float(barometer_value[0])
    gnss_position = gnss_value[:3]
    gnss_velocity = gnss_value[3:]
    vertical_velocity = float(np.dot(gnss_velocity, up))
    dynamic_pressure = max(
        0.0,
        body.last_dynamic_pressure_pa
        + float(rng.normal(0.0, sensors.get("dynamic_pressure_noise_pa", 0.0))),
    )
    engine_health = min(
        100.0,
        max(
            0.0,
            body.engine_health_percent
            + float(rng.normal(0.0, sensors.get("engine_health_noise_percent", 0.0))),
        ),
    )
    propulsion_running = (
        body.engine_started_s is not None
        and time_s
            < body.engine_started_s
                + float(body.stage["propulsion"]["burn_duration_s"])
    )
    return SensorFrame(
        time_s,
        dt_s,
        (ctypes.c_double * 3)(*acceleration),
        (ctypes.c_double * 3)(*gyro),
        (ctypes.c_double * 3)(*magnetic_body),
        barometer_altitude,
        (ctypes.c_double * 3)(*gnss_position),
        (ctypes.c_double * 3)(*gnss_velocity),
        vertical_velocity,
        dynamic_pressure,
        engine_health,
        gnss_valid,
        barometer_valid,
        int(separated),
        barometer_time,
        gnss_time,
        int(engine_health > 0.0),
        int(propulsion_running),
        int(body.drogue_deployed),
        int(body.main_deployed),
        imu_time,
        magnetometer_time,
        accel_valid,
        gyro_valid,
        magnetometer_valid,
    )


def _run_fsw_substeps(
    core: FlightCore,
    body: Body,
    scenario: dict[str, Any],
    rng: np.random.Generator,
    time_s: float,
    launch_position: np.ndarray,
    separated: bool,
    current_output: FswOutput,
    on_sensor: Callable[
        [str, list[SensorFrame], FswOutput], None
    ] | None = None,
) -> FswOutput:
    fsw_step_s = 1.0 / float(scenario["sensors"]["imu_rate_hz"])
    output = current_output
    while body.next_fsw_sample_s <= time_s + 1e-12:
        channel_count = int(scenario["sensors"].get("channel_count", 1))
        frames = [
            _sensor_frame(
                body,
                scenario,
                rng,
                body.next_fsw_sample_s,
                fsw_step_s,
                launch_position,
                separated,
                channel,
            )
            for channel in range(channel_count)
        ]
        frame = frames[0]
        propulsion = body.stage["propulsion"]
        burn_duration_s = float(propulsion["burn_duration_s"])
        propulsion_running = (
            body.engine_started_s is not None
            and body.next_fsw_sample_s
                < body.engine_started_s + burn_duration_s
        )
        output = core.step(
            frame,
            propulsion_running=propulsion_running,
            drogue_deployed=body.drogue_deployed,
            main_deployed=body.main_deployed,
            previous_execution_time_s=0.0,
            deadline_missed=False,
            sensor_channels=frames[1:],
        )
        if on_sensor:
            on_sensor(body.name, frames, output)
        body.next_fsw_sample_s += fsw_step_s
    return output


def _ramp(time_since_start: float, burn_duration: float) -> float:
    ramp_duration = min(0.35, burn_duration * 0.08)
    return max(
        0.0,
        min(
            1.0,
            time_since_start / ramp_duration,
            (burn_duration - time_since_start) / ramp_duration,
        ),
    )


def _propulsion(
    body: Body,
    output: FswOutput,
    time_s: float,
    dt_s: float,
    scenario: dict[str, Any],
) -> tuple[float, float, float]:
    propulsion = body.stage["propulsion"]
    engines = _stage_engines(body.stage)
    body.last_engine_thrusts_n = {engine["id"]: 0.0 for engine in engines}
    should_burn = False
    if body.stage_index == 0 and body.upper_mass_kg > 0.0:
        if output.stage1_ignite and body.engine_started_s is None:
            body.engine_started_s = time_s
        should_burn = body.engine_started_s is not None
    if body.stage_index == 1 and output.stage2_ignite:
        if body.engine_started_s is None:
            body.engine_started_s = time_s
        should_burn = True
    if output.abort:
        should_burn = False
    start = body.engine_started_s or 0.0
    elapsed = time_s - start
    duration = float(propulsion["burn_duration_s"])
    if not should_burn or elapsed < 0.0 or elapsed >= duration:
        return 0.0, 0.0, 293.15
    curve = propulsion.get("performance_curve")
    if curve:
        times = [row["time_s"] for row in curve]
        base_fuel_flow = float(
            np.interp(elapsed, times, [row["fuel_flow_kg_s"] for row in curve])
        )
        base_oxidizer_flow = float(
            np.interp(elapsed, times, [row["oxidizer_flow_kg_s"] for row in curve])
        )
        base_thrust = float(np.interp(elapsed, times, [row["thrust_n"] for row in curve]))
        chamber_pressure = float(
            np.interp(elapsed, times, [row["chamber_pressure_pa"] for row in curve])
        )
        temperature = float(
            np.interp(elapsed, times, [row["temperature_k"] for row in curve])
        )
    else:
        ramp = _ramp(elapsed, duration)
        base_fuel_flow = float(propulsion["fuel_flow_kg_s"]) * ramp
        base_oxidizer_flow = float(propulsion["oxidizer_flow_kg_s"]) * ramp
        base_thrust = (
            (base_fuel_flow + base_oxidizer_flow)
            * float(propulsion["c_star_m_s"])
            * float(propulsion["thrust_coefficient"])
            * float(propulsion["nozzle_efficiency"])
        )
        chamber_pressure = float(propulsion["chamber_pressure_pa"]) * ramp
        temperature = 293.15 + (
            float(propulsion["combustion_temperature_k"]) - 293.15
        ) * ramp

    flow_scale = 0.0
    thrust_scales: dict[str, float] = {}
    temperature_increase = 0.0
    for engine in engines:
        scale = (
            float(engine.get("performance_scale", 1.0))
            if engine.get("enabled", True)
            else 0.0
        )
        fault = _engine_fault_active(scenario, body.name, engine["id"], time_s)
        if fault and fault.get("type") == "cutoff":
            scale = 0.0
        flow_scale += scale
        thrust_scale = scale
        if fault and fault.get("type") == "thrust_scale":
            thrust_scale *= max(float(fault.get("value", 1.0)), 0.0)
        if fault and fault.get("type") == "overtemperature":
            temperature_increase = max(
                temperature_increase, max(float(fault.get("value", 0.0)), 0.0)
            )
        thrust_scales[engine["id"]] = thrust_scale

    if flow_scale <= 0.0:
        return 0.0, 0.0, 293.15

    requested_fuel_flow = base_fuel_flow * flow_scale
    requested_oxidizer_flow = base_oxidizer_flow * flow_scale
    availability = 1.0
    if requested_fuel_flow > 0.0:
        availability = min(availability, body.fuel_kg / (requested_fuel_flow * dt_s))
    if requested_oxidizer_flow > 0.0:
        availability = min(
            availability, body.oxidizer_kg / (requested_oxidizer_flow * dt_s)
        )
    availability = min(max(availability, 0.0), 1.0)
    body.fuel_kg -= requested_fuel_flow * availability * dt_s
    body.oxidizer_kg -= requested_oxidizer_flow * availability * dt_s
    body.last_engine_thrusts_n = {
        engine_id: base_thrust * scale * availability
        for engine_id, scale in thrust_scales.items()
    }
    thrust = sum(body.last_engine_thrusts_n.values())
    temperature += temperature_increase
    pressure_over = max(chamber_pressure / 2_500_000.0 - 0.8, 0.0)
    temperature_over = max(temperature / 3_600.0 - 0.85, 0.0)
    body.engine_health_percent = max(
        0.0,
        body.engine_health_percent
        - dt_s * (0.08 + pressure_over**2 + temperature_over**2),
    )
    return thrust, chamber_pressure, temperature


def _actuator_commands(
    output: FswOutput,
    scenario: dict[str, Any],
    body: Body,
    time_s: float,
    dt_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    max_tvc = math.radians(scenario["actuators"]["max_tvc_deg"])
    max_fin = math.radians(scenario["actuators"]["max_fin_deg"])
    movable_fins_enabled = body.stage.get("aerodynamics", {}).get(
        "movable_fins_enabled", False
    )
    target_tvc = np.clip(
        np.array([output.tvc_pitch_rad, output.tvc_yaw_rad]), -max_tvc, max_tvc
    )
    target_fin_rad = (
        np.clip(
            np.array(
                [output.fin_roll_rad, output.fin_pitch_rad, output.fin_yaw_rad]
            ),
            -max_fin,
            max_fin,
        )
        if movable_fins_enabled
        else np.zeros(3)
    )
    tvc_fault = _fault_active(scenario, body.name, "tvc", time_s)
    fin_fault = _fault_active(scenario, body.name, "fin", time_s)
    if tvc_fault and tvc_fault.get("type") == "stuck":
        target_tvc[:] = math.radians(float(tvc_fault.get("value_deg", 0.0)))
    if movable_fins_enabled and fin_fault and fin_fault.get("type") == "stuck":
        target_fin_rad[:] = math.radians(float(fin_fault.get("value_deg", 0.0)))
    max_delta = math.radians(scenario["actuators"]["max_rate_deg_s"]) * dt_s
    tvc = body.last_tvc_rad + np.clip(
        target_tvc - body.last_tvc_rad, -max_delta, max_delta
    )
    fin_rad = (
        body.last_fin_rad
        + np.clip(target_fin_rad - body.last_fin_rad, -max_delta, max_delta)
        if movable_fins_enabled
        else np.zeros(3)
    )
    body.last_tvc_rad = tvc
    body.last_fin_rad = fin_rad
    return tvc, fin_rad


def _forces(
    body: Body,
    scenario: dict[str, Any],
    state: np.ndarray,
    thrust_n: float,
    tvc_rad: np.ndarray,
    fin_commands: np.ndarray,
    time_s: float,
) -> tuple[np.ndarray, AeroResult, np.ndarray]:
    position = state[0:3]
    velocity = state[3:6]
    quaternion = quat_normalize(state[6:10])
    rates = state[10:13]
    launch_altitude = float(scenario["environment"]["launch_altitude_m"])
    altitude = max(float(np.linalg.norm(position) - EARTH_RADIUS_M), launch_altitude)
    density, _pressure, sound_speed = atmosphere(altitude)
    wind_ned = np.asarray(scenario["environment"]["wind_ned_m_s"], dtype=float)
    wind_ecef = ned_to_ecef(wind_ned, position)
    relative_ecef = velocity - wind_ecef
    relative_body = quat_rotate(quat_conjugate(quaternion), relative_ecef)
    aero = estimate(
        relative_body,
        rates,
        density,
        sound_speed,
        body.aerodynamic_stage(),
        fin_commands,
    )
    pitch, yaw = tvc_rad
    cg = float(body.aerodynamic_stage()["center_of_mass_m"])
    thrust_body = np.zeros(3)
    thrust_moment = np.zeros(3)
    for engine in _stage_engines(body.stage):
        engine_thrust = body.last_engine_thrusts_n.get(engine["id"], 0.0)
        direction = np.asarray(engine["direction_body"], dtype=float)
        if engine.get("gimbal_enabled", True):
            direction = direction + np.array([0.0, -yaw, pitch])
        direction = unit(direction, np.array([1.0, 0.0, 0.0]))
        engine_force = engine_thrust * direction
        thrust_body += engine_force
        position = np.asarray(engine["position_body_m"], dtype=float)
        lever = position - np.array([cg, 0.0, 0.0])
        thrust_moment += cross3(lever, engine_force)
    force_body = aero.force_body_n + thrust_body
    force_ecef = quat_rotate(quaternion, force_body)

    recovery = body.stage["recovery"]
    parachute_area = 0.0
    if body.main_deployed:
        parachute_area = float(recovery["main_area_m2"])
    elif body.drogue_deployed:
        parachute_area = float(recovery["drogue_area_m2"])
    if parachute_area > 0.0:
        delay = float(recovery.get("inflation_delay_s", 0.0))
        inflation = (
            min(max((time_s - (body.parachute_deployed_s or time_s)) / max(delay, 0.1), 0.0), 1.0)
            if delay > 0.0
            else 1.0
        )
        speed = float(np.linalg.norm(relative_ecef))
        force_ecef += (
            -0.5
            * density
            * speed**2
            * 1.5
            * parachute_area
            * inflation
            * unit(relative_ecef)
        )

    return force_ecef, aero, aero.moment_body_nm + thrust_moment


def _derivative(
    body: Body,
    scenario: dict[str, Any],
    state: np.ndarray,
    thrust_n: float,
    tvc_rad: np.ndarray,
    fin_commands: np.ndarray,
    time_s: float,
) -> np.ndarray:
    position = state[0:3]
    velocity = state[3:6]
    quaternion = quat_normalize(state[6:10])
    rates = state[10:13]
    force_ecef, _aero, moment_body = _forces(
        body, scenario, state, thrust_n, tvc_rad, fin_commands, time_s
    )
    radius = max(float(np.linalg.norm(position)), EARTH_RADIUS_M)
    earth_rate = np.array([0.0, 0.0, EARTH_ROTATION_RAD_S])
    gravity = -EARTH_MU * position / radius**3
    rotating_terms = -2.0 * cross3(earth_rate, velocity) - cross3(
        earth_rate, cross3(earth_rate, position)
    )
    acceleration = gravity + rotating_terms + force_ecef / body.mass_kg
    inertia = body.inertia_kg_m2
    rates_dot = (
        moment_body - cross3(rates, inertia * rates)
    ) / inertia
    return np.concatenate(
        (velocity, acceleration, quat_derivative(quaternion, rates), rates_dot)
    )


def _integrate_body(
    body: Body,
    scenario: dict[str, Any],
    thrust_n: float,
    tvc_rad: np.ndarray,
    fin_commands: np.ndarray,
    time_s: float,
    dt_s: float,
    launch_position: np.ndarray,
) -> AeroResult:
    if body.landed:
        return AeroResult(np.zeros(3), np.zeros(3), 0.0, 0.0, 0.0, True)
    state = np.concatenate(
        (
            body.position_ecef_m,
            body.velocity_ecef_m_s,
            body.attitude_wxyz,
            body.body_rates_rad_s,
        )
    )
    derivative = lambda value, offset: _derivative(  # noqa: E731
        body,
        scenario,
        value,
        thrust_n,
        tvc_rad,
        fin_commands,
        time_s + offset,
    )
    k1 = derivative(state, 0.0)
    k2 = derivative(state + 0.5 * dt_s * k1, 0.5 * dt_s)
    k3 = derivative(state + 0.5 * dt_s * k2, 0.5 * dt_s)
    k4 = derivative(state + dt_s * k3, dt_s)
    next_state = state + dt_s * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
    next_state[6:10] = quat_normalize(next_state[6:10])

    rail_length = 12.0
    relative_altitude = float(
        np.linalg.norm(next_state[0:3]) - np.linalg.norm(launch_position)
    )
    if relative_altitude < rail_length and body.upper_mass_kg > 0.0:
        axis = quat_rotate(next_state[6:10], np.array([1.0, 0.0, 0.0]))
        next_state[3:6] = max(float(np.dot(next_state[3:6], axis)), 0.0) * axis
        next_state[10:13] = 0.0

    body.position_ecef_m = next_state[0:3]
    body.velocity_ecef_m_s = next_state[3:6]
    body.attitude_wxyz = next_state[6:10]
    body.body_rates_rad_s = next_state[10:13]
    force_ecef, aero, _moment = _forces(
        body, scenario, next_state, thrust_n, tvc_rad, fin_commands, time_s + dt_s
    )
    body.last_specific_force_body_m_s2 = quat_rotate(
        quat_conjugate(body.attitude_wxyz), force_ecef / body.mass_kg
    )
    body.last_dynamic_pressure_pa = aero.dynamic_pressure_pa
    body.last_mach = aero.mach
    body.last_angle_of_attack_deg = aero.angle_of_attack_deg
    body.aero_valid = aero.valid

    if (
        np.linalg.norm(body.position_ecef_m) <= np.linalg.norm(launch_position)
        and np.dot(body.velocity_ecef_m_s, unit(body.position_ecef_m)) < 0.0
        and time_s > 2.0
    ):
        body.position_ecef_m = unit(body.position_ecef_m) * np.linalg.norm(launch_position)
        body.velocity_ecef_m_s[:] = 0.0
        body.body_rates_rad_s[:] = 0.0
        body.landed = True
    return aero


def _split_stack(
    integrated_stack: Body, scenario: dict[str, Any]
) -> tuple[Body, Body]:
    stages = scenario["vehicle"]["stages"]
    axis = quat_rotate(integrated_stack.attitude_wxyz, np.array([1.0, 0.0, 0.0]))
    impulse = float(scenario["mission"].get("separation_impulse_m_s", 0.8))
    core_stage = Body(
        "core_stage",
        0,
        stages[0],
        integrated_stack.position_ecef_m - 0.5 * axis,
        integrated_stack.velocity_ecef_m_s - impulse * axis,
        integrated_stack.attitude_wxyz.copy(),
        integrated_stack.body_rates_rad_s.copy(),
        integrated_stack.fuel_kg,
        integrated_stack.oxidizer_kg,
    )
    upper_stage = Body(
        "upper_stage",
        1,
        stages[1],
        integrated_stack.position_ecef_m + 0.5 * axis,
        integrated_stack.velocity_ecef_m_s + impulse * axis,
        integrated_stack.attitude_wxyz.copy(),
        integrated_stack.body_rates_rad_s.copy(),
        float(stages[1]["fuel_mass_kg"]),
        float(stages[1]["oxidizer_mass_kg"]),
    )
    return core_stage, upper_stage


def _consume_discrete_actuation(
    body: Body,
    output: FswOutput,
    action: int,
) -> bool:
    command = output.discrete_actuation
    if (
        not command.valid
        or int(command.action) != action
        or int(command.sequence) <= body.last_discrete_actuation_sequence
    ):
        return False
    body.last_discrete_actuation_sequence = int(command.sequence)
    return True


def _telemetry_row(
    time_s: float,
    body: Body,
    output: FswOutput,
    thrust_n: float,
    pressure_pa: float,
    temperature_k: float,
    launch_position: np.ndarray,
) -> dict[str, Any]:
    relative_altitude = float(
        np.linalg.norm(body.position_ecef_m) - np.linalg.norm(launch_position)
    )
    velocity_ned = ecef_to_ned(body.velocity_ecef_m_s, body.position_ecef_m)
    euler_deg = np.degrees(quat_to_euler(body.attitude_wxyz))
    return {
        "time_s": round(time_s, 6),
        "body": body.name,
        "mode": MODE_NAMES[output.mode] if 0 <= output.mode < len(MODE_NAMES) else "UNKNOWN",
        "altitude_m": relative_altitude,
        "position_ecef_x_m": body.position_ecef_m[0],
        "position_ecef_y_m": body.position_ecef_m[1],
        "position_ecef_z_m": body.position_ecef_m[2],
        "velocity_north_m_s": velocity_ned[0],
        "velocity_east_m_s": velocity_ned[1],
        "velocity_down_m_s": velocity_ned[2],
        "speed_m_s": float(np.linalg.norm(body.velocity_ecef_m_s)),
        "attitude_w": body.attitude_wxyz[0],
        "attitude_x": body.attitude_wxyz[1],
        "attitude_y": body.attitude_wxyz[2],
        "attitude_z": body.attitude_wxyz[3],
        "roll_deg": euler_deg[0],
        "pitch_deg": euler_deg[1],
        "yaw_deg": euler_deg[2],
        "rate_x_rad_s": body.body_rates_rad_s[0],
        "rate_y_rad_s": body.body_rates_rad_s[1],
        "rate_z_rad_s": body.body_rates_rad_s[2],
        "mass_kg": body.mass_kg,
        "propellant_fraction": body.propellant_fraction,
        "center_of_mass_m": body.center_of_mass_m,
        "inertia_x_kg_m2": body.inertia_kg_m2[0],
        "inertia_y_kg_m2": body.inertia_kg_m2[1],
        "inertia_z_kg_m2": body.inertia_kg_m2[2],
        "thrust_n": thrust_n,
        "chamber_pressure_pa": pressure_pa,
        "chamber_temperature_k": temperature_k,
        "engine_health_percent": body.engine_health_percent,
        "engine_count": len(_stage_engines(body.stage)),
        "active_engines": sum(
            thrust > 1e-6 for thrust in body.last_engine_thrusts_n.values()
        ),
        "engine_thrusts_n": json.dumps(body.last_engine_thrusts_n, sort_keys=True),
        "dynamic_pressure_pa": body.last_dynamic_pressure_pa,
        "mach": body.last_mach,
        "angle_of_attack_deg": body.last_angle_of_attack_deg,
        "fsw_stage_separate": int(output.stage_separate),
        "fsw_stage2_ignite": int(output.stage2_ignite),
        "fsw_deploy_drogue": int(output.deploy_drogue),
        "fsw_deploy_main": int(output.deploy_main),
        "fsw_abort": int(output.abort),
        "fsw_estimated_altitude_m": output.estimated_altitude_m,
        "fsw_estimated_vertical_velocity_m_s": output.estimated_vertical_velocity_m_s,
        "fsw_estimated_attitude_w": output.estimated_attitude_wxyz[0],
        "fsw_estimated_attitude_x": output.estimated_attitude_wxyz[1],
        "fsw_estimated_attitude_y": output.estimated_attitude_wxyz[2],
        "fsw_estimated_attitude_z": output.estimated_attitude_wxyz[3],
        "fsw_tvc_pitch_deg": math.degrees(output.tvc_pitch_rad),
        "fsw_tvc_yaw_deg": math.degrees(output.tvc_yaw_rad),
        "fsw_fin_roll_deg": math.degrees(output.fin_roll_rad),
        "fsw_fin_pitch_deg": math.degrees(output.fin_pitch_rad),
        "fsw_fin_yaw_deg": math.degrees(output.fin_yaw_rad),
        "fsw_navigation_status": NAVIGATION_STATUS_NAMES[output.navigation_status],
        "fsw_fault_flags": int(
            output.active_fault_flags | output.latched_fault_flags
        ),
        "fsw_faults": decode_faults(
            output.active_fault_flags | output.latched_fault_flags
        ),
        "fsw_active_fault_flags": int(output.active_fault_flags),
        "fsw_latched_fault_flags": int(output.latched_fault_flags),
        "fsw_highest_fault_severity": int(output.highest_fault_severity),
        "fsw_altitude_sigma_m": output.altitude_sigma_m,
        "fsw_vertical_velocity_sigma_m_s": output.vertical_velocity_sigma_m_s,
        "fsw_command_sequence": int(output.command_sequence),
        "fsw_command_type": int(output.command_type),
        "fsw_command_result": int(output.command_result),
        "fsw_inhibit_flags": int(output.inhibit_flags),
        "tvc_pitch_deg": math.degrees(body.last_tvc_rad[0]),
        "tvc_yaw_deg": math.degrees(body.last_tvc_rad[1]),
        "fin_roll_deg": math.degrees(body.last_fin_rad[0]),
        "fin_pitch_deg": math.degrees(body.last_fin_rad[1]),
        "fin_yaw_deg": math.degrees(body.last_fin_rad[2]),
        "drogue_deployed": int(body.drogue_deployed),
        "main_deployed": int(body.main_deployed),
        "landed": int(body.landed),
        "aero_valid": int(body.aero_valid),
    }


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run_simulation(
    scenario: dict[str, Any],
    seed: int | None = None,
    output_root: str | Path = "runs",
    create_report: bool = True,
    persist: bool = True,
    summary_only: bool = False,
    on_sample: Callable[
        [float, list[dict[str, Any]], list[dict[str, Any]]], None
    ]
    | None = None,
    should_cancel: Callable[[], bool] | None = None,
) -> RunResult:
    if summary_only and persist:
        raise ValueError("summary_only cannot persist telemetry")
    validate_scenario(scenario)
    if seed is None:
        seed = int(scenario.get("simulation", {}).get("seed", 1))
    rng = np.random.default_rng(seed)
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = Path(output_root) / f"{run_id}-{scenario_hash(scenario)[:8]}-s{seed}"
    suffix = 1
    if persist:
        while output_dir.exists():
            output_dir = output_dir.with_name(f"{output_dir.name}-{suffix}")
            suffix += 1
        output_dir.mkdir(parents=True)

    environment = scenario["environment"]
    stages = scenario["vehicle"]["stages"]
    dt_s = float(scenario["simulation"]["time_step_s"])
    max_time_s = float(scenario["simulation"]["max_time_s"])
    output_interval = 1.0 / float(scenario["simulation"]["output_rate_hz"])
    launch_position = geodetic_to_ecef(
        environment["latitude_deg"],
        environment["longitude_deg"],
        environment["launch_altitude_m"],
    )
    integrated_stack = Body(
        "integrated_stack",
        0,
        stages[0],
        launch_position.copy(),
        np.zeros(3),
        initial_attitude(launch_position, environment["launch_azimuth_deg"]),
        np.zeros(3),
        float(stages[0]["fuel_mass_kg"]),
        float(stages[0]["oxidizer_mass_kg"]),
        _stage_total_mass(stages[1]),
        float(stages[0]["length_m"] + stages[1]["length_m"]),
    )
    bodies = [integrated_stack]
    cores: dict[str, FlightCore] = {
        "integrated_stack": FlightCore(scenario, FSW_BODY_INTEGRATED)
    }
    outputs: dict[str, FswOutput] = {"integrated_stack": FswOutput()}
    telemetry: list[dict[str, Any]] = []
    fsw_rows: list[dict[str, Any]] = []
    summary_statistics = {
        "telemetry_samples": 0,
        "aero_out_of_envelope_samples": 0,
        "aero_out_of_envelope_pre_recovery_samples": 0,
        "maximum_altitude_m": 0.0,
        "maximum_mach": 0.0,
        "maximum_dynamic_pressure_pa": 0.0,
        "maximum_angle_of_attack_deg": 0.0,
        "minimum_engine_health_percent": 100.0,
        "altitude_error_squared_sum": 0.0,
        "finite_positive_state": True,
    }
    events: list[dict[str, Any]] = [
        {
            "time_s": 0.0,
            "body": "integrated_stack",
            "event": "simulation_started",
            "detail": "",
        }
    ]
    separated = False
    next_output_s = 0.0
    previous_modes: dict[str, int] = {}
    time_s = 0.0
    cancelled = False
    sensor_file = None
    sensor_writer = None
    command_file = None
    command_writer = None
    if persist:
        sensor_file = gzip.open(
            output_dir / "sensors.csv.gz",
            "wt",
            newline="",
            encoding="utf-8",
            compresslevel=1,
        )
        sensor_writer = csv.DictWriter(sensor_file, fieldnames=SENSOR_CSV_FIELDS)
        sensor_writer.writeheader()
        command_file = (output_dir / "commands.csv").open(
            "w", newline="", encoding="utf-8"
        )
        command_writer = csv.DictWriter(
            command_file,
            fieldnames=(
                "body",
                "time_s",
                "sequence",
                "command_type",
                "result",
                "inhibit_flags",
            ),
        )
        command_writer.writeheader()

    def record_sensor(
        body_name: str, frames: list[SensorFrame], output: FswOutput
    ) -> None:
        if sensor_writer is not None:
            for channel, frame in enumerate(frames):
                sensor_writer.writerow(
                    sensor_frame_to_row(body_name, frame, channel)
                )
        if command_writer is not None and output.event_flags & (1 << 2):
            frame = frames[0]
            command_writer.writerow(
                {
                    "body": body_name,
                    "time_s": frame.time_s,
                    "sequence": int(output.command_sequence),
                    "command_type": int(output.command_type),
                    "result": int(output.command_result),
                    "inhibit_flags": int(output.inhibit_flags),
                }
            )

    try:
        while time_s <= max_time_s:
            if should_cancel and should_cancel():
                cancelled = True
                events.append(
                    {
                        "time_s": time_s,
                        "body": "integrated_stack" if not separated else "all",
                        "event": "simulation_cancelled",
                        "detail": "operator request",
                    }
                )
                break
            for body in list(bodies):
                output = _run_fsw_substeps(
                    cores[body.name],
                    body,
                    scenario,
                    rng,
                    time_s,
                    launch_position,
                    separated,
                    outputs[body.name],
                    record_sensor,
                )
                outputs[body.name] = output
                previous_mode = previous_modes.get(body.name)
                if previous_mode != output.mode:
                    mode_name = MODE_NAMES[output.mode]
                    events.append(
                        {
                            "time_s": time_s,
                            "body": body.name,
                            "event": "flight_mode",
                            "detail": mode_name,
                        }
                    )
                    if (
                        previous_mode is not None
                        and MODE_NAMES[previous_mode] == "BOOST_1"
                        and mode_name == "SEPARATION"
                    ):
                        events.append(
                            {
                                "time_s": time_s,
                                "body": body.name,
                                "event": "burnout_stage_1",
                                "detail": "",
                            }
                        )
                    if mode_name == "BOOST_2":
                        events.append(
                            {
                                "time_s": time_s,
                                "body": body.name,
                                "event": "stage2_ignition",
                                "detail": "",
                            }
                        )
                    if (
                        previous_mode is not None
                        and MODE_NAMES[previous_mode] == "BOOST_2"
                        and mode_name == "COAST"
                    ):
                        events.append(
                            {
                                "time_s": time_s,
                                "body": body.name,
                                "event": "burnout_stage_2",
                                "detail": "",
                            }
                        )
                    previous_modes[body.name] = output.mode

            if (
                not separated
                and _consume_discrete_actuation(
                    integrated_stack,
                    outputs["integrated_stack"],
                    FSW_DISCRETE_ACTION_STAGE_SEPARATE,
                )
            ):
                core_stage, upper_stage = _split_stack(integrated_stack, scenario)
                integrated_core = cores.pop("integrated_stack")
                integrated_output = outputs.pop("integrated_stack")
                bodies = [core_stage, upper_stage]
                cores["core_stage"] = FlightCore(scenario, FSW_BODY_CORE)
                cores["upper_stage"] = integrated_core
                outputs["core_stage"] = FswOutput()
                outputs["upper_stage"] = integrated_output
                previous_modes.pop("integrated_stack", None)
                previous_modes["upper_stage"] = integrated_output.mode
                separated = True
                events.append(
                    {
                        "time_s": time_s,
                        "body": "integrated_stack",
                        "event": "stage_separation",
                        "detail": "core stage and upper stage created",
                    }
                )
                core_stage.next_fsw_sample_s = time_s
                upper_stage.next_fsw_sample_s = integrated_stack.next_fsw_sample_s
                outputs["core_stage"] = _run_fsw_substeps(
                    cores["core_stage"],
                    core_stage,
                    scenario,
                    rng,
                    time_s,
                    launch_position,
                    True,
                    outputs["core_stage"],
                    record_sensor,
                )

            current_values: dict[str, tuple[float, float, float]] = {}
            for body in bodies:
                output = outputs[body.name]
                drogue_failed = bool(
                    (
                        fault := _fault_active(
                            scenario, body.name, "drogue", time_s
                        )
                    )
                    and fault.get("type") == "failed"
                )
                main_failed = bool(
                    (
                        fault := _fault_active(
                            scenario, body.name, "main", time_s
                        )
                    )
                    and fault.get("type") == "failed"
                )
                deploy_drogue = _consume_discrete_actuation(
                    body,
                    output,
                    FSW_DISCRETE_ACTION_DEPLOY_DROGUE,
                )
                deploy_main = _consume_discrete_actuation(
                    body,
                    output,
                    FSW_DISCRETE_ACTION_DEPLOY_MAIN,
                )
                if deploy_drogue and not body.drogue_deployed and not drogue_failed:
                    body.drogue_deployed = True
                    body.parachute_deployed_s = time_s
                    events.append(
                        {
                            "time_s": time_s,
                            "body": body.name,
                            "event": "drogue_deployed",
                            "detail": "",
                        }
                    )
                if deploy_main and not body.main_deployed and not main_failed:
                    body.main_deployed = True
                    body.parachute_deployed_s = time_s
                    events.append(
                        {
                            "time_s": time_s,
                            "body": body.name,
                            "event": "main_deployed",
                            "detail": "",
                        }
                    )
                tvc, fins = _actuator_commands(
                    output, scenario, body, time_s, dt_s
                )
                thrust, pressure, temperature = _propulsion(
                    body, output, time_s, dt_s, scenario
                )
                was_landed = body.landed
                _integrate_body(
                    body,
                    scenario,
                    thrust,
                    tvc,
                    fins,
                    time_s,
                    dt_s,
                    launch_position,
                )
                if body.landed and not was_landed:
                    events.append(
                        {
                            "time_s": time_s,
                            "body": body.name,
                            "event": "landed",
                            "detail": "",
                        }
                    )
                current_values[body.name] = (thrust, pressure, temperature)

            if time_s + 1e-12 >= next_output_s:
                new_rows: list[dict[str, Any]] = []
                for body in bodies:
                    thrust, pressure, temperature = current_values[body.name]
                    output = outputs[body.name]
                    row = _telemetry_row(
                        time_s,
                        body,
                        output,
                        thrust,
                        pressure,
                        temperature,
                        launch_position,
                    )
                    fsw_row = {
                        "time_s": round(time_s, 6),
                        "body": body.name,
                        "mode": MODE_NAMES[output.mode],
                        "estimated_altitude_m": output.estimated_altitude_m,
                        "estimated_vertical_velocity_m_s": output.estimated_vertical_velocity_m_s,
                        "estimated_position_ecef_x_m": (
                            output.estimated_position_ecef_m[0]
                        ),
                        "estimated_position_ecef_y_m": (
                            output.estimated_position_ecef_m[1]
                        ),
                        "estimated_position_ecef_z_m": (
                            output.estimated_position_ecef_m[2]
                        ),
                        "estimated_velocity_ecef_x_m_s": (
                            output.estimated_velocity_ecef_m_s[0]
                        ),
                        "estimated_velocity_ecef_y_m_s": (
                            output.estimated_velocity_ecef_m_s[1]
                        ),
                        "estimated_velocity_ecef_z_m_s": (
                            output.estimated_velocity_ecef_m_s[2]
                        ),
                        "estimated_attitude_w": output.estimated_attitude_wxyz[0],
                        "estimated_attitude_x": output.estimated_attitude_wxyz[1],
                        "estimated_attitude_y": output.estimated_attitude_wxyz[2],
                        "estimated_attitude_z": output.estimated_attitude_wxyz[3],
                        "navigation_status": NAVIGATION_STATUS_NAMES[
                            output.navigation_status
                        ],
                        "fault_flags": int(
                            output.active_fault_flags
                            | output.latched_fault_flags
                        ),
                        "active_fault_flags": int(output.active_fault_flags),
                        "latched_fault_flags": int(output.latched_fault_flags),
                        "changed_fault_flags": int(output.changed_fault_flags),
                        "faults": decode_faults(
                            output.active_fault_flags
                            | output.latched_fault_flags
                        ),
                        "highest_fault_severity": int(
                            output.highest_fault_severity
                        ),
                        "altitude_sigma_m": output.altitude_sigma_m,
                        "vertical_velocity_sigma_m_s": (
                            output.vertical_velocity_sigma_m_s
                        ),
                        "attitude_sigma_x_rad": output.attitude_sigma_rad[0],
                        "attitude_sigma_y_rad": output.attitude_sigma_rad[1],
                        "attitude_sigma_z_rad": output.attitude_sigma_rad[2],
                        "barometer_innovation_m": (
                            output.barometer_innovation_m
                        ),
                        "gnss_altitude_innovation_m": (
                            output.gnss_altitude_innovation_m
                        ),
                        "gnss_velocity_innovation_m_s": (
                            output.gnss_velocity_innovation_m_s
                        ),
                        "command_sequence": int(output.command_sequence),
                        "command_type": int(output.command_type),
                        "command_result": int(output.command_result),
                        "inhibit_flags": int(output.inhibit_flags),
                        "consecutive_overruns": int(
                            output.consecutive_overruns
                        ),
                        "discrete_actuation_sequence": int(
                            output.discrete_actuation.sequence
                        ),
                        "discrete_actuation_action": int(
                            output.discrete_actuation.action
                        ),
                    }
                    summary_statistics["telemetry_samples"] += 1
                    summary_statistics["aero_out_of_envelope_samples"] += int(
                        not row["aero_valid"]
                    )
                    summary_statistics[
                        "aero_out_of_envelope_pre_recovery_samples"
                    ] += int(
                        not row["aero_valid"]
                        and not row["drogue_deployed"]
                        and not row["main_deployed"]
                    )
                    summary_statistics["maximum_altitude_m"] = max(
                        summary_statistics["maximum_altitude_m"],
                        float(row["altitude_m"]),
                    )
                    summary_statistics["maximum_mach"] = max(
                        summary_statistics["maximum_mach"],
                        float(row["mach"]),
                    )
                    summary_statistics["maximum_dynamic_pressure_pa"] = max(
                        summary_statistics["maximum_dynamic_pressure_pa"],
                        float(row["dynamic_pressure_pa"]),
                    )
                    summary_statistics["maximum_angle_of_attack_deg"] = max(
                        summary_statistics["maximum_angle_of_attack_deg"],
                        float(row["angle_of_attack_deg"]),
                    )
                    summary_statistics["minimum_engine_health_percent"] = min(
                        summary_statistics["minimum_engine_health_percent"],
                        float(row["engine_health_percent"]),
                    )
                    altitude_error = float(row["altitude_m"]) - float(
                        fsw_row["estimated_altitude_m"]
                    )
                    summary_statistics["altitude_error_squared_sum"] += (
                        altitude_error * altitude_error
                    )
                    summary_statistics["finite_positive_state"] = bool(
                        summary_statistics["finite_positive_state"]
                        and math.isfinite(float(row["altitude_m"]))
                        and math.isfinite(float(row["mass_kg"]))
                        and float(row["mass_kg"]) > 0.0
                    )
                    if not summary_only:
                        telemetry.append(row)
                        fsw_rows.append(fsw_row)
                    new_rows.append(row)
                if on_sample:
                    on_sample(time_s, new_rows, events)
                next_output_s += output_interval
            if separated and all(body.landed for body in bodies):
                break
            time_s += dt_s
    finally:
        for core in cores.values():
            core.close()
        if sensor_file is not None:
            sensor_file.close()
        if command_file is not None:
            command_file.close()

    invalid_rows = int(
        summary_statistics["aero_out_of_envelope_samples"]
    )
    invalid_pre_recovery_rows = int(
        summary_statistics["aero_out_of_envelope_pre_recovery_samples"]
    )
    invalid_recovery_rows = invalid_rows - invalid_pre_recovery_rows
    landed = {body.name: body.landed for body in bodies}
    impact_points = {}
    for body in bodies:
        if body.landed:
            latitude_deg, longitude_deg, altitude_m = ecef_to_geodetic(
                body.position_ecef_m
            )
            impact_points[body.name] = {
                "latitude_deg": latitude_deg,
                "longitude_deg": longitude_deg,
                "altitude_m": altitude_m,
            }
    maximum_altitude_m = float(
        summary_statistics["maximum_altitude_m"]
    )
    telemetry_samples = int(summary_statistics["telemetry_samples"])
    altitude_rmse_m = math.sqrt(
        float(summary_statistics["altitude_error_squared_sum"])
        / max(telemetry_samples, 1)
    )
    finite = bool(summary_statistics["finite_positive_state"])
    separated_event = any(event["event"] == "stage_separation" for event in events)
    checks = {
        "finite_positive_state": finite,
        "stage_separation": separated_event,
        "core_stage_landed": landed.get("core_stage", False),
        "upper_stage_landed": landed.get("upper_stage", False),
        "altitude_envelope_10_to_100_km": 10_000.0 <= maximum_altitude_m <= 100_000.0,
        "navigation_altitude_rmse_below_25_m": altitude_rmse_m < 25.0,
    }
    manifest = {
        "schema_version": "astara.run.v1",
        "model_version": __version__,
        "model_source_sha256": model_source_hash(),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scenario_name": scenario["name"],
        "scenario_sha256": scenario_hash(scenario),
        "seed": seed,
        "units": "SI",
        "coordinate_frames": "ECEF truth, body attitude quaternion, NED reports",
        "simulation_only": True,
        "unvalidated": True,
        "cancelled": cancelled,
        "aero_out_of_envelope_samples": invalid_rows,
        "aero_out_of_envelope_pre_recovery_samples": invalid_pre_recovery_rows,
        "aero_out_of_envelope_recovery_samples": invalid_recovery_rows,
        "landed": landed,
        "impact_points": impact_points,
        "duration_s": time_s,
        "maximum_altitude_m": maximum_altitude_m,
        "summary_metrics": {
            "telemetry_samples": telemetry_samples,
            "maximum_mach": summary_statistics["maximum_mach"],
            "maximum_dynamic_pressure_pa": summary_statistics[
                "maximum_dynamic_pressure_pa"
            ],
            "maximum_angle_of_attack_deg": summary_statistics[
                "maximum_angle_of_attack_deg"
            ],
            "minimum_engine_health_percent": summary_statistics[
                "minimum_engine_health_percent"
            ],
        },
        "navigation_altitude_rmse_m": altitude_rmse_m,
        "checks": checks,
        "model_configuration": {
            "propulsion": [
                "tabulated_curve"
                if stage["propulsion"].get("performance_curve")
                else "reduced_order"
                for stage in stages
            ],
            "aerodynamics": [
                "mach_coefficient_table"
                if stage.get("aerodynamics", {}).get("coefficient_table")
                else "geometry_estimate"
                for stage in stages
            ],
            "mass_properties": [
                "propellant_fraction_table"
                if stage.get("mass_properties")
                else "fixed"
                for stage in stages
            ],
            "engine_counts": [len(_stage_engines(stage)) for stage in stages],
        },
        "credibility": scenario.get(
            "credibility",
            {
                "status": "UNSPECIFIED",
                "intended_use": "No intended use was declared by the scenario.",
            },
        ),
        "result_interpretation": (
            "PASS/FAIL covers numerical mission checks only; it is not model validation "
            "or flight certification."
        ),
        "status": (
            "CANCELLED"
            if cancelled
            else (
                "PASS_WITH_MODEL_WARNINGS"
                if all(checks.values()) and invalid_rows
                else ("PASS" if all(checks.values()) else "FAIL")
            )
        ),
    }
    if persist:
        scenario_document, vehicle_document = evidence_documents(scenario)
        (output_dir / "scenario.json").write_text(
            json.dumps(scenario_document, indent=2), encoding="utf-8"
        )
        if vehicle_document is not None:
            (output_dir / "vehicle_definition.json").write_text(
                json.dumps(vehicle_document, indent=2), encoding="utf-8"
            )
        (output_dir / "manifest.json").write_text(
            json.dumps(manifest, indent=2), encoding="utf-8"
        )
        _write_csv(output_dir / "truth.csv", telemetry)
        _write_csv(output_dir / "fsw.csv", fsw_rows)
        _write_csv(output_dir / "events.csv", events)
        artifact_names = [
            "scenario.json",
            "sensors.csv.gz",
            "commands.csv",
            "truth.csv",
            "fsw.csv",
            "events.csv",
        ]
        if vehicle_document is not None:
            artifact_names.append("vehicle_definition.json")
        for filename in artifact_names:
            path = output_dir / filename
            manifest.setdefault("artifacts", {})[filename] = hashlib.sha256(
                path.read_bytes()
            ).hexdigest()
        (output_dir / "manifest.json").write_text(
            json.dumps(manifest, indent=2), encoding="utf-8"
        )
    result = RunResult(output_dir, manifest, telemetry, fsw_rows, events)
    rocketpy_config = scenario.get("reference_backends", {}).get("rocketpy", {})
    if (
        create_report
        and persist
        and not cancelled
        and rocketpy_config.get("enabled", False)
    ):
        try:
            from .rocketpy_adapter import run_rocketpy_reference

            reference = run_rocketpy_reference(scenario, telemetry, output_dir)
        except Exception as error:
            manifest.setdefault("reference_backends", {})["rocketpy"] = {
                "status": "UNAVAILABLE",
                "error": f"{type(error).__name__}: {error}",
            }
        else:
            manifest.setdefault("reference_backends", {})["rocketpy"] = {
                "status": reference["status"],
                "artifact": "rocketpy_reference.json",
            }
            path = output_dir / "rocketpy_reference.json"
            manifest.setdefault("artifacts", {})[path.name] = hashlib.sha256(
                path.read_bytes()
            ).hexdigest()
        (output_dir / "manifest.json").write_text(
            json.dumps(manifest, indent=2), encoding="utf-8"
        )
    if create_report and persist and not cancelled:
        from .reporting import create_report_artifacts

        create_report_artifacts(result)
    return result
