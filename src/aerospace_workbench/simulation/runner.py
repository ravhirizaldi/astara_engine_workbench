"""Deterministic 6-DOF software-in-the-loop mission simulation."""

from __future__ import annotations

import copy
import csv
import ctypes
import gzip
import json
import math
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import numpy as np

from .. import __version__
from .aerodynamics import AeroResult, atmosphere, estimate
from ..configuration.scenarios import (
    model_source_hash,
    scenario_hash,
)
from ..configuration.schemas import RUN_SCHEMA_VERSION
from ..configuration.validation import validate_scenario
from ..evidence.artifacts import (
    sha256_file,
    write_configuration_artifacts,
    write_csv,
    write_json,
)
from ..evidence.manifest import register_artifacts, write_manifest
from ..flight_software.abi import (
    FSW_BODY_CORE,
    FSW_BODY_INTEGRATED,
    FSW_DISCRETE_ACTION_DEPLOY_DROGUE,
    FSW_DISCRETE_ACTION_DEPLOY_MAIN,
    FSW_DISCRETE_ACTION_STAGE_SEPARATE,
    FSW_COMMAND_NONE,
    MODE_NAMES,
    NAVIGATION_STATUS_NAMES,
    FswAirDataSample,
    FswDiscreteInputs,
    FswDiscreteSample,
    FswOutput,
    FswPlatformStatus,
    FswPropulsionStatus,
    SensorFrame,
    decode_faults,
)
from ..flight_software.bridge import (
    SENSOR_CSV_FIELDS,
    FlightCore,
    FswDeviceInputs,
    fsw_sensor_diagnostics_to_row,
    sensor_frame_to_row,
)
from ..flight_software.timing import (
    FSW_TIMING_MODES,
    validate_timing_options,
)
from ..mathematics.frames import (
    EARTH_MU,
    EARTH_RADIUS_M,
    EARTH_ROTATION_RAD_S,
    ecef_to_geodetic,
    ecef_to_ned,
    geodetic_to_ecef,
    initial_attitude,
    ned_to_ecef,
)
from ..mathematics.quaternions import (
    quat_conjugate,
    quat_derivative,
    quat_normalize,
    quat_rotate,
    quat_to_euler,
)
from ..mathematics.vectors import (
    cross3,
    unit,
)
from .events import event, flight_mode_events
from .device_models import (
    AirDataComputerModel,
    DiscreteInputModule,
    EngineControllerModel,
    FlightComputerPlatformModel,
    RecoveryControllerModel,
)
from .actuators import (
    actuator_commands as _actuator_commands,
    consume_discrete_actuation as _consume_discrete_actuation,
)
from .sensors import (
    SensorChannelState,
    apply_sensor_faults as _apply_sensor_faults,
    fault_active as _fault_active,
    sensor_faults as _sensor_faults,
)
from .propulsion import (
    propulsion_step as _propulsion,
    stage_engines as _stage_engines,
)
from .scheduling import (
    BusScheduler,
    DeviceScheduler,
    EventQueue,
    SimulationClock,
    TaskScheduler,
    TimingProfile,
)
from .truth_model import Body, stage_total_mass as _stage_total_mass


@dataclass
class RunResult:
    output_dir: Path
    manifest: dict[str, Any]
    telemetry: list[dict[str, Any]]
    fsw_telemetry: list[dict[str, Any]]
    events: list[dict[str, Any]]
    avionics_timeline: list[dict[str, Any]] = field(default_factory=list)


AVIONICS_CSV_FIELDS = (
    "body",
    "subsystem",
    "truth_time_s",
    "sensor_sample_time_s",
    "sensor_completion_time_s",
    "bus_publish_time_s",
    "fsw_receive_time_s",
)

DEVICE_MODEL_TYPES = {
    "air_data_computer": AirDataComputerModel,
    "engine_controller": EngineControllerModel,
    "discrete_input_module": DiscreteInputModule,
    "recovery_controller": RecoveryControllerModel,
    "flight_computer_platform": FlightComputerPlatformModel,
}


@dataclass
class _AvionicsRuntime:
    clock: SimulationClock
    queue: EventQueue
    devices: DeviceScheduler
    tasks: TaskScheduler
    bus: BusScheduler
    received: list[dict[str, Any]]
    models: dict[str, Any]
    received_devices: dict[str, dict[str, Any]]
    timeline: list[dict[str, Any]]
    record_timeline: bool
    last_task_time_s: float | None = None
    last_deadline_missed: bool = False
    reported_execution_time_s: float = 0.0


def _scheduler_seed(seed: int, body_name: str, subsystem: str) -> int:
    return seed + sum(
        (index + 1) * ord(character)
        for index, character in enumerate(f"{body_name}:{subsystem}")
    )


def _avionics_runtime(
    scenario: dict[str, Any],
    seed: int,
    body_name: str,
    start_s: float,
    timeline: list[dict[str, Any]],
    initial_received: list[dict[str, Any]] | None = None,
    initial_device_received: dict[str, dict[str, Any]] | None = None,
    record_timeline: bool = True,
) -> _AvionicsRuntime:
    queue = EventQueue()
    devices = DeviceScheduler(queue)
    tasks = TaskScheduler(queue)
    avionics = scenario["avionics"]
    for name, values in avionics["devices"].items():
        devices.add(
            name,
            TimingProfile.from_mapping(values),
            _scheduler_seed(seed, body_name, name),
        )
        devices.start(name, start_s)
    models = {
        name: model_type(
            avionics["devices"][name],
            _scheduler_seed(seed, body_name, f"{name}:model"),
            float(avionics["devices"][name]["reset_epoch_s"]),
        )
        for name, model_type in DEVICE_MODEL_TYPES.items()
    }
    tasks.add(
        "fsw",
        TimingProfile.from_mapping(avionics["tasks"]["fsw"]),
        _scheduler_seed(seed, body_name, "fsw"),
    )
    tasks.start("fsw", start_s)
    return _AvionicsRuntime(
        SimulationClock(start_s),
        queue,
        devices,
        tasks,
        BusScheduler(
            queue,
            "sensor_bus",
            TimingProfile.from_mapping(avionics["buses"]["sensor_bus"]),
            _scheduler_seed(seed, body_name, "sensor_bus"),
        ),
        (
            copy.deepcopy(initial_received)
            if initial_received is not None
            else [
                {}
                for _ in range(
                    int(scenario["sensors"].get("channel_count", 1))
                )
            ]
        ),
        models,
        (
            copy.deepcopy(initial_device_received)
            if initial_device_received is not None
            else {}
        ),
        timeline,
        record_timeline,
    )


def _record_avionics_timeline(
    avionics: _AvionicsRuntime,
    payload: dict[str, Any],
    subsystem: str,
) -> None:
    if avionics.record_timeline:
        avionics.timeline.append(
            {
                field: payload.get(field)
                for field in AVIONICS_CSV_FIELDS
            }
            | {"subsystem": subsystem}
        )


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
    faults = _sensor_faults(scenario, body.name, device, time_s, channel)
    state.fault_state = tuple(str(fault.get("type")) for fault in faults)
    if device == "imu":
        acceleration, imu_time, state.accel_valid = _apply_sensor_faults(
            body.last_specific_force_body_m_s2
            + state.accelerometer_bias_m_s2
            + rng.normal(0.0, sensors["accelerometer_noise_m_s2"], 3),
            state.acceleration_body_m_s2,
            time_s,
            state.imu_sample_time_s,
            faults,
        )
        gyro, imu_time, state.gyro_valid = _apply_sensor_faults(
            body.body_rates_rad_s
            + state.gyro_bias_rad_s
            + rng.normal(0.0, sensors["gyro_noise_rad_s"], 3),
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
        magnetic, sample_time_s, state.magnetometer_valid = _apply_sensor_faults(
            quat_rotate(
                quat_conjugate(body.attitude_wxyz),
                unit(np.array([0.28, 0.08, -0.52])),
            )
            + state.magnetometer_bias
            + rng.normal(0.0, float(sensors["magnetometer_noise"]), 3),
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
        value, sample_time_s, state.barometer_valid = _apply_sensor_faults(
            np.array(
                [
                    altitude
                    + state.barometer_bias_m
                    + rng.normal(0.0, sensors["barometer_noise_m"])
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
        value, sample_time_s, state.gnss_valid = _apply_sensor_faults(
            np.concatenate(
                (
                    body.position_ecef_m
                    + state.gnss_position_bias_m
                    + rng.normal(0.0, sensors["gnss_position_noise_m"], 3),
                    body.velocity_ecef_m_s
                    + state.gnss_velocity_bias_m_s
                    + rng.normal(0.0, sensors["gnss_velocity_noise_m_s"], 3),
                )
            ),
            np.concatenate(
                (state.gnss_position_ecef_m, state.gnss_velocity_ecef_m_s)
            ),
            time_s,
            state.gnss_sample_time_s,
            faults,
        )
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


def _sample_device_model(
    core: FlightCore,
    body: Body,
    scenario: dict[str, Any],
    avionics: _AvionicsRuntime,
    name: str,
    sample_time_s: float,
) -> dict[str, Any]:
    model = avionics.models[name]
    fault = _fault_active(scenario, body.name, name, sample_time_s)
    if isinstance(model, FlightComputerPlatformModel):
        return model.sample(
            sample_time_s,
            avionics.reported_execution_time_s,
            avionics.last_deadline_missed,
            True,
            core.next_scheduled_command(sample_time_s),
            fault,
        )
    return model.sample(body, sample_time_s, fault)


def _fresh_device_sample(
    avionics: _AvionicsRuntime,
    name: str,
    time_s: float,
) -> dict[str, Any]:
    sample = dict(avionics.received_devices.get(name, {}))
    sample_time_s = float(sample.get("sample_time_s", 0.0))
    age_s = time_s - sample_time_s
    sample["valid"] = int(
        bool(sample.get("valid", 0))
        and -1e-12 <= age_s <= avionics.models[name].timeout_s + 1e-12
    )
    return sample


def _device_inputs(
    avionics: _AvionicsRuntime,
    time_s: float,
) -> FswDeviceInputs:
    air = _fresh_device_sample(avionics, "air_data_computer", time_s)
    engine = _fresh_device_sample(avionics, "engine_controller", time_s)
    discrete = _fresh_device_sample(avionics, "discrete_input_module", time_s)
    recovery = _fresh_device_sample(avionics, "recovery_controller", time_s)
    platform = _fresh_device_sample(
        avionics, "flight_computer_platform", time_s
    )
    command_type = (
        int(platform.get("command_type", FSW_COMMAND_NONE))
        if platform["valid"]
        else FSW_COMMAND_NONE
    )
    if command_type != FSW_COMMAND_NONE:
        avionics.received_devices["flight_computer_platform"][
            "command_type"
        ] = FSW_COMMAND_NONE
    return FswDeviceInputs(
        FswAirDataSample(
            float(air.get("dynamic_pressure_pa", 0.0)),
            float(air.get("sample_time_s", 0.0)),
            air["valid"],
        ),
        FswPropulsionStatus(
            float(engine.get("health_percent", 0.0)),
            float(engine.get("sample_time_s", 0.0)),
            engine["valid"],
            int(engine.get("ready", 0)),
            int(engine.get("running", 0)),
        ),
        FswDiscreteInputs(
            FswDiscreteSample(
                float(discrete.get("sample_time_s", 0.0)),
                discrete["valid"],
                int(discrete.get("stage_separated", 0)),
            ),
            FswDiscreteSample(
                float(recovery.get("sample_time_s", 0.0)),
                recovery["valid"],
                int(recovery.get("drogue_deployed", 0)),
            ),
            FswDiscreteSample(
                float(recovery.get("sample_time_s", 0.0)),
                recovery["valid"],
                int(recovery.get("main_deployed", 0)),
            ),
        ),
        FswPlatformStatus(
            float(platform.get("sample_time_s", 0.0)),
            float(platform.get("previous_execution_time_s", 0.0)),
            platform["valid"],
            int(platform.get("deadline_missed", 0)),
            int(platform.get("watchdog_healthy", 0)),
        ),
        command_type,
        float(platform.get("command_issue_time_s", time_s)),
    )


def _apply_device_inputs(
    frames: list[SensorFrame],
    inputs: FswDeviceInputs,
) -> None:
    for frame in frames:
        frame.dynamic_pressure_pa = inputs.air_data.dynamic_pressure_pa
        frame.engine_health_percent = inputs.propulsion.health_percent
        frame.propulsion_ready = inputs.propulsion.ready
        frame.propulsion_running = inputs.propulsion.running
        frame.stage_separated = inputs.discretes.stage_separated.asserted
        frame.drogue_deployed = inputs.discretes.drogue_deployed.asserted
        frame.main_deployed = inputs.discretes.main_deployed.asserted


def _run_fsw_substeps(
    core: FlightCore,
    body: Body,
    scenario: dict[str, Any],
    rng: np.random.Generator,
    time_s: float,
    launch_position: np.ndarray,
    separated: bool,
    current_output: FswOutput,
    avionics: _AvionicsRuntime,
    on_sensor: Callable[
        [str, list[SensorFrame], FswOutput], None
    ] | None = None,
    timing_mode: str = "deterministic",
    injected_execution_time_s: float | None = None,
    shadow_core: FlightCore | None = None,
    shadow_output: FswOutput | None = None,
) -> tuple[FswOutput, FswOutput | None]:
    timing_override_s = (
        None
        if timing_mode == "measured"
        else injected_execution_time_s if timing_mode == "injected" else 0.0
    )
    output = current_output
    avionics.clock.advance_to(time_s)
    while scheduled := avionics.queue.pop_due(time_s):
        if scheduled.kind == "device_sample":
            avionics.devices.released(scheduled)
            payload = {
                "body": body.name,
                "device": scheduled.subsystem,
                "truth_time_s": time_s,
                "sensor_sample_time_s": scheduled.truth_time_s,
            }
            if scheduled.subsystem in avionics.models:
                payload["measurement"] = _sample_device_model(
                    core,
                    body,
                    scenario,
                    avionics,
                    scheduled.subsystem,
                    scheduled.truth_time_s,
                )
            else:
                payload["measurements"] = [
                    _sample_device(
                        body,
                        scenario,
                        rng,
                        scheduled.truth_time_s,
                        launch_position,
                        channel,
                        scheduled.subsystem,
                    )
                    for channel in range(len(avionics.received))
                ]
            if avionics.devices.complete(scheduled, payload) is None:
                _record_avionics_timeline(
                    avionics, payload, scheduled.subsystem
                )
        elif scheduled.kind == "device_complete":
            device_profile = avionics.devices.profiles[
                str(scheduled.payload["device"])
            ]
            if (
                avionics.bus.submit(
                    scheduled, device_profile.publication_delay_s
                )
                is None
            ):
                _record_avionics_timeline(
                    avionics,
                    scheduled.payload,
                    str(scheduled.payload["device"]),
                )
        elif scheduled.kind == "bus_publish":
            avionics.bus.published(scheduled)
        elif scheduled.kind == "bus_receive":
            device = str(scheduled.payload["device"])
            if device in avionics.models:
                avionics.received_devices[device] = dict(
                    scheduled.payload["measurement"]
                )
            else:
                for received, measurement in zip(
                    avionics.received,
                    scheduled.payload["measurements"],
                    strict=True,
                ):
                    received.update(measurement)
            _record_avionics_timeline(
                avionics,
                scheduled.payload,
                device,
            )
        elif scheduled.kind == "task_release":
            avionics.tasks.released(scheduled)
            tick = scheduled.payload["tick"]
            frames = [
                _frame_from_values(
                    body,
                    scenario,
                    rng,
                    scheduled.truth_time_s,
                    tick.dt_s,
                    separated,
                    received,
                )
                for received in avionics.received
            ]
            inputs = _device_inputs(avionics, scheduled.truth_time_s)
            _apply_device_inputs(frames, inputs)
            avionics.tasks.complete(
                scheduled,
                {
                    "frames": frames,
                    "device_inputs": inputs,
                },
            )
        elif scheduled.kind == "task_complete":
            frames = scheduled.payload["frames"]
            deadline_missed = (
                bool(scheduled.payload["deadline_missed"])
                if timing_mode == "deterministic"
                else None
            )
            completed_output = core.step(
                frames[0],
                device_inputs=scheduled.payload["device_inputs"],
                sensor_channels=frames[1:],
            )
            avionics.reported_execution_time_s = (
                core.previous_execution_time_s
                if timing_override_s is None
                else timing_override_s
            )
            avionics.last_deadline_missed = bool(
                deadline_missed
                if deadline_missed is not None
                else (
                    avionics.reported_execution_time_s
                    > core.loop_deadline_s
                )
            )
            avionics.last_task_time_s = float(
                scheduled.payload["task_release_time_s"]
            )
            completed_shadow = shadow_output
            if shadow_core is not None:
                completed_shadow = shadow_core.step(
                    frames[0],
                    device_inputs=scheduled.payload["device_inputs"],
                    sensor_channels=frames[1:],
                )
            if on_sensor:
                on_sensor(body.name, frames, completed_output)
            avionics.tasks.publish(
                scheduled,
                {
                    "output": completed_output,
                    "shadow_output": completed_shadow,
                },
            )
        elif scheduled.kind == "task_publish":
            output = scheduled.payload["output"]
            shadow_output = scheduled.payload["shadow_output"]
    return output, shadow_output


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
    earth_rate_body = quat_rotate(quat_conjugate(quaternion), earth_rate)
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
        (
            velocity,
            acceleration,
            quat_derivative(quaternion, rates - earth_rate_body),
            rates_dot,
        )
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
    rail = scenario["environment"]["launch_rail"]
    constrained_to_rail = (
        body.upper_mass_kg > 0.0 and body.rail_exit_s is None
    )
    rail_attitude = initial_attitude(
        launch_position,
        scenario["environment"]["launch_azimuth_deg"],
    )
    rail_axis = quat_rotate(
        rail_attitude, np.array([1.0, 0.0, 0.0])
    )
    if constrained_to_rail and body.hold_down_released_s is None:
        gravity_m_s2 = EARTH_MU / float(np.linalg.norm(launch_position)) ** 2
        release = rail["hold_down_release"]
        if (
            thrust_n
            < float(release["minimum_thrust_to_weight"])
            * body.mass_kg
            * gravity_m_s2
        ):
            body.position_ecef_m = launch_position.copy()
            body.velocity_ecef_m_s[:] = 0.0
            body.attitude_wxyz = rail_attitude
            body.body_rates_rad_s = quat_rotate(
                quat_conjugate(rail_attitude),
                np.array([0.0, 0.0, EARTH_ROTATION_RAD_S]),
            )
            _force, aero, _moment = _forces(
                body,
                scenario,
                np.concatenate(
                    (
                        body.position_ecef_m,
                        body.velocity_ecef_m_s,
                        body.attitude_wxyz,
                        body.body_rates_rad_s,
                    )
                ),
                thrust_n,
                tvc_rad,
                fin_commands,
                time_s,
            )
            body.last_dynamic_pressure_pa = aero.dynamic_pressure_pa
            body.last_mach = aero.mach
            body.last_angle_of_attack_deg = aero.angle_of_attack_deg
            body.aero_valid = aero.valid
            return aero
        body.hold_down_released_s = time_s
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

    if constrained_to_rail:
        progress_m = max(
            float(np.dot(next_state[0:3] - launch_position, rail_axis)),
            0.0,
        )
        normal_acceleration = float(
            np.linalg.norm(
                k1[3:6] - np.dot(k1[3:6], rail_axis) * rail_axis
            )
        )
        friction_acceleration = (
            float(rail["friction_coefficient"]) * normal_acceleration
        )
        axial_speed = max(
            float(np.dot(next_state[3:6], rail_axis))
            - friction_acceleration * dt_s,
            0.0,
        )
        progress_m = max(
            progress_m - 0.5 * friction_acceleration * dt_s**2,
            0.0,
        )
        next_state[0:3] = launch_position + progress_m * rail_axis
        next_state[3:6] = axial_speed * rail_axis
        next_state[6:10] = rail_attitude
        next_state[10:13] = quat_rotate(
            quat_conjugate(rail_attitude),
            np.array([0.0, 0.0, EARTH_ROTATION_RAD_S]),
        )
        last_button_m = min(float(value) for value in rail["button_positions_m"])
        if progress_m + last_button_m >= float(rail["length_m"]):
            body.rail_exit_s = time_s + dt_s

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
) -> tuple[Body, Body, dict[str, float]]:
    stages = scenario["vehicle"]["stages"]
    axis = quat_rotate(integrated_stack.attitude_wxyz, np.array([1.0, 0.0, 0.0]))
    impulse_ns = float(scenario["mission"]["separation_impulse_ns"])
    parent_mass = integrated_stack.mass_kg
    parent_center_m = integrated_stack.center_of_mass_m
    parent_inertia = integrated_stack.inertia_kg_m2.copy()
    core_stage = Body(
        "core_stage",
        0,
        stages[0],
        integrated_stack.position_ecef_m.copy(),
        integrated_stack.velocity_ecef_m_s.copy(),
        integrated_stack.attitude_wxyz.copy(),
        integrated_stack.body_rates_rad_s.copy(),
        integrated_stack.fuel_kg,
        integrated_stack.oxidizer_kg,
    )
    upper_stage = Body(
        "upper_stage",
        1,
        stages[1],
        integrated_stack.position_ecef_m.copy(),
        integrated_stack.velocity_ecef_m_s.copy(),
        integrated_stack.attitude_wxyz.copy(),
        integrated_stack.body_rates_rad_s.copy(),
        float(stages[1]["fuel_mass_kg"]),
        float(stages[1]["oxidizer_mass_kg"]),
    )
    core_offset_body = np.array(
        [core_stage.center_of_mass_m - parent_center_m, 0.0, 0.0]
    )
    upper_offset_body = np.array(
        [
            float(stages[0]["length_m"])
            + upper_stage.center_of_mass_m
            - parent_center_m,
            0.0,
            0.0,
        ]
    )
    earth_rate_body = quat_rotate(
        quat_conjugate(integrated_stack.attitude_wxyz),
        np.array([0.0, 0.0, EARTH_ROTATION_RAD_S]),
    )
    angular_rate_body = (
        integrated_stack.body_rates_rad_s - earth_rate_body
    )
    core_relative_velocity_body = (
        cross3(angular_rate_body, core_offset_body)
        - impulse_ns / core_stage.mass_kg * np.array([1.0, 0.0, 0.0])
    )
    upper_relative_velocity_body = (
        cross3(angular_rate_body, upper_offset_body)
        + impulse_ns / upper_stage.mass_kg * np.array([1.0, 0.0, 0.0])
    )
    core_stage.position_ecef_m += quat_rotate(
        integrated_stack.attitude_wxyz, core_offset_body
    )
    upper_stage.position_ecef_m += quat_rotate(
        integrated_stack.attitude_wxyz, upper_offset_body
    )
    core_stage.velocity_ecef_m_s += quat_rotate(
        integrated_stack.attitude_wxyz, core_relative_velocity_body
    )
    upper_stage.velocity_ecef_m_s += quat_rotate(
        integrated_stack.attitude_wxyz, upper_relative_velocity_body
    )

    linear_residual = (
        core_stage.mass_kg * core_relative_velocity_body
        + upper_stage.mass_kg * upper_relative_velocity_body
    )
    angular_before = parent_inertia * angular_rate_body
    angular_after = (
        core_stage.inertia_kg_m2 * angular_rate_body
        + upper_stage.inertia_kg_m2 * angular_rate_body
        + cross3(
            core_offset_body,
            core_stage.mass_kg * core_relative_velocity_body,
        )
        + cross3(
            upper_offset_body,
            upper_stage.mass_kg * upper_relative_velocity_body,
        )
    )
    mass_residual = parent_mass - core_stage.mass_kg - upper_stage.mass_kg
    energy_before_j = 0.5 * float(
        np.dot(angular_rate_body, angular_before)
    )
    energy_after_j = 0.5 * (
        core_stage.mass_kg * float(np.dot(
            core_relative_velocity_body, core_relative_velocity_body
        ))
        + upper_stage.mass_kg * float(np.dot(
            upper_relative_velocity_body, upper_relative_velocity_body
        ))
        + float(np.dot(
            angular_rate_body,
            core_stage.inertia_kg_m2 * angular_rate_body,
        ))
        + float(np.dot(
            angular_rate_body,
            upper_stage.inertia_kg_m2 * angular_rate_body,
        ))
    )
    energy_delta_j = energy_after_j - energy_before_j
    expected_energy_j = 0.5 * impulse_ns**2 * (
        1.0 / core_stage.mass_kg + 1.0 / upper_stage.mass_kg
    )
    audit = {
        "linear_momentum_residual_kg_m_s": float(
            np.linalg.norm(linear_residual)
        ),
        "angular_momentum_residual_kg_m2_s": float(
            np.linalg.norm(angular_after - angular_before)
        ),
        "mass_residual_kg": abs(float(mass_residual)),
        "separation_energy_delta_j": energy_delta_j,
        "expected_separation_energy_j": expected_energy_j,
        "separation_energy_residual_j": abs(
            energy_delta_j - expected_energy_j
        ),
    }
    assert audit["linear_momentum_residual_kg_m_s"] <= (
        1e-9 * max(impulse_ns, 1.0)
    ), f"linear momentum residual: {audit}"
    assert audit["angular_momentum_residual_kg_m2_s"] <= (
        1e-9 * max(float(np.linalg.norm(angular_before)), 1.0)
    ), f"angular momentum residual: {audit}"
    assert audit["mass_residual_kg"] <= (
        1e-12 * max(parent_mass, 1.0)
    ), f"mass residual: {audit}"
    assert audit["separation_energy_residual_j"] <= (
        1e-9 * max(expected_energy_j, 1.0)
    ), f"separation energy residual: {audit}"
    return core_stage, upper_stage, audit


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
        "fsw_previous_execution_time_s": output.previous_execution_time_s,
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
    timing_mode: str = "deterministic",
    injected_execution_time_s: float | None = None,
) -> RunResult:
    if summary_only and persist:
        raise ValueError("summary_only cannot persist telemetry")
    validate_timing_options(timing_mode, injected_execution_time_s)
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
    launch_attitude = initial_attitude(
        launch_position, environment["launch_azimuth_deg"]
    )
    launch_body_rate = quat_rotate(
        quat_conjugate(launch_attitude),
        np.array([0.0, 0.0, EARTH_ROTATION_RAD_S]),
    )
    integrated_stack = Body(
        "integrated_stack",
        0,
        stages[0],
        launch_position.copy(),
        np.zeros(3),
        launch_attitude,
        launch_body_rate,
        float(stages[0]["fuel_mass_kg"]),
        float(stages[0]["oxidizer_mass_kg"]),
        _stage_total_mass(stages[1]),
        float(stages[0]["length_m"] + stages[1]["length_m"]),
        stages[1],
    )
    bodies = [integrated_stack]
    cores: dict[str, FlightCore] = {
        "integrated_stack": FlightCore(scenario, FSW_BODY_INTEGRATED)
    }
    dormant_core: FlightCore | None = FlightCore(
        scenario, FSW_BODY_CORE, auto_commands=False
    )
    dormant_output: FswOutput | None = FswOutput()
    outputs: dict[str, FswOutput] = {"integrated_stack": FswOutput()}
    avionics_timeline: list[dict[str, Any]] = []
    integrated_avionics = _avionics_runtime(
        scenario,
        seed,
        "integrated_stack",
        0.0,
        avionics_timeline,
        record_timeline=not summary_only,
    )
    avionics_runtimes = {"integrated_stack": integrated_avionics}
    all_avionics_runtimes = [integrated_avionics]
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
        event(0.0, "integrated_stack", "simulation_started")
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
                    event(
                        time_s,
                        "integrated_stack" if not separated else "all",
                        "simulation_cancelled",
                        "operator request",
                    )
                )
                break
            for body in list(bodies):
                output, shadow_output = _run_fsw_substeps(
                    cores[body.name],
                    body,
                    scenario,
                    rng,
                    time_s,
                    launch_position,
                    separated,
                    outputs[body.name],
                    avionics_runtimes[body.name],
                    record_sensor,
                    timing_mode,
                    injected_execution_time_s,
                    dormant_core if body.name == "integrated_stack" else None,
                    dormant_output,
                )
                outputs[body.name] = output
                if body.name == "integrated_stack":
                    dormant_output = shadow_output
                previous_mode = previous_modes.get(body.name)
                if previous_mode != output.mode:
                    events.extend(
                        flight_mode_events(
                            time_s,
                            body.name,
                            previous_mode,
                            output.mode,
                        )
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
                core_stage, upper_stage, separation_audit = _split_stack(
                    integrated_stack, scenario
                )
                integrated_core = cores.pop("integrated_stack")
                integrated_output = outputs.pop("integrated_stack")
                integrated_avionics = avionics_runtimes.pop(
                    "integrated_stack"
                )
                assert dormant_core is not None
                assert dormant_output is not None
                bodies = [core_stage, upper_stage]
                cores["core_stage"] = dormant_core
                cores["upper_stage"] = integrated_core
                outputs["core_stage"] = dormant_output
                outputs["upper_stage"] = integrated_output
                previous_fsw_time_s = integrated_avionics.last_task_time_s
                assert previous_fsw_time_s is not None
                for branch in ("core_stage", "upper_stage"):
                    runtime = _avionics_runtime(
                        scenario,
                        seed,
                        branch,
                        previous_fsw_time_s
                        + 1.0
                        / float(
                            scenario["avionics"]["tasks"]["fsw"][
                                "sample_rate_hz"
                            ]
                        ),
                        avionics_timeline,
                        integrated_avionics.received,
                        integrated_avionics.received_devices,
                        record_timeline=not summary_only,
                    )
                    runtime.reported_execution_time_s = (
                        integrated_avionics.reported_execution_time_s
                    )
                    runtime.last_deadline_missed = (
                        integrated_avionics.last_deadline_missed
                    )
                    avionics_runtimes[branch] = runtime
                    all_avionics_runtimes.append(runtime)
                dormant_core = None
                dormant_output = None
                previous_modes.pop("integrated_stack", None)
                previous_modes["upper_stage"] = integrated_output.mode
                separated = True
                events.append(
                    event(
                        time_s,
                        "integrated_stack",
                        "stage_separation",
                        json.dumps(separation_audit, sort_keys=True),
                    )
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
                        event(time_s, body.name, "drogue_deployed")
                    )
                if deploy_main and not body.main_deployed and not main_failed:
                    body.main_deployed = True
                    body.parachute_deployed_s = time_s
                    events.append(event(time_s, body.name, "main_deployed"))
                tvc, fins = _actuator_commands(
                    output, scenario, body, time_s, dt_s
                )
                thrust, pressure, temperature = _propulsion(
                    body, output, time_s, dt_s, scenario
                )
                was_landed = body.landed
                was_released = body.hold_down_released_s is not None
                was_rail_exited = body.rail_exit_s is not None
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
                if not was_released and body.hold_down_released_s is not None:
                    events.append(
                        event(
                            body.hold_down_released_s,
                            body.name,
                            "hold_down_released",
                        )
                    )
                if not was_rail_exited and body.rail_exit_s is not None:
                    events.append(
                        event(
                            body.rail_exit_s,
                            body.name,
                            "rail_exit",
                        )
                    )
                if body.landed and not was_landed:
                    events.append(event(time_s, body.name, "landed"))
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
                        **fsw_sensor_diagnostics_to_row(output),
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
                        "previous_execution_time_s": (
                            output.previous_execution_time_s
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
        if dormant_core is not None:
            dormant_core.close()
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
        "altitude_envelope_20_to_100_km": 20_000.0 <= maximum_altitude_m <= 100_000.0,
        "navigation_altitude_rmse_below_25_m": altitude_rmse_m < 25.0,
    }
    dropped_deadlines = {
        "devices": {
            name: sum(
                runtime.devices.dropped_deadlines[name]
                for runtime in all_avionics_runtimes
            )
            for name in scenario["avionics"]["devices"]
        },
        "tasks": {
            "fsw": sum(
                runtime.tasks.dropped_deadlines["fsw"]
                for runtime in all_avionics_runtimes
            )
        },
        "buses": {
            "sensor_bus": sum(
                runtime.bus.dropped_deadlines
                for runtime in all_avionics_runtimes
            )
        },
    }
    manifest = {
        "schema_version": RUN_SCHEMA_VERSION,
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
        "fsw_timing": {
            "mode": timing_mode,
            "injected_execution_time_s": (
                injected_execution_time_s
                if timing_mode == "injected"
                else None
            ),
        },
        "avionics_timing": {
            "profiles": scenario["avionics"],
            "dropped_deadlines": dropped_deadlines,
            "timeline_rows": len(avionics_timeline),
        },
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
        configuration_artifacts = write_configuration_artifacts(
            output_dir, scenario
        )
        write_manifest(output_dir, manifest)
        write_csv(output_dir / "truth.csv", telemetry)
        write_csv(output_dir / "fsw.csv", fsw_rows)
        write_csv(output_dir / "events.csv", events)
        write_csv(output_dir / "avionics.csv", avionics_timeline)
        artifact_paths = [
            *configuration_artifacts,
            *(
                output_dir / filename
                for filename in (
                    "sensors.csv.gz",
                    "commands.csv",
                    "truth.csv",
                    "fsw.csv",
                    "events.csv",
                    "avionics.csv",
                )
                if (output_dir / filename).exists()
            ),
        ]
        register_artifacts(
            manifest,
            output_dir,
            artifact_paths,
        )
        write_manifest(output_dir, manifest)
    result = RunResult(
        output_dir,
        manifest,
        telemetry,
        fsw_rows,
        events,
        avionics_timeline,
    )
    rocketpy_config = scenario.get("reference_backends", {}).get("rocketpy", {})
    if (
        create_report
        and persist
        and not cancelled
        and rocketpy_config.get("enabled", False)
    ):
        try:
            from ..adapters.rocketpy import run_rocketpy_reference

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
            manifest.setdefault("artifacts", {})[path.name] = sha256_file(path)
        write_manifest(output_dir, manifest)
    if create_report and persist and not cancelled:
        from ..evidence.reporting import create_report_artifacts

        create_report_artifacts(result)
    return result
