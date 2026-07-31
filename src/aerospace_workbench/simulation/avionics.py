"""Sensor sampling and deterministic avionics execution."""

from __future__ import annotations

import copy
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from ..flight_software.abi import (
    FSW_COMMAND_NONE,
    FswAirDataSample,
    FswDiscreteInputs,
    FswDiscreteSample,
    FswOutput,
    FswPlatformStatus,
    FswPropulsionStatus,
    SensorFrame,
)
from ..flight_software.bridge import FlightCore, FswDeviceInputs
from .device_models import (
    AirDataComputerModel,
    DiscreteInputModule,
    EngineControllerModel,
    FlightComputerPlatformModel,
    RecoveryControllerModel,
)
from .scheduling import (
    BusScheduler,
    DeviceScheduler,
    EventQueue,
    SimulationClock,
    TaskScheduler,
    TimingProfile,
)
from .sensors import (
    _frame_from_values,
    _sample_device,
    fault_active as _fault_active,
)
from .truth_model import Body


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
