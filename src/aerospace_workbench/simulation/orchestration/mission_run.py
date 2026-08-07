"""Mission-level orchestration for deterministic event-driven SIL runs."""

from __future__ import annotations

import csv
import gzip
import math
from collections import defaultdict, deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import numpy as np

from ... import __version__
from ...configuration.scenarios import model_source_hash, scenario_hash
from ...configuration.schemas import RUN_SCHEMA_VERSION
from ...configuration.validation import validate_scenario
from ...evidence.artifacts import (
    sha256_file,
    write_configuration_artifacts,
    write_csv,
)
from ...evidence.manifest import register_artifacts, write_manifest
from ...flight_software.abi import (
    FSW_BODY_CORE,
    FSW_BODY_INTEGRATED,
    FSW_COMMAND_ABORT,
    FSW_COMMAND_ARM,
    FSW_COMMAND_CLEAR_FAULTS,
    FSW_COMMAND_DISARM,
    FSW_COMMAND_LAUNCH,
    FSW_DISCRETE_ACTION_DEPLOY_DROGUE,
    FSW_DISCRETE_ACTION_DEPLOY_MAIN,
    FSW_DISCRETE_ACTION_DEPLOY_PAYLOAD,
    FSW_DISCRETE_ACTION_STAGE_SEPARATE,
    MODE_NAMES,
    NAVIGATION_STATUS_NAMES,
    FswOutput,
    SensorFrame,
    decode_faults,
)
from ...flight_software.bridge import (
    SENSOR_CSV_FIELDS,
    FlightCore,
    fsw_sensor_diagnostics_to_row,
    sensor_frame_to_row,
)
from ...flight_software.timing import validate_timing_options
from ...mathematics.frames import (
    EARTH_ROTATION_RAD_S,
    ecef_to_geodetic,
    geodetic_to_ecef,
    initial_attitude,
)
from ...mathematics.quaternions import quat_conjugate, quat_rotate
from ..actuators import (
    actuator_commands as _actuator_commands,
    consume_discrete_actuation as _consume_discrete_actuation,
)
from ..avionics import (
    _avionics_runtime,
    handle_avionics_event,
    queue_fsw_command,
)
from ..dynamics import _integrate_body
from ..event_queue import ScheduledEvent
from ..kernel import EventPriority, SimulationKernel
from ..orbit import orbital_elements
from ..propulsion import (
    propulsion_step as _propulsion,
    stage_engines as _stage_engines,
)
from ..scheduler import MissionScheduler
from ..sensors import fault_active as _fault_active
from ..separation import _split_stack
from ..telemetry import _telemetry_row
from ..truth_model import Body, stage_total_mass as _stage_total_mass
from .evidence_sink import EvidenceSink


AVIONICS_EVENT_KINDS = (
    "device_sample",
    "device_complete",
    "bus_publish",
    "bus_receive",
    "task_release",
    "task_complete",
    "task_publish",
)

COMMAND_TYPES = {
    "ARM": FSW_COMMAND_ARM,
    "DISARM": FSW_COMMAND_DISARM,
    "LAUNCH": FSW_COMMAND_LAUNCH,
    "ABORT": FSW_COMMAND_ABORT,
    "CLEAR_FAULTS": FSW_COMMAND_CLEAR_FAULTS,
}

LIVE_FAULT_BODIES = ("all", "integrated_stack", "core_stage", "upper_stage")
LIVE_SENSOR_COMPONENTS = ("imu", "magnetometer", "barometer", "gnss")
LIVE_SENSOR_FAULT_TYPES = (
    "dropout",
    "stale",
    "freeze",
    "stuck-valid",
    "bias",
    "scale_error",
)
LIVE_ENGINE_FAULT_TYPES = ("cutoff", "thrust_scale", "overtemperature")
LIVE_FAULT_TYPES_BY_COMPONENT = {
    **{
        component: LIVE_SENSOR_FAULT_TYPES
        for component in LIVE_SENSOR_COMPONENTS
    },
    "engine": LIVE_ENGINE_FAULT_TYPES,
}
LIVE_FAULT_VALUE_TYPES = frozenset(
    {"bias", "scale_error", "thrust_scale", "overtemperature"}
)


def normalize_live_fault_command(command: dict[str, Any]) -> dict[str, Any]:
    """Validate and normalize an operator command crossing the worker boundary."""
    action = str(command.get("action", ""))
    if action == "clear":
        return {"action": "clear"}
    if action != "inject":
        raise ValueError("live fault action must be inject or clear")

    body = str(command.get("body", ""))
    component = str(command.get("component", ""))
    fault_type = str(command.get("fault_type", ""))
    if body not in LIVE_FAULT_BODIES:
        raise ValueError("unsupported live fault body")
    if component not in LIVE_FAULT_TYPES_BY_COMPONENT:
        raise ValueError("unsupported live fault component")
    if fault_type not in LIVE_FAULT_TYPES_BY_COMPONENT[component]:
        raise ValueError("fault type is not supported for this component")

    duration_s = float(command.get("duration_s", 0.0))
    if not math.isfinite(duration_s) or duration_s < 0.0:
        raise ValueError("fault duration must be finite and nonnegative")
    normalized: dict[str, Any] = {
        "action": "inject",
        "body": body,
        "component": component,
        "fault_type": fault_type,
        "duration_s": duration_s,
    }
    if fault_type in LIVE_FAULT_VALUE_TYPES:
        value = float(command.get("value"))
        if not math.isfinite(value):
            raise ValueError("fault value must be finite")
        if fault_type != "bias" and value < 0.0:
            raise ValueError(f"{fault_type} value must be nonnegative")
        normalized["value"] = value
    return normalized


@dataclass
class RunResult:
    output_dir: Path
    manifest: dict[str, Any]
    telemetry: list[dict[str, Any]]
    fsw_telemetry: list[dict[str, Any]]
    events: list[dict[str, Any]]
    avionics_timeline: list[dict[str, Any]] = field(default_factory=list)


def _fsw_row(time_s: float, body: Body, output: FswOutput) -> dict[str, Any]:
    return {
        "time_s": round(time_s, 6),
        "body": body.name,
        "mode": MODE_NAMES[output.mode],
        "estimated_altitude_m": output.estimated_altitude_m,
        "estimated_vertical_velocity_m_s": (
            output.estimated_vertical_velocity_m_s
        ),
        "estimated_position_ecef_x_m": output.estimated_position_ecef_m[0],
        "estimated_position_ecef_y_m": output.estimated_position_ecef_m[1],
        "estimated_position_ecef_z_m": output.estimated_position_ecef_m[2],
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
        "navigation_status": NAVIGATION_STATUS_NAMES[output.navigation_status],
        **fsw_sensor_diagnostics_to_row(output),
        "fault_flags": int(
            output.active_fault_flags | output.latched_fault_flags
        ),
        "active_fault_flags": int(output.active_fault_flags),
        "latched_fault_flags": int(output.latched_fault_flags),
        "changed_fault_flags": int(output.changed_fault_flags),
        "faults": decode_faults(
            output.active_fault_flags | output.latched_fault_flags
        ),
        "highest_fault_severity": int(output.highest_fault_severity),
        "altitude_sigma_m": output.altitude_sigma_m,
        "vertical_velocity_sigma_m_s": output.vertical_velocity_sigma_m_s,
        "attitude_sigma_x_rad": output.attitude_sigma_rad[0],
        "attitude_sigma_y_rad": output.attitude_sigma_rad[1],
        "attitude_sigma_z_rad": output.attitude_sigma_rad[2],
        "barometer_innovation_m": output.barometer_innovation_m,
        "gnss_altitude_innovation_m": output.gnss_altitude_innovation_m,
        "gnss_velocity_innovation_m_s": (
            output.gnss_velocity_innovation_m_s
        ),
        "command_sequence": int(output.command_sequence),
        "command_type": int(output.command_type),
        "command_result": int(output.command_result),
        "inhibit_flags": int(output.inhibit_flags),
        "consecutive_overruns": int(output.consecutive_overruns),
        "previous_execution_time_s": output.previous_execution_time_s,
        "discrete_actuation_sequence": int(
            output.discrete_actuation.sequence
        ),
        "discrete_actuation_action": int(output.discrete_actuation.action),
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
    control_source: Callable[[float], list[dict[str, Any]]] | None = None,
) -> RunResult:
    if summary_only and persist:
        raise ValueError("summary_only cannot persist telemetry")
    validate_timing_options(timing_mode, injected_execution_time_s)
    validate_scenario(scenario)
    if seed is None:
        seed = int(scenario.get("simulation", {}).get("seed", 1))
    rng = np.random.default_rng(seed)
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = (
        Path(output_root) / f"{run_id}-{scenario_hash(scenario)[:8]}-s{seed}"
    )
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
    output_interval = 1.0 / float(
        scenario["simulation"]["output_rate_hz"]
    )
    launch_position = geodetic_to_ecef(
        environment["latitude_deg"],
        environment["longitude_deg"],
        environment["launch_altitude_m"],
    )
    launch_attitude = initial_attitude(
        launch_position, environment["launch_azimuth_deg"]
    )
    launch_axis = quat_rotate(launch_attitude, np.array([1.0, 0.0, 0.0]))
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
    integrated_stack.attached_payload_mass_kg = float(
        scenario.get("mission", {}).get("payload", {}).get("mass_kg", 0.0)
    )
    integrated_stack.attached_payload_position_m = float(
        stages[0]["length_m"] + stages[1]["length_m"]
    )
    bodies = [integrated_stack]
    kernel = SimulationKernel()
    cores: dict[str, FlightCore] = {
        "integrated_stack": FlightCore(
            scenario, FSW_BODY_INTEGRATED, auto_commands=False
        )
    }
    dormant_core: FlightCore | None = FlightCore(
        scenario, FSW_BODY_CORE, auto_commands=False
    )
    dormant_output: FswOutput | None = FswOutput()
    outputs = {"integrated_stack": FswOutput()}
    avionics_timeline: list[dict[str, Any]] = []
    integrated_avionics = _avionics_runtime(
        scenario,
        seed,
        "integrated_stack",
        0.0,
        avionics_timeline,
        record_timeline=not summary_only,
        queue=kernel.queue,
        clock=kernel.clock,
    )
    avionics_runtimes = {"integrated_stack": integrated_avionics}
    all_avionics_runtimes = [integrated_avionics]
    telemetry: list[dict[str, Any]] = []
    fsw_rows: list[dict[str, Any]] = []
    evidence = EvidenceSink()
    evidence.record(
        0.0,
        "integrated_stack",
        "simulation_started",
        source="kernel",
    )
    events = evidence.events
    statistics = {
        "telemetry_samples": 0,
        "aero_out_of_envelope_samples": 0,
        "aero_out_of_envelope_pre_recovery_samples": 0,
        "maximum_altitude_m": 0.0,
        "maximum_mach": 0.0,
        "maximum_dynamic_pressure_pa": 0.0,
        "maximum_angle_of_attack_deg": 0.0,
        "minimum_engine_health_percent": 100.0,
        "altitude_error_squared_sum": 0.0,
        "navigation_samples": 0,
        "finite_positive_state": True,
    }
    current_values: dict[str, tuple[float, float, float]] = {
        "integrated_stack": (0.0, 0.0, 293.15)
    }
    previous_modes: dict[str, int] = {}
    max_q_samples: dict[str, deque[tuple[float, float]]] = defaultdict(
        lambda: deque(maxlen=3)
    )
    max_q_emitted = False
    separated = False
    completion_scheduled = False
    payload_deployed = False
    payload_elements: dict[str, float] = {}
    cancelled = False
    faults_by_id = {
        str(fault["id"]): fault for fault in scenario.get("faults", [])
    }
    for fault in faults_by_id.values():
        fault["_active"] = False
    live_fault_ids: set[str] = set()
    live_fault_sequence = 0

    def clear_live_fault(identifier: str, time_s: float, reason: str) -> None:
        fault = faults_by_id.get(identifier)
        if fault is None or not bool(fault.get("_active", False)):
            return
        fault["_active"] = False
        kernel.cancel_owner(f"live-fault:{identifier}")
        evidence.record(
            time_s,
            str(fault["body"]),
            "fault_cleared",
            source="operator",
            detail={"fault_id": identifier, "reason": reason},
        )

    def handle_live_fault_clear(scheduled: ScheduledEvent) -> None:
        clear_live_fault(
            str(scheduled.payload["fault_id"]),
            scheduled.truth_time_s,
            "duration elapsed",
        )

    kernel.register("live_fault_clear", handle_live_fault_clear)

    def apply_live_control(command: dict[str, Any], time_s: float) -> None:
        nonlocal live_fault_sequence
        normalized = normalize_live_fault_command(command)
        if normalized["action"] == "clear":
            for identifier in tuple(live_fault_ids):
                clear_live_fault(identifier, time_s, "operator clear")
            return

        live_fault_sequence += 1
        identifier = f"operator-fault-{live_fault_sequence}"
        fault: dict[str, Any] = {
            "id": identifier,
            "body": normalized["body"],
            "type": normalized["fault_type"],
            "_active": True,
        }
        component = str(normalized["component"])
        if component == "engine":
            fault.update({"component": "engine", "engine_id": "all"})
        else:
            fault["sensor"] = component
        if "value" in normalized:
            fault["value"] = normalized["value"]
        scenario.setdefault("faults", []).append(fault)
        faults_by_id[identifier] = fault
        live_fault_ids.add(identifier)
        evidence.record(
            time_s,
            str(normalized["body"]),
            "fault_injected",
            source="operator",
            detail={"fault_id": identifier, **normalized},
        )
        duration_s = float(normalized["duration_s"])
        if duration_s > 0.0:
            kernel.schedule(
                time_s + duration_s,
                EventPriority.TIMELINE,
                "live_fault_clear",
                payload={"fault_id": identifier},
                owner=f"live-fault:{identifier}",
            )

    def poll_live_controls(time_s: float) -> None:
        if control_source is None:
            return
        try:
            commands = control_source(time_s)
        except Exception as error:
            evidence.record(
                time_s,
                "all",
                "fault_rejected",
                source="operator",
                detail=f"control source failed: {type(error).__name__}: {error}",
            )
            return
        for command in commands:
            try:
                apply_live_control(command, time_s)
            except (TypeError, ValueError) as error:
                evidence.record(
                    time_s,
                    "all",
                    "fault_rejected",
                    source="operator",
                    detail=str(error),
                )

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
        sensor_writer = csv.DictWriter(
            sensor_file, fieldnames=SENSOR_CSV_FIELDS
        )
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
            command_writer.writerow(
                {
                    "body": body_name,
                    "time_s": frames[0].time_s,
                    "sequence": int(output.command_sequence),
                    "command_type": int(output.command_type),
                    "result": int(output.command_result),
                    "inhibit_flags": int(output.inhibit_flags),
                }
            )

    def split_stack(time_s: float) -> dict[str, Any]:
        nonlocal bodies, separated, dormant_core, dormant_output
        core_stage, upper_stage, audit = _split_stack(
            integrated_stack, scenario
        )
        integrated_core = cores.pop("integrated_stack")
        integrated_output = outputs.pop("integrated_stack")
        old_runtime = avionics_runtimes.pop("integrated_stack")
        kernel.cancel_owner("integrated_stack")
        if dormant_core is None or dormant_output is None:
            raise RuntimeError("missing dormant core at separation")
        bodies = [core_stage, upper_stage]
        cores["core_stage"] = dormant_core
        cores["upper_stage"] = integrated_core
        outputs["core_stage"] = dormant_output
        outputs["upper_stage"] = integrated_output
        previous_fsw_time_s = old_runtime.last_task_time_s
        if previous_fsw_time_s is None:
            raise RuntimeError("separation occurred before an FSW release")
        start_s = previous_fsw_time_s + 1.0 / float(
            scenario["avionics"]["tasks"]["fsw"]["sample_rate_hz"]
        )
        for branch in ("core_stage", "upper_stage"):
            runtime = _avionics_runtime(
                scenario,
                seed,
                branch,
                start_s,
                avionics_timeline,
                old_runtime.received,
                old_runtime.received_devices,
                record_timeline=not summary_only,
                queue=kernel.queue,
                clock=kernel.clock,
            )
            runtime.reported_execution_time_s = (
                old_runtime.reported_execution_time_s
            )
            runtime.last_deadline_missed = old_runtime.last_deadline_missed
            avionics_runtimes[branch] = runtime
            all_avionics_runtimes.append(runtime)
        dormant_core = None
        dormant_output = None
        previous_modes.pop("integrated_stack", None)
        previous_modes["upper_stage"] = integrated_output.mode
        current_values.pop("integrated_stack", None)
        current_values["core_stage"] = (0.0, 0.0, 293.15)
        current_values["upper_stage"] = (0.0, 0.0, 293.15)
        separated = True
        return audit

    def separate_payload(time_s: float) -> dict[str, Any]:
        nonlocal payload_deployed
        if payload_deployed:
            return payload_elements
        upper = next(body for body in bodies if body.name == "upper_stage")
        payload_config = scenario["mission"]["payload"]
        payload_mass_kg = float(payload_config["mass_kg"])
        if upper.attached_payload_mass_kg + 1e-12 < payload_mass_kg:
            raise RuntimeError("upper stage does not carry the configured payload")
        axis = quat_rotate(
            upper.attitude_wxyz, np.array([1.0, 0.0, 0.0])
        )
        upper_mass_kg = upper.mass_kg - payload_mass_kg
        total_mass_kg = upper_mass_kg + payload_mass_kg
        relative_speed_m_s = float(payload_config["separation_speed_m_s"])
        upper.velocity_ecef_m_s -= (
            payload_mass_kg / total_mass_kg * relative_speed_m_s * axis
        )
        payload_velocity = upper.velocity_ecef_m_s + relative_speed_m_s * axis
        payload_stage = {
            "name": "Payload",
            "dry_mass_kg": payload_mass_kg,
            "fuel_mass_kg": 0.0,
            "oxidizer_mass_kg": 0.0,
            "length_m": float(payload_config["length_m"]),
            "diameter_m": float(payload_config["diameter_m"]),
            "center_of_mass_m": 0.5 * float(payload_config["length_m"]),
            "inertia_kg_m2": list(payload_config["inertia_kg_m2"]),
            "propulsion": {
                "burn_duration_s": 1.0,
                "fuel_flow_kg_s": 0.0,
                "oxidizer_flow_kg_s": 0.0,
                "c_star_m_s": 1.0,
                "thrust_coefficient": 1.0,
                "nozzle_efficiency": 1.0,
                "chamber_pressure_pa": 1.0,
                "combustion_temperature_k": 293.15,
            },
            "aerodynamics": {
                "fin_count": 0,
                "fin_area_m2": 0.0,
                "movable_fins_enabled": False,
                "base_drag_coefficient": float(
                    payload_config["drag_coefficient"]
                ),
                "center_of_pressure_m": 0.5
                * float(payload_config["length_m"]),
                "induced_drag_factor": 0.0,
            },
            "recovery": {
                "drogue_area_m2": 0.0,
                "main_area_m2": 0.0,
                "main_deploy_altitude_m": 1.0,
                "inflation_delay_s": 0.0,
            },
        }
        upper.attached_payload_mass_kg = 0.0
        payload = Body(
            "payload",
            2,
            payload_stage,
            upper.position_ecef_m
            + axis * float(payload_config["separation_distance_m"]),
            payload_velocity,
            upper.attitude_wxyz.copy(),
            upper.body_rates_rad_s.copy(),
            0.0,
            0.0,
        )
        bodies.append(payload)
        outputs["payload"] = FswOutput()
        outputs["payload"].mode = 15
        current_values["payload"] = (0.0, 0.0, 293.15)
        payload_deployed = True
        payload_elements.update(
            orbital_elements(payload.position_ecef_m, payload.velocity_ecef_m_s)
        )
        kernel.schedule(
            time_s + float(payload_config["propagation_time_s"]),
            EventPriority.LIFECYCLE,
            "payload_propagation_complete",
            owner="payload",
        )
        return dict(payload_elements)

    def timeline_handler(
        entry: dict[str, Any], scheduled: ScheduledEvent
    ) -> None:
        action = entry["action"]
        action_type = str(action["type"])
        body_name = str(scheduled.payload["body"])
        detail = scheduled.payload.get("detail", {})
        effective_time_s = float(scheduled.payload["effective_time_s"])
        if action_type == "fsw_command":
            target = str(action["target"])
            body_name = target
            runtime = avionics_runtimes.get(target)
            if runtime is None:
                raise RuntimeError(
                    f"timeline command target {target!r} is unavailable"
                )
            queue_fsw_command(
                runtime,
                COMMAND_TYPES[str(action["command"])],
                scheduled.truth_time_s,
            )
        elif action_type == "split_stage":
            if separated:
                return
            detail = split_stack(scheduled.truth_time_s)
        elif action_type == "deploy_recovery":
            body = next(
                candidate
                for candidate in bodies
                if candidate.name == body_name
            )
            device = str(action["device"])
            fault = _fault_active(
                scenario, body.name, device, scheduled.truth_time_s
            )
            if fault and fault.get("type") == "failed":
                return
            if device == "drogue":
                body.drogue_deployed = True
            else:
                body.main_deployed = True
            body.parachute_deployed_s = scheduled.truth_time_s
        elif action_type == "separate_payload":
            detail = separate_payload(scheduled.truth_time_s)
        elif action_type == "set_fault":
            fault = faults_by_id[str(action["fault_id"])]
            body_name = str(fault["body"])
            fault["_active"] = action["state"] == "active"
        elif action_type not in {"record", "complete_mission"}:
            raise ValueError(f"unsupported timeline action {action_type!r}")
        evidence.record(
            effective_time_s,
            body_name,
            str(entry["id"]),
            source=str(scheduled.payload["source"]),
            detected_time_s=scheduled.truth_time_s,
            detail=detail,
        )
        if action_type == "complete_mission":
            kernel.stop()

    mission_scheduler = MissionScheduler(
        kernel, scenario["mission"]["timeline"], timeline_handler
    )

    def publish_mode_facts(
        body_name: str, output: FswOutput, time_s: float
    ) -> None:
        previous_mode = previous_modes.get(body_name)
        if previous_mode == output.mode:
            return
        current_name = MODE_NAMES[output.mode]
        previous_name = (
            MODE_NAMES[previous_mode] if previous_mode is not None else None
        )
        evidence.record(
            time_s,
            body_name,
            "flight_mode",
            source="fsw",
            detail=current_name,
        )
        if current_name == "BOOST_1":
            mission_scheduler.publish(
                "fsw_fact", "launch", time_s, body_name
            )
        if previous_name == "BOOST_1" and current_name == "SEPARATION":
            mission_scheduler.publish(
                "fsw_fact", "meco", time_s, body_name
            )
        if current_name == "BOOST_2":
            mission_scheduler.publish(
                "fsw_fact", "stage2_ignition", time_s, body_name
            )
        if previous_name == "BOOST_2" and current_name == "COAST":
            mission_scheduler.publish(
                "fsw_fact", "stage2_first_cutoff", time_s, body_name
            )
        if current_name == "ORBIT_INSERTION":
            mission_scheduler.publish(
                "fsw_fact", "stage2_second_ignition", time_s, body_name
            )
        if current_name == "ORBIT":
            body = next(candidate for candidate in bodies if candidate.name == body_name)
            mission_scheduler.publish(
                "fsw_fact",
                "orbit_insertion",
                time_s,
                body_name,
                orbital_elements(
                    body.position_ecef_m, body.velocity_ecef_m_s
                ),
            )
        if (
            current_name == "ABORT"
            and previous_name in {"COAST", "ORBIT_INSERTION"}
            and scenario["mission"].get("orbit", {}).get("enabled", False)
        ):
            evidence.record(
                time_s,
                body_name,
                "orbit_insertion_failed",
                source="fsw",
            )
        previous_modes[body_name] = output.mode

    def handle_avionics(scheduled: ScheduledEvent) -> None:
        body_name = scheduled.owner
        runtime = avionics_runtimes.get(body_name)
        core = cores.get(body_name)
        body = next(
            (candidate for candidate in bodies if candidate.name == body_name),
            None,
        )
        if runtime is None or core is None or body is None:
            return
        shadow_core = (
            dormant_core if body_name == "integrated_stack" else None
        )
        output, shadow = handle_avionics_event(
            core,
            body,
            scenario,
            rng,
            launch_position,
            separated,
            outputs[body_name],
            runtime,
            scheduled,
            record_sensor,
            timing_mode,
            injected_execution_time_s,
            shadow_core,
            dormant_output,
        )
        outputs[body_name] = output
        if body_name == "integrated_stack" and shadow is not None:
            nonlocal_set_dormant_output(shadow)
        if scheduled.kind != "task_publish":
            return
        publish_mode_facts(body_name, output, scheduled.truth_time_s)
        actions = (
            (
                FSW_DISCRETE_ACTION_STAGE_SEPARATE,
                "stage_separation",
            ),
            (
                FSW_DISCRETE_ACTION_DEPLOY_DROGUE,
                "drogue_deployed",
            ),
            (
                FSW_DISCRETE_ACTION_DEPLOY_MAIN,
                "main_deployed",
            ),
            (
                FSW_DISCRETE_ACTION_DEPLOY_PAYLOAD,
                "payload_deploy",
            ),
        )
        for action, fact in actions:
            if _consume_discrete_actuation(body, output, action):
                mission_scheduler.publish(
                    "fsw_fact", fact, scheduled.truth_time_s, body_name
                )

    def nonlocal_set_dormant_output(output: FswOutput) -> None:
        nonlocal dormant_output
        dormant_output = output

    for kind in AVIONICS_EVENT_KINDS:
        kernel.register(kind, handle_avionics)

    def estimate_peak(
        samples: deque[tuple[float, float]],
    ) -> tuple[float, float]:
        (t0, q0), (t1, q1), (t2, q2) = samples
        denominator = q0 - 2.0 * q1 + q2
        if abs(denominator) <= 1e-12:
            return t1, q1
        half_span = 0.5 * (t2 - t0)
        offset = 0.5 * (q0 - q2) / denominator
        peak_time = t1 + offset * half_span
        peak_q = q1 - 0.25 * (q0 - q2) * offset
        return peak_time, peak_q

    def handle_truth(scheduled: ScheduledEvent) -> None:
        nonlocal max_q_emitted, completion_scheduled
        time_s = scheduled.truth_time_s
        for body in list(bodies):
            output = outputs[body.name]
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
                launch_attitude,
                launch_axis,
            )
            if not was_released and body.hold_down_released_s is not None:
                mission_scheduler.publish(
                    "truth_detector",
                    "hold_down_released",
                    time_s,
                    body.name,
                    effective_time_s=body.hold_down_released_s,
                )
            if not was_rail_exited and body.rail_exit_s is not None:
                mission_scheduler.publish(
                    "truth_detector",
                    "rail_exit",
                    time_s,
                    body.name,
                    effective_time_s=body.rail_exit_s,
                )
            if body.landed and not was_landed:
                mission_scheduler.publish(
                    "truth_detector", "landed", time_s, body.name
                )
            primary = (
                body.name == "integrated_stack"
                or separated and body.name == "upper_stage"
            )
            if primary and not max_q_emitted:
                samples = max_q_samples["mission"]
                samples.append((time_s, body.last_dynamic_pressure_pa))
                if (
                    len(samples) == 3
                    and samples[1][1] > samples[0][1]
                    and samples[1][1] >= samples[2][1]
                    and samples[1][1] > 0.0
                ):
                    peak_time, peak_q = estimate_peak(samples)
                    mission_scheduler.publish(
                        "truth_detector",
                        "max_q",
                        time_s,
                        body.name,
                        {"dynamic_pressure_pa": peak_q},
                        peak_time,
                    )
                    max_q_emitted = True
            current_values[body.name] = (
                thrust,
                pressure,
                temperature,
            )
        next_index = int(scheduled.payload["index"]) + 1
        next_time_s = next_index * dt_s
        if next_time_s <= max_time_s + 1e-12:
            kernel.schedule(
                next_time_s,
                EventPriority.TRUTH,
                "truth_step",
                payload={"index": next_index},
                owner="truth",
            )
        if (
            not scenario["mission"].get("orbit", {}).get("enabled", False)
            and
            separated
            and all(body.landed for body in bodies)
            and not completion_scheduled
        ):
            completion_scheduled = True
            kernel.schedule(
                time_s,
                EventPriority.LIFECYCLE,
                "mission_complete",
                owner="lifecycle",
            )

    kernel.register("truth_step", handle_truth)

    def handle_output(scheduled: ScheduledEvent) -> None:
        time_s = scheduled.truth_time_s
        poll_live_controls(time_s)
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
            statistics["telemetry_samples"] += 1
            statistics["aero_out_of_envelope_samples"] += int(
                not row["aero_valid"]
            )
            statistics[
                "aero_out_of_envelope_pre_recovery_samples"
            ] += int(
                not row["aero_valid"]
                and not row["drogue_deployed"]
                and not row["main_deployed"]
            )
            statistics["maximum_altitude_m"] = max(
                statistics["maximum_altitude_m"],
                float(row["altitude_m"]),
            )
            statistics["maximum_mach"] = max(
                statistics["maximum_mach"], float(row["mach"])
            )
            statistics["maximum_dynamic_pressure_pa"] = max(
                statistics["maximum_dynamic_pressure_pa"],
                float(row["dynamic_pressure_pa"]),
            )
            statistics["maximum_angle_of_attack_deg"] = max(
                statistics["maximum_angle_of_attack_deg"],
                float(row["angle_of_attack_deg"]),
            )
            statistics["minimum_engine_health_percent"] = min(
                statistics["minimum_engine_health_percent"],
                float(row["engine_health_percent"]),
            )
            fsw_row = None
            if body.name in avionics_runtimes:
                fsw_row = _fsw_row(time_s, body, output)
                altitude_error = float(row["altitude_m"]) - float(
                    fsw_row["estimated_altitude_m"]
                )
                statistics["altitude_error_squared_sum"] += (
                    altitude_error * altitude_error
                )
                statistics["navigation_samples"] += 1
            statistics["finite_positive_state"] = bool(
                statistics["finite_positive_state"]
                and math.isfinite(float(row["altitude_m"]))
                and math.isfinite(float(row["mass_kg"]))
                and float(row["mass_kg"]) > 0.0
            )
            if not summary_only:
                telemetry.append(row)
                if fsw_row is not None:
                    fsw_rows.append(fsw_row)
            new_rows.append(row)
        if on_sample:
            on_sample(time_s, new_rows, events)
        next_index = int(scheduled.payload["index"]) + 1
        next_time_s = next_index * output_interval
        if next_time_s <= max_time_s + 1e-12:
            kernel.schedule(
                next_time_s,
                EventPriority.EVIDENCE,
                "output_sample",
                payload={"index": next_index},
                owner="evidence",
            )

    kernel.register("output_sample", handle_output)

    def handle_completion(scheduled: ScheduledEvent) -> None:
        evidence.record(
            scheduled.truth_time_s,
            "all",
            "mission_complete",
            source="kernel",
        )
        kernel.stop()

    kernel.register("mission_complete", handle_completion)

    def handle_payload_completion(scheduled: ScheduledEvent) -> None:
        payload = next(body for body in bodies if body.name == "payload")
        payload_elements.clear()
        payload_elements.update(
            orbital_elements(payload.position_ecef_m, payload.velocity_ecef_m_s)
        )
        evidence.record(
            scheduled.truth_time_s,
            "payload",
            "payload_propagation_complete",
            source="truth",
            detail=dict(payload_elements),
        )
        kernel.stop()

    kernel.register(
        "payload_propagation_complete", handle_payload_completion
    )
    mission_scheduler.start()
    kernel.schedule(
        0.0,
        EventPriority.TRUTH,
        "truth_step",
        payload={"index": 0},
        owner="truth",
    )
    kernel.schedule(
        0.0,
        EventPriority.EVIDENCE,
        "output_sample",
        payload={"index": 0},
        owner="evidence",
    )

    def cancel_requested() -> bool:
        nonlocal cancelled
        cancelled = bool(should_cancel and should_cancel())
        return cancelled

    try:
        kernel.run(max_time_s, cancel_requested)
        if cancelled:
            evidence.record(
                kernel.clock.truth_time_s,
                "integrated_stack" if not separated else "all",
                "simulation_cancelled",
                source="kernel",
                detail="operator request",
            )
    finally:
        for core in cores.values():
            core.close()
        if dormant_core is not None:
            dormant_core.close()
        if sensor_file is not None:
            sensor_file.close()
        if command_file is not None:
            command_file.close()
        for fault in faults_by_id.values():
            fault.pop("_active", None)
        if live_fault_ids:
            scenario["faults"][:] = [
                fault
                for fault in scenario.get("faults", [])
                if str(fault.get("id")) not in live_fault_ids
            ]

    time_s = kernel.clock.truth_time_s
    invalid_rows = int(statistics["aero_out_of_envelope_samples"])
    invalid_pre_recovery_rows = int(
        statistics["aero_out_of_envelope_pre_recovery_samples"]
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
    maximum_altitude_m = float(statistics["maximum_altitude_m"])
    telemetry_samples = int(statistics["telemetry_samples"])
    altitude_rmse_m = math.sqrt(
        float(statistics["altitude_error_squared_sum"])
        / max(int(statistics["navigation_samples"]), 1)
    )
    separated_event = any(
        row["event"] == "stage_separation" for row in events
    )
    orbit_config = scenario["mission"].get("orbit", {})
    if orbit_config.get("enabled", False):
        target_altitude_m = float(orbit_config["target_altitude_m"])
        altitude_tolerance_m = float(orbit_config["altitude_tolerance_m"])
        checks = {
            "finite_positive_state": bool(statistics["finite_positive_state"]),
            "stage_separation": separated_event,
            "orbit_insertion": any(
                row["event"] == "orbit_insertion" for row in events
            ),
            "payload_deployed": payload_deployed,
            "payload_propagated": any(
                row["event"] == "payload_propagation_complete"
                for row in events
            ),
            "periapsis_in_target_band": abs(
                payload_elements.get("periapsis_altitude_m", -math.inf)
                - target_altitude_m
            )
            <= altitude_tolerance_m,
            "apoapsis_in_target_band": abs(
                payload_elements.get("apoapsis_altitude_m", -math.inf)
                - target_altitude_m
            )
            <= altitude_tolerance_m,
            "inclination_in_target_band": abs(
                payload_elements.get("inclination_deg", -math.inf)
                - float(orbit_config["target_inclination_deg"])
            )
            <= float(orbit_config["inclination_tolerance_deg"]),
        }
    else:
        checks = {
            "finite_positive_state": bool(statistics["finite_positive_state"]),
            "stage_separation": separated_event,
            "core_stage_landed": landed.get("core_stage", False),
            "upper_stage_landed": landed.get("upper_stage", False),
            "altitude_envelope_20_to_100_km": (
                20_000.0 <= maximum_altitude_m <= 100_000.0
            ),
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
        "event_schema_version": "aerospace-workbench.events.v2",
        "model_version": __version__,
        "model_source_sha256": model_source_hash(),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scenario_name": scenario["name"],
        "scenario_sha256": scenario_hash(scenario),
        "seed": seed,
        "units": "SI",
        "coordinate_frames": (
            "ECEF truth, body attitude quaternion, NED reports"
        ),
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
        "aero_out_of_envelope_pre_recovery_samples": (
            invalid_pre_recovery_rows
        ),
        "aero_out_of_envelope_recovery_samples": invalid_recovery_rows,
        "landed": landed,
        "impact_points": impact_points,
        "payload_orbit": dict(payload_elements),
        "duration_s": time_s,
        "maximum_altitude_m": maximum_altitude_m,
        "summary_metrics": {
            "telemetry_samples": telemetry_samples,
            "maximum_mach": statistics["maximum_mach"],
            "maximum_dynamic_pressure_pa": statistics[
                "maximum_dynamic_pressure_pa"
            ],
            "maximum_angle_of_attack_deg": statistics[
                "maximum_angle_of_attack_deg"
            ],
            "minimum_engine_health_percent": statistics[
                "minimum_engine_health_percent"
            ],
        },
        "navigation_altitude_rmse_m": altitude_rmse_m,
        "checks": checks,
        "model_configuration": {
            "propulsion": [
                (
                    "tabulated_curve"
                    if stage["propulsion"].get("performance_curve")
                    else "reduced_order"
                )
                for stage in stages
            ],
            "aerodynamics": [
                (
                    "mach_coefficient_table"
                    if stage.get("aerodynamics", {}).get(
                        "coefficient_table"
                    )
                    else "geometry_estimate"
                )
                for stage in stages
            ],
            "mass_properties": [
                (
                    "propellant_fraction_table"
                    if stage.get("mass_properties")
                    else "fixed"
                )
                for stage in stages
            ],
            "engine_counts": [
                len(_stage_engines(stage)) for stage in stages
            ],
        },
        "credibility": scenario.get(
            "credibility",
            {
                "status": "UNSPECIFIED",
                "intended_use": (
                    "No intended use was declared by the scenario."
                ),
            },
        ),
        "result_interpretation": (
            "PASS/FAIL covers numerical mission checks only; it is not "
            "model validation or flight certification."
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
        register_artifacts(manifest, output_dir, artifact_paths)
        write_manifest(output_dir, manifest)
    result = RunResult(
        output_dir,
        manifest,
        telemetry,
        fsw_rows,
        events,
        avionics_timeline,
    )
    rocketpy_config = scenario.get(
        "reference_backends", {}
    ).get("rocketpy", {})
    if (
        create_report
        and persist
        and not cancelled
        and rocketpy_config.get("enabled", False)
    ):
        try:
            from ...adapters.rocketpy import run_rocketpy_reference

            reference = run_rocketpy_reference(
                scenario, telemetry, output_dir
            )
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
            manifest.setdefault("artifacts", {})[path.name] = sha256_file(
                path
            )
        write_manifest(output_dir, manifest)
    if create_report and persist and not cancelled:
        from ...evidence.reporting import create_report_artifacts

        create_report_artifacts(result)
    return result
