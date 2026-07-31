"""Mission-level orchestration for deterministic SIL runs."""

from __future__ import annotations

import csv
import gzip
import json
import math
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
    FSW_DISCRETE_ACTION_DEPLOY_DROGUE,
    FSW_DISCRETE_ACTION_DEPLOY_MAIN,
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
from ..avionics import _avionics_runtime, _run_fsw_substeps
from ..dynamics import _integrate_body
from ..events import event, flight_mode_events
from ..propulsion import (
    propulsion_step as _propulsion,
    stage_engines as _stage_engines,
)
from ..sensors import fault_active as _fault_active
from ..separation import _split_stack
from ..telemetry import _telemetry_row
from ..truth_model import Body, stage_total_mass as _stage_total_mass


@dataclass
class RunResult:
    output_dir: Path
    manifest: dict[str, Any]
    telemetry: list[dict[str, Any]]
    fsw_telemetry: list[dict[str, Any]]
    events: list[dict[str, Any]]
    avionics_timeline: list[dict[str, Any]] = field(default_factory=list)


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
            from ...adapters.rocketpy import run_rocketpy_reference

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
        from ...evidence.reporting import create_report_artifacts

        create_report_artifacts(result)
    return result
