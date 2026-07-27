"""ASTARA scenario and vehicle-definition loading."""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
from typing import Any

SCHEMA_VERSION = "astara.scenario.v1"
LEGACY_SCHEMA_VERSION = "astara.scenario.v0"
VEHICLE_SCHEMA_VERSION = "astara.vehicle.v1"
VEHICLE_KEYS = ("vehicle", "sensors", "actuators")


def default_scenario_path() -> Path:
    return (
        Path(__file__).resolve().parent.parent
        / "scenarios"
        / "anthariksa_reference_mission.json"
    )


def load_scenario_documents(
    path: str | Path | None = None,
) -> tuple[dict[str, Any], dict[str, Any] | None, Path | None]:
    source = Path(path) if path else default_scenario_path()
    scenario = json.loads(source.read_text(encoding="utf-8"))
    reference = scenario.get("vehicle_definition")
    if reference is None:
        return scenario, None, None
    if not isinstance(reference, str) or not reference:
        raise ValueError("vehicle_definition must be a nonempty path string")
    vehicle_path = (source.parent / reference).resolve()
    vehicle = json.loads(vehicle_path.read_text(encoding="utf-8"))
    if vehicle.get("schema_version") != VEHICLE_SCHEMA_VERSION:
        raise ValueError(
            f"vehicle schema_version must be {VEHICLE_SCHEMA_VERSION!r}"
        )
    for key in VEHICLE_KEYS:
        if key not in vehicle:
            raise ValueError(f"vehicle definition is missing {key!r}")
    return scenario, vehicle, vehicle_path


def load_scenario(path: str | Path | None = None) -> dict[str, Any]:
    scenario, vehicle, _vehicle_path = load_scenario_documents(path)
    if vehicle:
        for key in VEHICLE_KEYS:
            if key in scenario:
                raise ValueError(
                    f"scenario must not override vehicle definition key {key!r}"
                )
            scenario[key] = copy.deepcopy(vehicle[key])
    validate_scenario(scenario)
    return scenario


def default_scenario() -> dict[str, Any]:
    return copy.deepcopy(load_scenario())


def scenario_hash(scenario: dict[str, Any]) -> str:
    canonical = copy.deepcopy(scenario)
    if canonical.get("vehicle_definition") and all(
        key in canonical for key in VEHICLE_KEYS
    ):
        vehicle = {key: canonical.pop(key) for key in VEHICLE_KEYS}
        canonical.pop("vehicle_definition")
        canonical["vehicle_definition_sha256"] = hashlib.sha256(
            json.dumps(vehicle, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
    encoded = json.dumps(canonical, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def evidence_documents(
    scenario: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    scenario_document = copy.deepcopy(scenario)
    if scenario_document.get("schema_version") != SCHEMA_VERSION:
        return scenario_document, None
    vehicle_document = {
        "schema_version": VEHICLE_SCHEMA_VERSION,
        "name": Path(str(scenario_document["vehicle_definition"])).stem,
        **{
            key: scenario_document.pop(key)
            for key in VEHICLE_KEYS
        },
    }
    scenario_document["vehicle_definition"] = "vehicle_definition.json"
    return scenario_document, vehicle_document


def model_source_hash() -> str:
    root = Path(__file__).resolve().parent.parent
    paths = sorted((root / "astara").glob("*.py"))
    paths.extend(
        (
            root / "flight_core" / "include" / "fsw.h",
            root / "flight_core" / "src" / "fsw.cpp",
        )
    )
    digest = hashlib.sha256()
    for path in paths:
        digest.update(str(path.relative_to(root)).encode())
        digest.update(b"\0")
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _positive(mapping: dict[str, Any], names: tuple[str, ...], prefix: str) -> None:
    for name in names:
        value = mapping.get(name)
        if not isinstance(value, (int, float)) or value <= 0:
            raise ValueError(f"{prefix}.{name} must be positive")


def _validate_table(
    table: Any,
    independent: str,
    required: tuple[str, ...],
    prefix: str,
) -> None:
    if not isinstance(table, list) or len(table) < 2:
        raise ValueError(f"{prefix} requires at least two rows")
    previous = -float("inf")
    for index, row in enumerate(table):
        if not isinstance(row, dict):
            raise ValueError(f"{prefix}[{index}] must be an object")
        for name in (independent, *required):
            value = row.get(name)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{prefix}[{index}].{name} must be finite")
        current = float(row[independent])
        if current <= previous:
            raise ValueError(f"{prefix}.{independent} values must be strictly increasing")
        previous = current


def _integral(table: list[dict[str, Any]], value: str) -> float:
    return sum(
        0.5
        * (float(left[value]) + float(right[value]))
        * (float(right["time_s"]) - float(left["time_s"]))
        for left, right in zip(table, table[1:])
    )


def _validate_engines(stage: dict[str, Any], prefix: str) -> float:
    engines = stage.get("engines")
    if engines is None:
        return 1.0
    if not isinstance(engines, list) or not engines:
        raise ValueError(f"{prefix}.engines must contain at least one engine")

    identifiers: set[str] = set()
    performance_scale = 0.0
    for index, engine in enumerate(engines):
        engine_prefix = f"{prefix}.engines[{index}]"
        if not isinstance(engine, dict):
            raise ValueError(f"{engine_prefix} must be an object")
        identifier = engine.get("id")
        if not isinstance(identifier, str) or not identifier.strip():
            raise ValueError(f"{engine_prefix}.id must be a nonempty string")
        if identifier in identifiers:
            raise ValueError(f"{prefix}.engines contains duplicate id {identifier!r}")
        identifiers.add(identifier)

        for name in ("position_body_m", "direction_body"):
            vector = engine.get(name)
            if (
                not isinstance(vector, list)
                or len(vector) != 3
                or any(
                    not isinstance(value, (int, float)) or not math.isfinite(value)
                    for value in vector
                )
            ):
                raise ValueError(f"{engine_prefix}.{name} requires three finite values")
        if math.sqrt(sum(float(value) ** 2 for value in engine["direction_body"])) <= 1e-12:
            raise ValueError(f"{engine_prefix}.direction_body cannot be zero")

        scale = engine.get("performance_scale", 1.0)
        if not isinstance(scale, (int, float)) or not math.isfinite(scale) or scale < 0.0:
            raise ValueError(f"{engine_prefix}.performance_scale must be finite and nonnegative")
        if not isinstance(engine.get("enabled", True), bool):
            raise ValueError(f"{engine_prefix}.enabled must be boolean")
        if not isinstance(engine.get("gimbal_enabled", True), bool):
            raise ValueError(f"{engine_prefix}.gimbal_enabled must be boolean")
        if engine.get("enabled", True):
            performance_scale += float(scale)

    if performance_scale <= 0.0:
        raise ValueError(f"{prefix}.engines must enable positive total performance")
    return performance_scale


def resolve_mission_events(scenario: dict[str, Any]) -> dict[str, float]:
    """Resolve mission event chains into absolute simulation times."""
    mission = scenario.get("mission", {})
    stage1_burn_s = float(
        scenario["vehicle"]["stages"][0]["propulsion"]["burn_duration_s"]
    )
    definitions = mission.get("events")
    if definitions is None:
        separation = stage1_burn_s + float(mission["separation_delay_s"])
        return {
            "burnout_stage_1": stage1_burn_s,
            "stage_separation": separation,
            "stage2_ignition": separation
            + float(mission["stage2_ignition_delay_s"]),
        }
    if not isinstance(definitions, list) or not definitions:
        raise ValueError("mission.events must contain at least one event")

    pending: dict[str, tuple[str, float]] = {}
    for index, definition in enumerate(definitions):
        prefix = f"mission.events[{index}]"
        if not isinstance(definition, dict):
            raise ValueError(f"{prefix} must be an object")
        event = definition.get("event")
        trigger = definition.get("trigger")
        delay = definition.get("delay")
        if not isinstance(event, str) or not event:
            raise ValueError(f"{prefix}.event must be a nonempty string")
        if not isinstance(trigger, str) or not trigger:
            raise ValueError(f"{prefix}.trigger must be a nonempty string")
        if event == "burnout_stage_1" or event in pending:
            raise ValueError(f"mission.events contains duplicate event {event!r}")
        if (
            not isinstance(delay, (int, float))
            or not math.isfinite(delay)
            or delay < 0.0
        ):
            raise ValueError(f"{prefix}.delay must be finite and nonnegative")
        pending[event] = (trigger, float(delay))

    resolved = {"burnout_stage_1": stage1_burn_s}
    while pending:
        ready = [
            event
            for event, (trigger, _delay) in pending.items()
            if trigger in resolved
        ]
        if not ready:
            unresolved = ", ".join(sorted(pending))
            raise ValueError(
                f"mission.events has cyclic or unknown triggers: {unresolved}"
            )
        for event in ready:
            trigger, delay = pending.pop(event)
            resolved[event] = resolved[trigger] + delay

    for required in ("stage_separation", "stage2_ignition"):
        if required not in resolved:
            raise ValueError(f"mission.events is missing required event {required!r}")
    if resolved["stage2_ignition"] < resolved["stage_separation"]:
        raise ValueError("stage2_ignition cannot occur before stage_separation")
    return resolved


def validate_scenario(scenario: dict[str, Any]) -> None:
    if scenario.get("schema_version") not in (SCHEMA_VERSION, LEGACY_SCHEMA_VERSION):
        raise ValueError(
            f"schema_version must be {SCHEMA_VERSION!r} "
            f"or legacy {LEGACY_SCHEMA_VERSION!r}"
        )
    if scenario.get("schema_version") == SCHEMA_VERSION and not isinstance(
        scenario.get("vehicle_definition"), str
    ):
        raise ValueError("vehicle_definition is required for scenario v2")

    simulation = scenario.get("simulation", {})
    _positive(simulation, ("time_step_s", "max_time_s"), "simulation")
    if not isinstance(simulation.get("seed", 1), int) or simulation.get("seed", 1) < 0:
        raise ValueError("simulation.seed must be a nonnegative integer")
    monte_carlo = scenario.get("monte_carlo", {"samples": 20, "seed": 1})
    if (
        not isinstance(monte_carlo.get("samples"), int)
        or monte_carlo["samples"] < 1
    ):
        raise ValueError("monte_carlo.samples must be a positive integer")
    if (
        not isinstance(monte_carlo.get("seed"), int)
        or monte_carlo["seed"] < 0
    ):
        raise ValueError("monte_carlo.seed must be a nonnegative integer")
    telemetry_sample_percent = monte_carlo.get(
        "telemetry_sample_percent", 2.0
    )
    if (
        not isinstance(telemetry_sample_percent, (int, float))
        or not 0.0 <= telemetry_sample_percent <= 100.0
    ):
        raise ValueError(
            "monte_carlo.telemetry_sample_percent must be between 0 and 100"
        )
    environment = scenario.get("environment", {})
    latitude = environment.get("latitude_deg")
    longitude = environment.get("longitude_deg")
    if not isinstance(latitude, (int, float)) or not -90 <= latitude <= 90:
        raise ValueError("environment.latitude_deg must be between -90 and 90")
    if not isinstance(longitude, (int, float)) or not -180 <= longitude <= 180:
        raise ValueError("environment.longitude_deg must be between -180 and 180")

    stages = scenario.get("vehicle", {}).get("stages")
    if not isinstance(stages, list) or len(stages) != 2:
        raise ValueError("vehicle.stages must contain exactly two stages")
    for index, stage in enumerate(stages, start=1):
        prefix = f"vehicle.stages[{index - 1}]"
        _positive(
            stage,
            ("dry_mass_kg", "fuel_mass_kg", "oxidizer_mass_kg", "length_m", "diameter_m"),
            prefix,
        )
        propulsion = stage.get("propulsion", {})
        engine_performance_scale = _validate_engines(stage, prefix)
        _positive(
            propulsion,
            (
                "burn_duration_s",
                "fuel_flow_kg_s",
                "oxidizer_flow_kg_s",
                "c_star_m_s",
                "thrust_coefficient",
                "nozzle_efficiency",
            ),
            f"{prefix}.propulsion",
        )
        curve = propulsion.get("performance_curve")
        if curve is not None:
            curve_prefix = f"{prefix}.propulsion.performance_curve"
            _validate_table(
                curve,
                "time_s",
                (
                    "thrust_n",
                    "fuel_flow_kg_s",
                    "oxidizer_flow_kg_s",
                    "chamber_pressure_pa",
                    "temperature_k",
                ),
                curve_prefix,
            )
            if float(curve[0]["time_s"]) != 0.0:
                raise ValueError(f"{curve_prefix} must start at time_s=0")
            if abs(float(curve[-1]["time_s"]) - float(propulsion["burn_duration_s"])) > 1e-9:
                raise ValueError(f"{curve_prefix} must end at burn_duration_s")
            for row_index, row in enumerate(curve):
                for name in (
                    "thrust_n",
                    "fuel_flow_kg_s",
                    "oxidizer_flow_kg_s",
                    "chamber_pressure_pa",
                    "temperature_k",
                ):
                    if float(row[name]) < 0.0:
                        raise ValueError(f"{curve_prefix}[{row_index}].{name} cannot be negative")
            fuel_required = _integral(curve, "fuel_flow_kg_s")
            oxidizer_required = _integral(curve, "oxidizer_flow_kg_s")
        else:
            fuel_required = propulsion["fuel_flow_kg_s"] * propulsion["burn_duration_s"]
            oxidizer_required = (
                propulsion["oxidizer_flow_kg_s"] * propulsion["burn_duration_s"]
            )
        fuel_required *= engine_performance_scale
        oxidizer_required *= engine_performance_scale
        if fuel_required > stage["fuel_mass_kg"] * 1.02:
            raise ValueError(f"{prefix} does not contain enough fuel")
        if oxidizer_required > stage["oxidizer_mass_kg"] * 1.02:
            raise ValueError(f"{prefix} does not contain enough oxidizer")
        mass_properties = stage.get("mass_properties")
        if mass_properties is not None:
            mass_prefix = f"{prefix}.mass_properties"
            _validate_table(
                mass_properties,
                "propellant_fraction",
                ("center_of_mass_m",),
                mass_prefix,
            )
            if (
                float(mass_properties[0]["propellant_fraction"]) != 0.0
                or float(mass_properties[-1]["propellant_fraction"]) != 1.0
            ):
                raise ValueError(f"{mass_prefix} must cover propellant_fraction 0 through 1")
            for row_index, row in enumerate(mass_properties):
                inertia = row.get("inertia_kg_m2")
                if (
                    not isinstance(inertia, list)
                    or len(inertia) != 3
                    or any(
                        not isinstance(value, (int, float)) or value <= 0.0
                        for value in inertia
                    )
                ):
                    raise ValueError(
                        f"{mass_prefix}[{row_index}].inertia_kg_m2 requires three positive values"
                    )
        aerodynamics = stage.get("aerodynamics", {})
        if not isinstance(
            aerodynamics.get("movable_fins_enabled", False), bool
        ):
            raise ValueError(
                f"{prefix}.aerodynamics.movable_fins_enabled must be boolean"
            )
        coefficient_table = aerodynamics.get("coefficient_table")
        if coefficient_table is not None:
            _validate_table(
                coefficient_table,
                "mach",
                (
                    "drag_coefficient",
                    "normal_force_slope_per_rad",
                    "pitch_damping_coefficient",
                    "control_force_coefficient",
                ),
                f"{prefix}.aerodynamics.coefficient_table",
            )
            for row_index, row in enumerate(coefficient_table):
                if any(
                    float(row[name]) < 0.0
                    for name in (
                        "drag_coefficient",
                        "normal_force_slope_per_rad",
                        "pitch_damping_coefficient",
                        "control_force_coefficient",
                    )
                ):
                    raise ValueError(
                        f"{prefix}.aerodynamics.coefficient_table[{row_index}] "
                        "coefficients cannot be negative"
                    )
        recovery = stage.get("recovery", {})
        _positive(
            recovery,
            ("drogue_area_m2", "main_area_m2", "main_deploy_altitude_m"),
            f"{prefix}.recovery",
        )

    mission = scenario.get("mission", {})
    if "events" in mission:
        resolve_mission_events(scenario)
    else:
        _positive(
            mission,
            ("separation_delay_s", "stage2_ignition_delay_s"),
            "mission",
        )
    schedule = mission.get("attitude_schedule")
    if not isinstance(schedule, list) or not 2 <= len(schedule) <= 32:
        raise ValueError("mission.attitude_schedule requires 2 to 32 points")
    previous_time = None
    for index, point in enumerate(schedule):
        if not isinstance(point, dict):
            raise ValueError(f"mission.attitude_schedule[{index}] must be an object")
        for name in ("time_s", "pitch_deg", "azimuth_deg"):
            value = point.get(name)
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(
                    f"mission.attitude_schedule[{index}].{name} must be finite"
                )
        if previous_time is not None and point["time_s"] <= previous_time:
            raise ValueError("mission.attitude_schedule times must strictly increase")
        previous_time = point["time_s"]
    commands = mission.get("commands", [])
    if not isinstance(commands, list):
        raise ValueError("mission.commands must be a list")
    previous_command_time = None
    allowed_commands = {"ARM", "DISARM", "LAUNCH", "ABORT", "CLEAR_FAULTS"}
    for index, command in enumerate(commands):
        if not isinstance(command, dict):
            raise ValueError(f"mission.commands[{index}] must be an object")
        time_s = command.get("time_s")
        command_name = command.get("command")
        if (
            not isinstance(time_s, (int, float))
            or not math.isfinite(time_s)
            or time_s < 0.0
        ):
            raise ValueError(
                f"mission.commands[{index}].time_s must be finite and nonnegative"
            )
        if command_name not in allowed_commands:
            raise ValueError(
                f"mission.commands[{index}].command must be one of "
                f"{sorted(allowed_commands)}"
            )
        if previous_command_time is not None and time_s <= previous_command_time:
            raise ValueError("mission.commands times must strictly increase")
        previous_command_time = time_s

    actuators = scenario.get("actuators", {})
    _positive(
        actuators,
        ("max_tvc_deg", "max_fin_deg", "max_rate_deg_s"),
        "actuators",
    )

    sensors = scenario.get("sensors", {})
    _positive(sensors, ("imu_rate_hz", "barometer_rate_hz", "gnss_rate_hz"), "sensors")
    channel_count = sensors.get("channel_count", 1)
    if (
        not isinstance(channel_count, int)
        or isinstance(channel_count, bool)
        or not 1 <= channel_count <= 3
    ):
        raise ValueError("sensors.channel_count must be an integer from 1 to 3")
    for name in (
        "imu_timeout_s",
        "barometer_timeout_s",
        "gnss_timeout_s",
        "air_data_timeout_s",
        "propulsion_status_timeout_s",
        "discrete_feedback_timeout_s",
        "platform_status_timeout_s",
        "step_time_tolerance_s",
        "altitude_filter_tau_s",
        "velocity_filter_tau_s",
    ):
        if name in sensors:
            _positive(sensors, (name,), "sensors")
    for name in (
        "accelerometer_noise_m_s2",
        "gyro_noise_rad_s",
        "barometer_noise_m",
        "gnss_position_noise_m",
        "gnss_velocity_noise_m_s",
        "dynamic_pressure_noise_pa",
        "engine_health_noise_percent",
    ):
        value = sensors.get(name)
        if (
            not isinstance(value, (int, float))
            or value < 0.0
            or not math.isfinite(value)
        ):
            raise ValueError(f"sensors.{name} must be finite and nonnegative")
    fsw_step_s = 1.0 / float(sensors["imu_rate_hz"])
    if simulation["time_step_s"] > fsw_step_s + 1e-12:
        raise ValueError(
            "simulation.time_step_s must be no greater than the IMU/flight-software period "
            f"({fsw_step_s:g} s)"
        )

    uncertainty = scenario.get("uncertainty", {})
    for name, value in uncertainty.items():
        if name == "basis":
            continue
        if not isinstance(value, (int, float)) or value < 0.0 or not math.isfinite(value):
            raise ValueError(f"uncertainty.{name} must be finite and nonnegative")
