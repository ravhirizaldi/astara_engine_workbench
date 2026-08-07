"""Validation for scenario, vehicle, mission, and sensor contracts."""

from __future__ import annotations

import math
from typing import Any

from .schemas import (
    SCENARIO_SCHEMA_VERSION,
    VEHICLE_KEYS,
    require_schema_version,
)
from .scenarios import resolve_mission_events


def _number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _reject_nonfinite_numbers(value: Any, path: str = "scenario") -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError(f"{path} must be finite")
    if isinstance(value, dict):
        for name, child in value.items():
            _reject_nonfinite_numbers(child, f"{path}.{name}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _reject_nonfinite_numbers(child, f"{path}[{index}]")


def _positive(mapping: dict[str, Any], names: tuple[str, ...], prefix: str) -> None:
    for name in names:
        value = mapping.get(name)
        if not _number(value) or value <= 0:
            raise ValueError(f"{prefix}.{name} must be positive")


def _nonnegative(
    mapping: dict[str, Any], names: tuple[str, ...], prefix: str
) -> None:
    for name in names:
        value = mapping.get(name)
        if not _number(value) or value < 0:
            raise ValueError(f"{prefix}.{name} must be nonnegative")


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
            if not _number(value):
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
                    not _number(value)
                    for value in vector
                )
            ):
                raise ValueError(f"{engine_prefix}.{name} requires three finite values")
        if math.sqrt(sum(float(value) ** 2 for value in engine["direction_body"])) <= 1e-12:
            raise ValueError(f"{engine_prefix}.direction_body cannot be zero")

        scale = engine.get("performance_scale", 1.0)
        if not _number(scale) or scale < 0.0:
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


def validate_scenario(scenario: dict[str, Any]) -> None:
    _reject_nonfinite_numbers(scenario)
    require_schema_version(scenario, SCENARIO_SCHEMA_VERSION)
    if not isinstance(scenario.get("vehicle_definition"), str):
        raise ValueError("vehicle_definition is required")
    if not all(key in scenario for key in VEHICLE_KEYS):
        raise ValueError("resolved vehicle data is required")

    simulation = scenario.get("simulation", {})
    _positive(
        simulation,
        ("time_step_s", "max_time_s", "output_rate_hz"),
        "simulation",
    )
    maximum_output_rate_hz = 1.0 / float(simulation["time_step_s"])
    if float(simulation["output_rate_hz"]) > maximum_output_rate_hz:
        raise ValueError(
            "simulation.output_rate_hz must be no greater than "
            f"1 / simulation.time_step_s ({maximum_output_rate_hz:g} Hz)"
        )
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
        not _number(telemetry_sample_percent)
        or not 0.0 <= telemetry_sample_percent <= 100.0
    ):
        raise ValueError(
            "monte_carlo.telemetry_sample_percent must be between 0 and 100"
        )
    environment = scenario.get("environment", {})
    latitude = environment.get("latitude_deg")
    longitude = environment.get("longitude_deg")
    if not _number(latitude) or not -90 <= latitude <= 90:
        raise ValueError("environment.latitude_deg must be between -90 and 90")
    if not _number(longitude) or not -180 <= longitude <= 180:
        raise ValueError("environment.longitude_deg must be between -180 and 180")
    wind = environment.get("wind_ned_m_s")
    if (
        not isinstance(wind, list)
        or len(wind) != 3
        or any(
            not _number(value)
            for value in wind
        )
    ):
        raise ValueError(
            "environment.wind_ned_m_s requires three finite numeric values"
        )

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
                expected_thrust_n = (
                    float(row["fuel_flow_kg_s"])
                    + float(row["oxidizer_flow_kg_s"])
                ) * (
                    float(propulsion["c_star_m_s"])
                    * float(propulsion["thrust_coefficient"])
                    * float(propulsion["nozzle_efficiency"])
                )
                thrust_n = float(row["thrust_n"])
                if abs(thrust_n - expected_thrust_n) > 0.02 * max(
                    thrust_n, expected_thrust_n, 1.0
                ):
                    raise ValueError(
                        f"{curve_prefix}[{row_index}].thrust_n is inconsistent "
                        "with propellant flow and declared performance"
                    )
            fuel_required = _integral(curve, "fuel_flow_kg_s")
            oxidizer_required = _integral(curve, "oxidizer_flow_kg_s")
        else:
            fuel_required = propulsion["fuel_flow_kg_s"] * propulsion["burn_duration_s"]
            oxidizer_required = (
                propulsion["oxidizer_flow_kg_s"] * propulsion["burn_duration_s"]
            )
        fuel_required *= engine_performance_scale
        oxidizer_required *= engine_performance_scale
        if fuel_required > float(stage["fuel_mass_kg"]) + 1e-9:
            raise ValueError(f"{prefix} does not contain enough fuel")
        if oxidizer_required > float(stage["oxidizer_mass_kg"]) + 1e-9:
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
                        not _number(value) or value <= 0.0
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

    rail = environment.get("launch_rail")
    if not isinstance(rail, dict):
        raise ValueError("environment.launch_rail must be an object")
    _positive(rail, ("length_m",), "environment.launch_rail")
    friction = rail.get("friction_coefficient")
    if (
        not _number(friction)
        or friction < 0.0
    ):
        raise ValueError(
            "environment.launch_rail.friction_coefficient must be nonnegative"
        )
    buttons = rail.get("button_positions_m")
    stacked_length_m = sum(float(stage["length_m"]) for stage in stages)
    if (
        not isinstance(buttons, list)
        or len(buttons) < 2
        or any(
            not _number(value)
            or not 0.0 <= float(value) <= stacked_length_m
            for value in buttons
        )
        or any(
            float(right) <= float(left)
            for left, right in zip(buttons, buttons[1:])
        )
        or (
            isinstance(buttons, list)
            and buttons
            and min(float(value) for value in buttons)
            >= float(rail["length_m"])
        )
    ):
        raise ValueError(
            "environment.launch_rail.button_positions_m must contain "
            "increasing vehicle-axis positions"
        )
    release = rail.get("hold_down_release")
    if not isinstance(release, dict):
        raise ValueError(
            "environment.launch_rail.hold_down_release must be an object"
        )
    if release.get("condition") != "thrust_to_weight":
        raise ValueError(
            "environment.launch_rail.hold_down_release.condition must be "
            "thrust_to_weight"
        )
    _positive(
        release,
        ("minimum_thrust_to_weight",),
        "environment.launch_rail.hold_down_release",
    )

    mission = scenario.get("mission", {})
    _positive(mission, ("separation_impulse_ns",), "mission")
    faults = scenario.get("faults", [])
    if not isinstance(faults, list):
        raise ValueError("faults must be a list")
    fault_ids: set[str] = set()
    bodies = {"all", "integrated_stack", "core_stage", "upper_stage"}
    sensor_fault_types = {
        "dropout", "stale", "freeze", "stuck-valid", "bias", "scale_error"
    }
    device_fault_types = {"dropout", "freeze", "stuck"}
    engine_fault_types = {"cutoff", "thrust_scale", "overtemperature"}
    engine_ids = {
        engine["id"]
        for stage in stages
        for engine in stage.get("engines", [])
    }
    for index, fault in enumerate(faults):
        prefix = f"faults[{index}]"
        if not isinstance(fault, dict):
            raise ValueError(f"{prefix} must be an object")
        identifier = fault.get("id")
        if not isinstance(identifier, str) or not identifier:
            raise ValueError(f"{prefix}.id must be a nonempty string")
        if identifier in fault_ids:
            raise ValueError(f"faults contains duplicate id {identifier!r}")
        if "start_s" in fault or "duration_s" in fault:
            raise ValueError(
                f"{prefix} timing must be declared in mission.timeline"
            )
        if fault.get("body") not in bodies:
            raise ValueError(f"{prefix}.body must name a supported body")
        fault_type = fault.get("type")
        if "sensor" in fault:
            if fault["sensor"] not in {"imu", "magnetometer", "barometer", "gnss"}:
                raise ValueError(f"{prefix}.sensor is not supported")
            if fault_type not in sensor_fault_types:
                raise ValueError(f"{prefix}.type is not supported for sensors")
            channel = fault.get("channel", 0)
            if (
                not isinstance(channel, int)
                or isinstance(channel, bool)
                or not 0 <= channel < int(scenario["sensors"]["channel_count"])
            ):
                raise ValueError(f"{prefix}.channel is outside the sensor suite")
        elif fault.get("component") == "engine":
            if fault_type not in engine_fault_types:
                raise ValueError(f"{prefix}.type is not supported for engines")
            if fault.get("engine_id") not in engine_ids | {"all"}:
                raise ValueError(f"{prefix}.engine_id is not configured")
        elif fault.get("component") in {
            "air_data_computer",
            "engine_controller",
            "discrete_input_module",
            "recovery_controller",
            "flight_computer_platform",
        }:
            if fault_type not in device_fault_types:
                raise ValueError(f"{prefix}.type is not supported for devices")
        else:
            raise ValueError(f"{prefix} must name a supported sensor or component")
        if fault_type in {"bias", "scale_error", "thrust_scale", "overtemperature"}:
            if not _number(fault.get("value")):
                raise ValueError(f"{prefix}.value must be finite and numeric")
        fault_ids.add(identifier)
    timeline = mission.get("timeline")
    if not isinstance(timeline, list) or not timeline:
        raise ValueError("mission.timeline must contain at least one event")
    identifiers: set[str] = set()
    dependencies: dict[str, str] = {}
    trigger_types = {"time", "after_event", "fsw_fact", "truth_detector"}
    action_types = {
        "fsw_command",
        "set_fault",
        "record",
        "split_stage",
        "deploy_recovery",
        "separate_payload",
        "complete_mission",
    }
    allowed_commands = {"ARM", "DISARM", "LAUNCH", "ABORT", "CLEAR_FAULTS"}
    fsw_facts = {
        "launch",
        "meco",
        "stage_separation",
        "stage2_ignition",
        "stage2_first_cutoff",
        "stage2_second_ignition",
        "orbit_insertion",
        "payload_deploy",
        "drogue_deployed",
        "main_deployed",
    }
    truth_detectors = {"hold_down_released", "rail_exit", "max_q", "landed"}
    for index, entry in enumerate(timeline):
        prefix = f"mission.timeline[{index}]"
        if not isinstance(entry, dict):
            raise ValueError(f"{prefix} must be an object")
        identifier = entry.get("id")
        trigger = entry.get("trigger")
        action = entry.get("action")
        if not isinstance(identifier, str) or not identifier:
            raise ValueError(f"{prefix}.id must be a nonempty string")
        if identifier in identifiers:
            raise ValueError(
                f"mission.timeline contains duplicate id {identifier!r}"
            )
        identifiers.add(identifier)
        if not isinstance(trigger, dict) or trigger.get("type") not in trigger_types:
            raise ValueError(
                f"{prefix}.trigger.type must be one of {sorted(trigger_types)}"
            )
        if not isinstance(action, dict) or action.get("type") not in action_types:
            raise ValueError(
                f"{prefix}.action.type must be one of {sorted(action_types)}"
            )
        trigger_type = trigger["type"]
        if trigger_type == "time":
            at_s = trigger.get("at_s")
            if (
                not _number(at_s)
                or at_s < 0.0
            ):
                raise ValueError(f"{prefix}.trigger.at_s must be nonnegative")
        elif trigger_type == "after_event":
            event_id = trigger.get("event")
            delay_s = trigger.get("delay_s")
            if not isinstance(event_id, str) or not event_id:
                raise ValueError(f"{prefix}.trigger.event must be nonempty")
            if (
                not _number(delay_s)
                or delay_s < 0.0
            ):
                raise ValueError(
                    f"{prefix}.trigger.delay_s must be nonnegative"
                )
            dependencies[identifier] = event_id
        else:
            key = "fact" if trigger_type == "fsw_fact" else "detector"
            if not isinstance(trigger.get(key), str) or not trigger[key]:
                raise ValueError(f"{prefix}.trigger.{key} must be nonempty")
            allowed = fsw_facts if trigger_type == "fsw_fact" else truth_detectors
            if trigger[key] not in allowed:
                raise ValueError(f"{prefix}.trigger.{key} is not supported")
        if action["type"] == "fsw_command":
            if action.get("command") not in allowed_commands:
                raise ValueError(
                    f"{prefix}.action.command must be one of "
                    f"{sorted(allowed_commands)}"
                )
            if action.get("target") not in bodies - {"all"}:
                raise ValueError(f"{prefix}.action.target must be a supported body")
        if action["type"] == "set_fault":
            if action.get("fault_id") not in fault_ids:
                raise ValueError(
                    f"{prefix}.action.fault_id must reference a fault"
                )
            if action.get("state") not in {"active", "inactive"}:
                raise ValueError(
                    f"{prefix}.action.state must be active or inactive"
                )
        if (
            action["type"] == "deploy_recovery"
            and action.get("device") not in {"drogue", "main"}
        ):
            raise ValueError(f"{prefix}.action.device must be drogue or main")
    for identifier, dependency in dependencies.items():
        if dependency not in identifiers:
            raise ValueError(
                f"mission.timeline event {identifier!r} references "
                f"unknown event {dependency!r}"
            )
        seen = {identifier}
        current = dependency
        while current in dependencies:
            if current in seen:
                raise ValueError("mission.timeline contains a dependency cycle")
            seen.add(current)
            current = dependencies[current]
    policy = mission.get("flight_core")
    if not isinstance(policy, dict):
        raise ValueError("mission.flight_core must be an object")
    _nonnegative(
        policy,
        ("separation_delay_s", "stage2_ignition_delay_s"),
        "mission.flight_core",
    )
    _positive(
        policy,
        ("stage2_first_burn_s",),
        "mission.flight_core",
    )
    if float(policy["stage2_first_burn_s"]) >= float(
        stages[1]["propulsion"]["burn_duration_s"]
    ):
        raise ValueError(
            "mission.flight_core.stage2_first_burn_s must leave "
            "propellant time for circularization"
        )
    orbit = mission.get("orbit")
    if not isinstance(orbit, dict) or not isinstance(
        orbit.get("enabled"), bool
    ):
        raise ValueError("mission.orbit.enabled must be boolean")
    _positive(
        orbit,
        (
            "target_altitude_m",
            "altitude_tolerance_m",
            "target_inclination_deg",
            "inclination_tolerance_deg",
            "cutoff_speed_margin_m_s",
            "circularization_max_burn_s",
        ),
        "mission.orbit",
    )
    _nonnegative(
        orbit,
        (
            "cutoff_speed_margin_m_s",
            "radial_velocity_tolerance_m_s",
        ),
        "mission.orbit",
    )
    if orbit.get("circularization_guidance") != "prograde":
        raise ValueError(
            "mission.orbit.circularization_guidance must be prograde"
        )
    payload = mission.get("payload")
    if not isinstance(payload, dict):
        raise ValueError("mission.payload must be an object")
    _positive(
        payload,
        (
            "mass_kg",
            "separation_speed_m_s",
            "separation_distance_m",
            "propagation_time_s",
            "length_m",
            "diameter_m",
            "drag_coefficient",
        ),
        "mission.payload",
    )
    _nonnegative(payload, ("deploy_delay_s",), "mission.payload")
    inertia = payload.get("inertia_kg_m2")
    if (
        not isinstance(inertia, list)
        or len(inertia) != 3
        or any(
            not _number(value) or value <= 0.0
            for value in inertia
        )
    ):
        raise ValueError(
            "mission.payload.inertia_kg_m2 requires three positive values"
        )
    resolve_mission_events(scenario)
    schedule = mission.get("attitude_schedule")
    if not isinstance(schedule, list) or not 2 <= len(schedule) <= 32:
        raise ValueError("mission.attitude_schedule requires 2 to 32 points")
    previous_time = None
    for index, point in enumerate(schedule):
        if not isinstance(point, dict):
            raise ValueError(f"mission.attitude_schedule[{index}] must be an object")
        for name in ("time_s", "pitch_deg", "azimuth_deg"):
            value = point.get(name)
            if not _number(value):
                raise ValueError(
                    f"mission.attitude_schedule[{index}].{name} must be finite"
                )
        if previous_time is not None and point["time_s"] <= previous_time:
            raise ValueError("mission.attitude_schedule times must strictly increase")
        previous_time = point["time_s"]

    actuators = scenario.get("actuators", {})
    _positive(
        actuators,
        (
            "max_tvc_deg",
            "max_fin_deg",
            "max_rate_deg_s",
            "tvc_kp",
            "tvc_kd",
            "response_frequency_hz",
            "damping_ratio",
            "supply_voltage_v",
            "max_current_a",
            "max_power_w",
        ),
        "actuators",
    )
    _nonnegative(
        actuators,
        (
            "command_delay_s",
            "deadband_deg",
            "backlash_deg",
            "feedback_quantization_deg",
            "idle_current_a",
            "current_per_rad_s_a",
        ),
        "actuators",
    )
    response_order = actuators.get("response_order")
    if (
        not isinstance(response_order, int)
        or isinstance(response_order, bool)
        or response_order not in (1, 2)
    ):
        raise ValueError("actuators.response_order must be 1 or 2")
    available_current_a = min(
        float(actuators["max_current_a"]),
        float(actuators["max_power_w"])
        / float(actuators["supply_voltage_v"]),
    )
    if float(actuators["idle_current_a"]) > available_current_a:
        raise ValueError(
            "actuators.idle_current_a cannot exceed the current/power limit"
        )

    sensors = scenario.get("sensors", {})
    _positive(
        sensors,
        (
            "imu_rate_hz",
            "magnetometer_rate_hz",
            "magnetometer_timeout_s",
            "barometer_rate_hz",
            "gnss_rate_hz",
        ),
        "sensors",
    )
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
        "max_voter_sample_skew_s",
        "step_time_tolerance_s",
        "altitude_filter_tau_s",
        "velocity_filter_tau_s",
    ):
        if name in sensors:
            _positive(sensors, (name,), "sensors")
    for name in (
        "accelerometer_noise_m_s2",
        "gyro_noise_rad_s",
        "magnetometer_noise",
        "magnetometer_bias_sigma",
        "barometer_noise_m",
        "gnss_position_noise_m",
        "gnss_velocity_noise_m_s",
        "dynamic_pressure_noise_pa",
        "engine_health_noise_percent",
    ):
        value = sensors.get(name)
        if (
            not _number(value)
            or value < 0.0
        ):
            raise ValueError(f"sensors.{name} must be finite and nonnegative")
    _positive(
        sensors,
        (
            "accelerometer_saturation_m_s2",
            "gyro_saturation_rad_s",
            "magnetometer_saturation",
            "barometer_lag_time_constant_s",
            "barometer_port_time_constant_s",
            "nominal_sensor_temperature_k",
        ),
        "sensors",
    )
    _nonnegative(
        sensors,
        (
            "accelerometer_bias_random_walk_m_s2_sqrt_s",
            "gyro_bias_random_walk_rad_s_sqrt_s",
            "accelerometer_quantization_m_s2",
            "gyro_quantization_rad_s",
            "accelerometer_vibration_sensitivity",
            "gyro_vibration_sensitivity",
            "engine_temperature_coupling",
            "magnetometer_bias_random_walk_sqrt_s",
            "magnetometer_quantization",
            "barometer_bias_random_walk_m_sqrt_s",
            "barometer_quantization_m",
            "gnss_position_bias_random_walk_m_sqrt_s",
            "gnss_velocity_bias_random_walk_m_s_sqrt_s",
            "gnss_position_quantization_m",
            "gnss_velocity_quantization_m_s",
            "gnss_acquisition_time_s",
            "gnss_reacquisition_time_s",
        ),
        "sensors",
    )
    for name in (
        "accelerometer_scale_factor_error",
        "gyro_scale_factor_error",
        "accelerometer_temperature_coefficient_m_s2_k",
        "gyro_temperature_coefficient_rad_s_k",
        "imu_lever_arm_body_m",
        "magnetometer_scale_factor_error",
        "magnetometer_temperature_coefficient_k",
        "gnss_position_scale_factor_error",
        "gnss_velocity_scale_factor_error",
    ):
        values = sensors.get(name)
        if (
            not isinstance(values, list)
            or len(values) != 3
            or any(not _number(value) for value in values)
        ):
            raise ValueError(f"sensors.{name} requires three numeric values")
    for name in (
        "imu_cross_axis_misalignment",
        "magnetometer_cross_axis_misalignment",
    ):
        matrix = sensors.get(name)
        if (
            not isinstance(matrix, list)
            or len(matrix) != 3
            or any(
                not isinstance(row, list)
                or len(row) != 3
                or any(not _number(value) for value in row)
                for row in matrix
            )
        ):
            raise ValueError(f"sensors.{name} requires a 3x3 numeric matrix")
    if (
        float(sensors["barometer_min_altitude_m"])
        >= float(sensors["barometer_max_altitude_m"])
    ):
        raise ValueError(
            "sensors.barometer_min_altitude_m must be below the maximum"
        )
    fsw_step_s = 1.0 / float(sensors["imu_rate_hz"])
    if simulation["time_step_s"] > fsw_step_s + 1e-12:
        raise ValueError(
            "simulation.time_step_s must be no greater than the IMU/flight-software period "
            f"({fsw_step_s:g} s)"
        )

    avionics = scenario.get("avionics", {})
    modeled_devices = (
        "air_data_computer",
        "engine_controller",
        "discrete_input_module",
        "recovery_controller",
        "flight_computer_platform",
    )
    expected_subsystems = {
        "devices": (
            "imu",
            "magnetometer",
            "barometer",
            "gnss",
            *modeled_devices,
        ),
        "tasks": ("fsw",),
        "buses": ("sensor_bus",),
    }
    timing_fields = {
        "sample_rate_hz",
        "clock_offset_s",
        "drift_ppm",
        "jitter_s",
        "processing_delay_s",
        "publication_delay_s",
        "deadline_s",
        "drop_on_deadline_miss",
        "phase_offset_s",
        "reset_epoch_s",
    }
    for group, names in expected_subsystems.items():
        profiles = avionics.get(group)
        if not isinstance(profiles, dict):
            raise ValueError(f"avionics.{group} must be an object")
        for name in names:
            profile = profiles.get(name)
            prefix = f"avionics.{group}.{name}"
            if not isinstance(profile, dict):
                raise ValueError(f"{prefix} must be an object")
            missing = timing_fields - profile.keys()
            if missing:
                raise ValueError(
                    f"{prefix} is missing {', '.join(sorted(missing))}"
                )
            for field in timing_fields - {"drop_on_deadline_miss"}:
                value = profile[field]
                if not _number(value):
                    raise ValueError(f"{prefix}.{field} must be numeric")
            for field in (
                "sample_rate_hz",
                "deadline_s",
            ):
                if float(profile[field]) <= 0.0:
                    raise ValueError(f"{prefix}.{field} must be positive")
            for field in (
                "jitter_s",
                "processing_delay_s",
                "publication_delay_s",
            ):
                if float(profile[field]) < 0.0:
                    raise ValueError(f"{prefix}.{field} must be nonnegative")
            if not isinstance(profile["drop_on_deadline_miss"], bool):
                raise ValueError(
                    f"{prefix}.drop_on_deadline_miss must be boolean"
                )
    model_fields = {
        "quantization",
        "valid",
        "timeout_s",
        "noise_stddev",
        "accuracy",
        "startup_delay_s",
        "reset_behavior",
        "command_acknowledgment",
        "command_ack_delay_s",
    }
    for name in modeled_devices:
        profile = avionics["devices"][name]
        prefix = f"avionics.devices.{name}"
        missing = model_fields - profile.keys()
        if missing:
            raise ValueError(
                f"{prefix} is missing {', '.join(sorted(missing))}"
            )
        for field in (
            "quantization",
            "timeout_s",
            "noise_stddev",
            "accuracy",
            "startup_delay_s",
            "command_ack_delay_s",
        ):
            value = profile[field]
            if (
                not _number(value)
            ):
                raise ValueError(f"{prefix}.{field} must be finite and numeric")
        if float(profile["quantization"]) < 0.0:
            raise ValueError(f"{prefix}.quantization must be nonnegative")
        if float(profile["timeout_s"]) <= 0.0:
            raise ValueError(f"{prefix}.timeout_s must be positive")
        for field in (
            "noise_stddev",
            "startup_delay_s",
            "command_ack_delay_s",
        ):
            if float(profile[field]) < 0.0:
                raise ValueError(f"{prefix}.{field} must be nonnegative")
        if not 0.0 <= float(profile["accuracy"]) <= 1.0:
            raise ValueError(f"{prefix}.accuracy must be from 0 to 1")
        if not isinstance(profile["valid"], bool):
            raise ValueError(f"{prefix}.valid must be boolean")
        if profile["reset_behavior"] not in {"invalidate", "hold_last"}:
            raise ValueError(
                f"{prefix}.reset_behavior must be invalidate or hold_last"
            )
        if profile["command_acknowledgment"] not in {
            "immediate",
            "delayed",
            "drop",
        }:
            raise ValueError(
                f"{prefix}.command_acknowledgment must be "
                "immediate, delayed, or drop"
            )
    for device, rate_name in (
        ("imu", "imu_rate_hz"),
        ("magnetometer", "magnetometer_rate_hz"),
        ("barometer", "barometer_rate_hz"),
        ("gnss", "gnss_rate_hz"),
    ):
        if (
            float(avionics["devices"][device]["sample_rate_hz"])
            != float(sensors[rate_name])
        ):
            raise ValueError(
                f"avionics.devices.{device}.sample_rate_hz must match "
                f"sensors.{rate_name}"
            )

    uncertainty = scenario.get("uncertainty", {})
    for name, value in uncertainty.items():
        if name == "basis":
            continue
        if not _number(value) or value < 0.0:
            raise ValueError(f"uncertainty.{name} must be finite and nonnegative")
