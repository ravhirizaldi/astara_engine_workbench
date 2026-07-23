"""ASTARA RocketPy reference run for trajectory comparison."""

from __future__ import annotations

import json
from importlib.metadata import version
from pathlib import Path
from typing import Any

import numpy as np

from .scenario import resolve_mission_events


def _enabled_engine_scale(stage: dict[str, Any]) -> float:
    engines = stage.get("engines")
    if not engines:
        return 1.0
    return sum(
        float(engine.get("performance_scale", 1.0))
        for engine in engines
        if engine.get("enabled", True)
    )


def run_rocketpy_reference(
    scenario: dict[str, Any],
    native_telemetry: list[dict[str, Any]],
    output_dir: str | Path,
) -> dict[str, Any]:
    """Compare native and RocketPy core-stage powered-ascent predictions."""
    from rocketpy import Environment, Flight, GenericMotor, Rocket

    stages = scenario["vehicle"]["stages"]
    core = stages[0]
    upper = stages[1]
    propulsion = core["propulsion"]
    scale = _enabled_engine_scale(core)
    curve = propulsion.get("performance_curve")
    if curve:
        thrust_source = np.array(
            [[float(row["time_s"]), float(row["thrust_n"]) * scale] for row in curve]
        )
    else:
        burn_time = float(propulsion["burn_duration_s"])
        thrust = (
            (
                float(propulsion["fuel_flow_kg_s"])
                + float(propulsion["oxidizer_flow_kg_s"])
            )
            * float(propulsion["c_star_m_s"])
            * float(propulsion["thrust_coefficient"])
            * float(propulsion["nozzle_efficiency"])
            * scale
        )
        thrust_source = np.array([[0.0, thrust], [burn_time, thrust]])

    environment_data = scenario["environment"]
    environment = Environment(
        latitude=float(environment_data["latitude_deg"]),
        longitude=float(environment_data["longitude_deg"]),
        elevation=float(environment_data["launch_altitude_m"]),
    )
    wind_north, wind_east, _wind_down = environment_data["wind_ned_m_s"]
    environment.set_atmospheric_model(
        type="custom_atmosphere",
        wind_u=float(wind_east),
        wind_v=float(wind_north),
    )

    propellant_mass = float(core["fuel_mass_kg"] + core["oxidizer_mass_kg"])
    motor = GenericMotor(
        thrust_source=thrust_source,
        burn_time=float(propulsion["burn_duration_s"]),
        chamber_radius=float(core["diameter_m"]) / 2.0,
        chamber_height=float(core["length_m"]),
        chamber_position=float(core["center_of_mass_m"]),
        propellant_initial_mass=propellant_mass,
        nozzle_radius=float(core["diameter_m"]) * 0.1,
        dry_mass=0.0,
        center_of_dry_mass_position=0.0,
        dry_inertia=(0.0, 0.0, 0.0),
        nozzle_position=0.0,
    )

    coefficient_table = core["aerodynamics"]["coefficient_table"]
    mach_points = [float(row["mach"]) for row in coefficient_table]
    drag_points = [float(row["drag_coefficient"]) for row in coefficient_table]

    def drag_coefficient(mach: float) -> float:
        return float(np.interp(mach, mach_points, drag_points))

    upper_mass = sum(
        float(upper[name])
        for name in ("dry_mass_kg", "fuel_mass_kg", "oxidizer_mass_kg")
    )
    rocket = Rocket(
        radius=float(core["diameter_m"]) / 2.0,
        mass=float(core["dry_mass_kg"]) + upper_mass,
        inertia=tuple(float(value) for value in core["inertia_kg_m2"]),
        power_off_drag=drag_coefficient,
        power_on_drag=drag_coefficient,
        center_of_mass_without_motor=float(core["center_of_mass_m"]),
        coordinate_system_orientation="tail_to_nose",
    )
    rocket.add_motor(motor, position=0.0)

    burn_time = float(propulsion["burn_duration_s"])
    reference_end = resolve_mission_events(scenario)["stage_separation"]
    flight = Flight(
        rocket=rocket,
        environment=environment,
        rail_length=12.0,
        inclination=90.0,
        heading=float(environment_data["launch_azimuth_deg"]),
        max_time=reference_end,
        max_time_step=max(float(scenario["simulation"]["time_step_s"]) * 4.0, 0.01),
        time_overshoot=True,
        verbose=False,
        ode_solver="LSODA",
    )

    native_rows = [
        row
        for row in native_telemetry
        if row["body"] == "integrated_stack" and row["time_s"] <= reference_end + 1e-9
    ]
    if not native_rows:
        raise ValueError("native telemetry does not contain integrated-stack ascent")
    native = min(native_rows, key=lambda row: abs(float(row["time_s"]) - burn_time))
    reference = {
        "altitude_m": float(flight.z(burn_time)),
        "speed_m_s": float(flight.speed(burn_time)),
        "mach": float(flight.mach_number(burn_time)),
        "max_dynamic_pressure_pa": float(flight.max_dynamic_pressure),
    }
    native_values = {
        "altitude_m": float(native["altitude_m"]),
        "speed_m_s": float(native["speed_m_s"]),
        "mach": float(native["mach"]),
        "max_dynamic_pressure_pa": max(
            float(row["dynamic_pressure_pa"]) for row in native_rows
        ),
    }
    differences = {
        name: {
            "absolute": reference[name] - native_values[name],
            "relative_percent": 100.0
            * (reference[name] - native_values[name])
            / max(abs(native_values[name]), 1e-12),
        }
        for name in reference
    }
    summary = {
        "schema_version": "astara.rocketpy-reference.v1",
        "rocketpy_version": version("rocketpy"),
        "status": "REFERENCE_ONLY_UNVALIDATED",
        "scope": "Core-stage powered ascent through stage-separation command time.",
        "comparison_time_s": burn_time,
        "native": native_values,
        "rocketpy": reference,
        "difference_rocketpy_minus_native": differences,
        "limitations": [
            "Uses the same provisional thrust and drag inputs as the native model.",
            "Does not replace the native C++ flight-software SIL.",
            "Does not model separated core-stage recovery or upper-stage ignition.",
            "Agreement is a cross-check, not validation against physical test data.",
        ],
    }
    output_path = Path(output_dir) / "rocketpy_reference.json"
    output_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary
