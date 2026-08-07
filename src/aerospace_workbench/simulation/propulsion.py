"""Engine-cluster configuration and propulsion fault policy."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from ..flight_software.abi import FswOutput


def stage_engines(stage: dict[str, Any]) -> list[dict[str, Any]]:
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


def engine_fault_active(
    scenario: dict[str, Any],
    body: str,
    engine_id: str,
    time_s: float,
) -> dict[str, Any] | None:
    for fault in scenario.get("faults", []):
        target = fault.get("component", fault.get("sensor"))
        target_engine = fault.get("engine_id", "all")
        if (
            bool(fault.get("_active", False))
            and
            fault.get("body", "all") in ("all", body)
            and target == "engine"
            and target_engine in ("all", engine_id)
        ):
            return fault
    return None


def thrust_ramp(time_since_start: float, burn_duration: float) -> float:
    ramp_duration = min(0.35, burn_duration * 0.08)
    return max(
        0.0,
        min(
            1.0,
            time_since_start / ramp_duration,
            (burn_duration - time_since_start) / ramp_duration,
        ),
    )


def _record_engine_parameters(
    body: Any,
    propulsion: dict[str, Any],
    thrust_n: float,
    chamber_pressure_pa: float,
    temperature_k: float,
) -> tuple[float, float, float]:
    body.last_chamber_pressure_pa = chamber_pressure_pa
    body.last_engine_temperature_k = temperature_k
    nominal_pressure = max(float(propulsion["chamber_pressure_pa"]), 1.0)
    body.last_engine_rpm = 30_000.0 * math.sqrt(
        max(chamber_pressure_pa / nominal_pressure, 0.0)
    )
    return thrust_n, chamber_pressure_pa, temperature_k


def propulsion_step(
    body: Any,
    output: FswOutput,
    time_s: float,
    dt_s: float,
    scenario: dict[str, Any],
) -> tuple[float, float, float]:
    propulsion = body.stage["propulsion"]
    engines = stage_engines(body.stage)
    body.last_engine_thrusts_n = {engine["id"]: 0.0 for engine in engines}
    ignition_request = False
    if body.stage_index == 0 and body.upper_mass_kg > 0.0:
        ignition_request = bool(output.stage1_ignite)
    if body.stage_index == 1:
        ignition_request = bool(output.stage2_ignite)
    duration = float(propulsion["burn_duration_s"])
    if (
        ignition_request
        and not body.engine_running
        and body.engine_burn_elapsed_s < duration
    ):
        body.engine_running = True
        body.engine_ignition_s = time_s
        if body.engine_started_s is None:
            body.engine_started_s = time_s
    shutdown_request = bool(
        output.abort
        or (body.stage_index == 1 and output.stage2_shutdown)
    )
    if body.engine_running and shutdown_request:
        body.engine_burn_elapsed_s += max(
            time_s
            - float(
                body.engine_ignition_s
                if body.engine_ignition_s is not None
                else time_s
            ),
            0.0,
        )
        body.engine_running = False
        body.engine_ignition_s = None
    elapsed = body.engine_burn_elapsed_s + (
        max(
            time_s
            - float(
                body.engine_ignition_s
                if body.engine_ignition_s is not None
                else time_s
            ),
            0.0,
        )
        if body.engine_running
        else 0.0
    )
    if body.engine_running and elapsed >= duration:
        body.engine_burn_elapsed_s = duration
        body.engine_running = False
        body.engine_ignition_s = None
    if not body.engine_running or elapsed < 0.0 or elapsed >= duration:
        return _record_engine_parameters(body, propulsion, 0.0, 0.0, 293.15)
    curve = propulsion.get("performance_curve")
    if curve:
        times = [row["time_s"] for row in curve]
        base_fuel_flow = float(
            np.interp(
                elapsed,
                times,
                [row["fuel_flow_kg_s"] for row in curve],
            )
        )
        base_oxidizer_flow = float(
            np.interp(
                elapsed,
                times,
                [row["oxidizer_flow_kg_s"] for row in curve],
            )
        )
        base_thrust = float(
            np.interp(elapsed, times, [row["thrust_n"] for row in curve])
        )
        chamber_pressure = float(
            np.interp(
                elapsed,
                times,
                [row["chamber_pressure_pa"] for row in curve],
            )
        )
        temperature = float(
            np.interp(
                elapsed, times, [row["temperature_k"] for row in curve]
            )
        )
    else:
        ramp = thrust_ramp(elapsed, duration)
        base_fuel_flow = float(propulsion["fuel_flow_kg_s"]) * ramp
        base_oxidizer_flow = (
            float(propulsion["oxidizer_flow_kg_s"]) * ramp
        )
        base_thrust = (
            (base_fuel_flow + base_oxidizer_flow)
            * float(propulsion["c_star_m_s"])
            * float(propulsion["thrust_coefficient"])
            * float(propulsion["nozzle_efficiency"])
        )
        chamber_pressure = (
            float(propulsion["chamber_pressure_pa"]) * ramp
        )
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
        fault = engine_fault_active(
            scenario, body.name, engine["id"], time_s
        )
        if fault and fault.get("type") == "cutoff":
            scale = 0.0
        flow_scale += scale
        thrust_scale = scale
        if fault and fault.get("type") == "thrust_scale":
            thrust_scale *= max(float(fault.get("value", 1.0)), 0.0)
        if fault and fault.get("type") == "overtemperature":
            temperature_increase = max(
                temperature_increase,
                max(float(fault.get("value", 0.0)), 0.0),
            )
        thrust_scales[engine["id"]] = thrust_scale

    if flow_scale <= 0.0:
        return _record_engine_parameters(body, propulsion, 0.0, 0.0, 293.15)

    requested_fuel_flow = base_fuel_flow * flow_scale
    requested_oxidizer_flow = base_oxidizer_flow * flow_scale
    availability = 1.0
    if requested_fuel_flow > 0.0:
        availability = min(
            availability, body.fuel_kg / (requested_fuel_flow * dt_s)
        )
    if requested_oxidizer_flow > 0.0:
        availability = min(
            availability,
            body.oxidizer_kg / (requested_oxidizer_flow * dt_s),
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
    return _record_engine_parameters(
        body, propulsion, thrust, chamber_pressure, temperature
    )
