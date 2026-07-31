"""Mutable vehicle truth state and mass-property interpolation."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .actuators import ActuatorState
from .sensors import SensorChannelState


def stage_propellant_fraction(stage: dict[str, Any], mass_kg: float) -> float:
    propellant_mass = float(stage["fuel_mass_kg"]) + float(
        stage["oxidizer_mass_kg"]
    )
    return min(
        max((mass_kg - float(stage["dry_mass_kg"])) / propellant_mass, 0.0),
        1.0,
    )


def stage_center_of_mass_m(
    stage: dict[str, Any], propellant_fraction: float
) -> float:
    table = stage.get("mass_properties")
    if not table:
        return float(stage["center_of_mass_m"])
    return float(
        np.interp(
            propellant_fraction,
            [row["propellant_fraction"] for row in table],
            [row["center_of_mass_m"] for row in table],
        )
    )


def stage_inertia_kg_m2(
    stage: dict[str, Any], propellant_fraction: float
) -> np.ndarray:
    table = stage.get("mass_properties")
    if not table:
        return np.asarray(stage["inertia_kg_m2"], dtype=float)
    return np.array(
        [
            np.interp(
                propellant_fraction,
                [row["propellant_fraction"] for row in table],
                [row["inertia_kg_m2"][axis] for row in table],
            )
            for axis in range(3)
        ]
    )


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
    attached_stage: dict[str, Any] | None = None
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
    last_discrete_actuation_sequence: int = 0
    tvc_actuator: ActuatorState = field(
        default_factory=lambda: ActuatorState.zeroed(2)
    )
    fin_actuator: ActuatorState = field(
        default_factory=lambda: ActuatorState.zeroed(3)
    )
    last_engine_thrusts_n: dict[str, float] = field(default_factory=dict)
    last_chamber_pressure_pa: float = 0.0
    last_engine_temperature_k: float = 293.15
    last_engine_rpm: float = 0.0
    hold_down_released_s: float | None = None
    rail_exit_s: float | None = None

    @property
    def last_tvc_rad(self) -> np.ndarray:
        return self.tvc_actuator.position_rad

    @last_tvc_rad.setter
    def last_tvc_rad(self, value: np.ndarray) -> None:
        self.tvc_actuator.position_rad = np.asarray(value, dtype=float).copy()

    @property
    def last_fin_rad(self) -> np.ndarray:
        return self.fin_actuator.position_rad

    @last_fin_rad.setter
    def last_fin_rad(self, value: np.ndarray) -> None:
        self.fin_actuator.position_rad = np.asarray(value, dtype=float).copy()

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
        return min(
            max((self.fuel_kg + self.oxidizer_kg) / initial, 0.0), 1.0
        )

    @property
    def center_of_mass_m(self) -> float:
        own_center = stage_center_of_mass_m(
            self.stage, self.propellant_fraction
        )
        if self.attached_stage is None or self.upper_mass_kg <= 0.0:
            return own_center
        attached_fraction = stage_propellant_fraction(
            self.attached_stage, self.upper_mass_kg
        )
        attached_center = float(self.stage["length_m"]) + (
            stage_center_of_mass_m(self.attached_stage, attached_fraction)
        )
        own_mass = self.mass_kg - self.upper_mass_kg
        return (
            own_mass * own_center + self.upper_mass_kg * attached_center
        ) / self.mass_kg

    @property
    def inertia_kg_m2(self) -> np.ndarray:
        inertia = stage_inertia_kg_m2(
            self.stage, self.propellant_fraction
        )
        if self.attached_stage is not None and self.upper_mass_kg > 0.0:
            attached_fraction = stage_propellant_fraction(
                self.attached_stage, self.upper_mass_kg
            )
            attached_inertia = stage_inertia_kg_m2(
                self.attached_stage, attached_fraction
            )
            own_mass = self.mass_kg - self.upper_mass_kg
            own_center = stage_center_of_mass_m(
                self.stage, self.propellant_fraction
            )
            attached_center = float(self.stage["length_m"]) + (
                stage_center_of_mass_m(
                    self.attached_stage, attached_fraction
                )
            )
            combined_center = self.center_of_mass_m
            inertia = inertia + attached_inertia
            inertia[1:] += (
                own_mass * (own_center - combined_center) ** 2
                + self.upper_mass_kg
                * (attached_center - combined_center) ** 2
            )
        return np.maximum(inertia, 1e-3)

    def aerodynamic_stage(self) -> dict[str, Any]:
        combined = dict(self.stage)
        combined["center_of_mass_m"] = self.center_of_mass_m
        if self.stacked_length_m is None:
            return combined
        combined["length_m"] = self.stacked_length_m
        combined["center_of_mass_m"] = self.center_of_mass_m
        combined["aerodynamics"] = dict(self.stage["aerodynamics"])
        if self.attached_stage is not None:
            own_aero = self.stage["aerodynamics"]
            attached_aero = self.attached_stage["aerodynamics"]
            own_table = own_aero.get("coefficient_table") or [{}]
            attached_table = attached_aero.get("coefficient_table") or [{}]
            own_weight = float(self.stage["diameter_m"]) ** 2 * float(
                own_table[0].get("normal_force_slope_per_rad", 1.0)
            )
            attached_weight = float(
                self.attached_stage["diameter_m"]
            ) ** 2 * float(
                attached_table[0].get("normal_force_slope_per_rad", 1.0)
            )
            attached_cp = float(self.stage["length_m"]) + float(
                attached_aero["center_of_pressure_m"]
            )
            combined["aerodynamics"]["center_of_pressure_m"] = (
                own_weight * float(own_aero["center_of_pressure_m"])
                + attached_weight * attached_cp
            ) / (own_weight + attached_weight)
        return combined


def stage_total_mass(stage: dict[str, Any]) -> float:
    return (
        stage["dry_mass_kg"]
        + stage["fuel_mass_kg"]
        + stage["oxidizer_mass_kg"]
    )
