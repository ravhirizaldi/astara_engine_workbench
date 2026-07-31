"""Mutable vehicle truth state and mass-property interpolation."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .sensors import SensorChannelState


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
        return min(
            max((self.fuel_kg + self.oxidizer_kg) / initial, 0.0), 1.0
        )

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
                [
                    2.4,
                    self.upper_mass_kg * 2.2**2,
                    self.upper_mass_kg * 2.2**2,
                ]
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
        combined["aerodynamics"][
            "center_of_pressure_m"
        ] = self.stacked_length_m * 0.68
        return combined


def stage_total_mass(stage: dict[str, Any]) -> float:
    return (
        stage["dry_mass_kg"]
        + stage["fuel_mass_kg"]
        + stage["oxidizer_mass_kg"]
    )
