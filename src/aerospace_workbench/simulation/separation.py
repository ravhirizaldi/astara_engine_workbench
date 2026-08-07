"""Stage-separation state transition and conservation audit."""

from __future__ import annotations

from typing import Any

import numpy as np

from ..mathematics.frames import EARTH_ROTATION_RAD_S
from ..mathematics.quaternions import quat_conjugate, quat_rotate
from ..mathematics.vectors import cross3
from .truth_model import Body


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
    upper_stage.attached_payload_mass_kg = (
        integrated_stack.attached_payload_mass_kg
    )
    upper_stage.attached_payload_position_m = max(
        integrated_stack.attached_payload_position_m
        - float(stages[0]["length_m"]),
        0.0,
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
