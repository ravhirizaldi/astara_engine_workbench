"""TVC, movable-fin, and discrete-actuation behavior."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from ..flight_software.abi import FswOutput
from .sensors import fault_active


def actuator_commands(
    output: FswOutput,
    scenario: dict[str, Any],
    body: Any,
    time_s: float,
    dt_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    max_tvc = math.radians(scenario["actuators"]["max_tvc_deg"])
    max_fin = math.radians(scenario["actuators"]["max_fin_deg"])
    movable_fins_enabled = body.stage.get("aerodynamics", {}).get(
        "movable_fins_enabled", False
    )
    target_tvc = np.clip(
        np.array([output.tvc_pitch_rad, output.tvc_yaw_rad]),
        -max_tvc,
        max_tvc,
    )
    target_fin_rad = (
        np.clip(
            np.array(
                [
                    output.fin_roll_rad,
                    output.fin_pitch_rad,
                    output.fin_yaw_rad,
                ]
            ),
            -max_fin,
            max_fin,
        )
        if movable_fins_enabled
        else np.zeros(3)
    )
    tvc_fault = fault_active(scenario, body.name, "tvc", time_s)
    fin_fault = fault_active(scenario, body.name, "fin", time_s)
    if tvc_fault and tvc_fault.get("type") == "stuck":
        target_tvc[:] = math.radians(
            float(tvc_fault.get("value_deg", 0.0))
        )
    if movable_fins_enabled and fin_fault and fin_fault.get("type") == "stuck":
        target_fin_rad[:] = math.radians(
            float(fin_fault.get("value_deg", 0.0))
        )
    max_delta = (
        math.radians(scenario["actuators"]["max_rate_deg_s"]) * dt_s
    )
    tvc = body.last_tvc_rad + np.clip(
        target_tvc - body.last_tvc_rad, -max_delta, max_delta
    )
    fin_rad = (
        body.last_fin_rad
        + np.clip(
            target_fin_rad - body.last_fin_rad, -max_delta, max_delta
        )
        if movable_fins_enabled
        else np.zeros(3)
    )
    body.last_tvc_rad = tvc
    body.last_fin_rad = fin_rad
    return tvc, fin_rad


def consume_discrete_actuation(
    body: Any, output: FswOutput, action: int
) -> bool:
    command = output.discrete_actuation
    if (
        not command.valid
        or int(command.action) != action
        or int(command.sequence) <= body.last_discrete_actuation_sequence
    ):
        return False
    body.last_discrete_actuation_sequence = int(command.sequence)
    return True
