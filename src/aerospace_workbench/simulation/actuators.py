"""TVC, movable-fin, and discrete-actuation behavior."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
from typing import Any

import numpy as np

from ..flight_software.abi import FswOutput
from .sensors import fault_active


@dataclass
class ActuatorState:
    position_rad: np.ndarray
    velocity_rad_s: np.ndarray
    delayed_command_rad: np.ndarray
    backlash_command_rad: np.ndarray
    feedback_rad: np.ndarray
    pending_commands: deque[tuple[float, np.ndarray]] = field(
        default_factory=deque
    )
    current_a: float = 0.0
    power_w: float = 0.0

    @classmethod
    def zeroed(cls, axes: int) -> ActuatorState:
        return cls(*(np.zeros(axes) for _ in range(5)))

    def reset(self) -> None:
        self.position_rad[:] = 0.0
        self.velocity_rad_s[:] = 0.0
        self.delayed_command_rad[:] = 0.0
        self.backlash_command_rad[:] = 0.0
        self.feedback_rad[:] = 0.0
        self.pending_commands.clear()
        self.current_a = 0.0
        self.power_w = 0.0


def _update_feedback(
    state: ActuatorState,
    quantization_rad: float,
) -> None:
    state.feedback_rad = (
        np.round(state.position_rad / quantization_rad) * quantization_rad
        if quantization_rad > 0.0
        else state.position_rad.copy()
    )


def _step_actuator(
    state: ActuatorState,
    target_rad: np.ndarray,
    position_limit_rad: float,
    config: dict[str, Any],
    time_s: float,
    dt_s: float,
    fault: dict[str, Any] | None,
) -> np.ndarray:
    state.pending_commands.append(
        (
            time_s + float(config["command_delay_s"]),
            np.clip(target_rad, -position_limit_rad, position_limit_rad),
        )
    )
    while (
        state.pending_commands
        and state.pending_commands[0][0] <= time_s + 1e-12
    ):
        _due_s, state.delayed_command_rad = state.pending_commands.popleft()

    fault_type = str((fault or {}).get("type", ""))
    quantization_rad = math.radians(
        float(config["feedback_quantization_deg"])
    )
    if fault_type in {"stuck", "loss_of_power"}:
        if fault_type == "stuck" and "value_deg" in (fault or {}):
            state.position_rad[:] = np.clip(
                math.radians(float(fault["value_deg"])),
                -position_limit_rad,
                position_limit_rad,
            )
        state.velocity_rad_s[:] = 0.0
        state.current_a = 0.0
        state.power_w = 0.0
        _update_feedback(state, quantization_rad)
        return state.position_rad.copy()

    command_rad = state.delayed_command_rad.copy()
    if fault_type == "hardover":
        value_deg = float((fault or {}).get("value_deg", 90.0))
        command_rad[:] = math.copysign(
            position_limit_rad, value_deg
        )

    backlash_rad = math.radians(float(config["backlash_deg"]))
    state.backlash_command_rad = np.clip(
        state.backlash_command_rad,
        command_rad - backlash_rad,
        command_rad + backlash_rad,
    )
    error_rad = state.backlash_command_rad - state.position_rad
    deadband_rad = math.radians(float(config["deadband_deg"]))
    inactive = np.abs(error_rad) <= deadband_rad
    error_rad[inactive] = 0.0

    frequency_rad_s = 2.0 * math.pi * float(
        config["response_frequency_hz"]
    )
    if int(config["response_order"]) == 1:
        desired_velocity_rad_s = frequency_rad_s * error_rad
    else:
        acceleration_rad_s2 = (
            frequency_rad_s**2 * error_rad
            - 2.0
            * float(config["damping_ratio"])
            * frequency_rad_s
            * state.velocity_rad_s
        )
        desired_velocity_rad_s = (
            state.velocity_rad_s + acceleration_rad_s2 * dt_s
        )
    desired_velocity_rad_s[inactive] = 0.0

    rate_scale = (
        min(max(float((fault or {}).get("value", 0.5)), 0.0), 1.0)
        if fault_type == "degraded"
        else 1.0
    )
    rate_limit_rad_s = (
        math.radians(float(config["max_rate_deg_s"])) * rate_scale
    )
    desired_velocity_rad_s = np.clip(
        desired_velocity_rad_s,
        -rate_limit_rad_s,
        rate_limit_rad_s,
    )

    # ponytail: lumped current budget; use per-axis motor/load models when
    # hardware sizing or shared-bus transients matter.
    voltage_v = float(config["supply_voltage_v"])
    idle_current_a = float(config["idle_current_a"])
    motion_current_a = float(config["current_per_rad_s_a"]) * float(
        np.sum(np.abs(desired_velocity_rad_s))
    )
    available_current_a = min(
        float(config["max_current_a"]),
        float(config["max_power_w"]) / voltage_v,
    )
    requested_current_a = idle_current_a + motion_current_a
    if requested_current_a > available_current_a and motion_current_a > 0.0:
        desired_velocity_rad_s *= max(
            (available_current_a - idle_current_a) / motion_current_a,
            0.0,
        )
        requested_current_a = available_current_a

    step_rad = desired_velocity_rad_s * dt_s
    if int(config["response_order"]) == 1:
        step_rad = np.sign(error_rad) * np.minimum(
            np.abs(step_rad), np.abs(error_rad)
        )
    previous_position_rad = state.position_rad.copy()
    state.position_rad = np.clip(
        state.position_rad + step_rad,
        -position_limit_rad,
        position_limit_rad,
    )
    state.velocity_rad_s = (
        (state.position_rad - previous_position_rad) / dt_s
        if dt_s > 0.0
        else np.zeros_like(state.position_rad)
    )
    state.current_a = min(requested_current_a, available_current_a)
    state.power_w = voltage_v * state.current_a
    _update_feedback(state, quantization_rad)
    return state.position_rad.copy()


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
    tvc = _step_actuator(
        body.tvc_actuator,
        target_tvc,
        max_tvc,
        scenario["actuators"],
        time_s,
        dt_s,
        tvc_fault,
    )
    fin_rad = (
        _step_actuator(
            body.fin_actuator,
            target_fin_rad,
            max_fin,
            scenario["actuators"],
            time_s,
            dt_s,
            fin_fault,
        )
        if movable_fins_enabled
        else np.zeros(3)
    )
    if not movable_fins_enabled:
        body.fin_actuator.reset()
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
