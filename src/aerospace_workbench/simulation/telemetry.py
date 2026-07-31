"""Truth and flight-software telemetry row shaping."""

from __future__ import annotations

import json
import math
from typing import Any

import numpy as np

from ..flight_software.abi import (
    MODE_NAMES,
    NAVIGATION_STATUS_NAMES,
    FswOutput,
    decode_faults,
)
from ..mathematics.frames import ecef_to_ned
from ..mathematics.quaternions import quat_to_euler
from .propulsion import stage_engines as _stage_engines
from .truth_model import Body


def _telemetry_row(
    time_s: float,
    body: Body,
    output: FswOutput,
    thrust_n: float,
    pressure_pa: float,
    temperature_k: float,
    launch_position: np.ndarray,
) -> dict[str, Any]:
    relative_altitude = float(
        np.linalg.norm(body.position_ecef_m) - np.linalg.norm(launch_position)
    )
    velocity_ned = ecef_to_ned(body.velocity_ecef_m_s, body.position_ecef_m)
    euler_deg = np.degrees(quat_to_euler(body.attitude_wxyz))
    return {
        "time_s": round(time_s, 6),
        "body": body.name,
        "mode": MODE_NAMES[output.mode] if 0 <= output.mode < len(MODE_NAMES) else "UNKNOWN",
        "altitude_m": relative_altitude,
        "position_ecef_x_m": body.position_ecef_m[0],
        "position_ecef_y_m": body.position_ecef_m[1],
        "position_ecef_z_m": body.position_ecef_m[2],
        "velocity_north_m_s": velocity_ned[0],
        "velocity_east_m_s": velocity_ned[1],
        "velocity_down_m_s": velocity_ned[2],
        "speed_m_s": float(np.linalg.norm(body.velocity_ecef_m_s)),
        "attitude_w": body.attitude_wxyz[0],
        "attitude_x": body.attitude_wxyz[1],
        "attitude_y": body.attitude_wxyz[2],
        "attitude_z": body.attitude_wxyz[3],
        "roll_deg": euler_deg[0],
        "pitch_deg": euler_deg[1],
        "yaw_deg": euler_deg[2],
        "rate_x_rad_s": body.body_rates_rad_s[0],
        "rate_y_rad_s": body.body_rates_rad_s[1],
        "rate_z_rad_s": body.body_rates_rad_s[2],
        "mass_kg": body.mass_kg,
        "propellant_fraction": body.propellant_fraction,
        "center_of_mass_m": body.center_of_mass_m,
        "inertia_x_kg_m2": body.inertia_kg_m2[0],
        "inertia_y_kg_m2": body.inertia_kg_m2[1],
        "inertia_z_kg_m2": body.inertia_kg_m2[2],
        "thrust_n": thrust_n,
        "chamber_pressure_pa": pressure_pa,
        "chamber_temperature_k": temperature_k,
        "engine_health_percent": body.engine_health_percent,
        "engine_count": len(_stage_engines(body.stage)),
        "active_engines": sum(
            thrust > 1e-6 for thrust in body.last_engine_thrusts_n.values()
        ),
        "engine_thrusts_n": json.dumps(body.last_engine_thrusts_n, sort_keys=True),
        "dynamic_pressure_pa": body.last_dynamic_pressure_pa,
        "mach": body.last_mach,
        "angle_of_attack_deg": body.last_angle_of_attack_deg,
        "fsw_stage_separate": int(output.stage_separate),
        "fsw_stage2_ignite": int(output.stage2_ignite),
        "fsw_deploy_drogue": int(output.deploy_drogue),
        "fsw_deploy_main": int(output.deploy_main),
        "fsw_abort": int(output.abort),
        "fsw_estimated_altitude_m": output.estimated_altitude_m,
        "fsw_estimated_vertical_velocity_m_s": output.estimated_vertical_velocity_m_s,
        "fsw_estimated_attitude_w": output.estimated_attitude_wxyz[0],
        "fsw_estimated_attitude_x": output.estimated_attitude_wxyz[1],
        "fsw_estimated_attitude_y": output.estimated_attitude_wxyz[2],
        "fsw_estimated_attitude_z": output.estimated_attitude_wxyz[3],
        "fsw_tvc_pitch_deg": math.degrees(output.tvc_pitch_rad),
        "fsw_tvc_yaw_deg": math.degrees(output.tvc_yaw_rad),
        "fsw_fin_roll_deg": math.degrees(output.fin_roll_rad),
        "fsw_fin_pitch_deg": math.degrees(output.fin_pitch_rad),
        "fsw_fin_yaw_deg": math.degrees(output.fin_yaw_rad),
        "fsw_navigation_status": NAVIGATION_STATUS_NAMES[output.navigation_status],
        "fsw_fault_flags": int(
            output.active_fault_flags | output.latched_fault_flags
        ),
        "fsw_faults": decode_faults(
            output.active_fault_flags | output.latched_fault_flags
        ),
        "fsw_active_fault_flags": int(output.active_fault_flags),
        "fsw_latched_fault_flags": int(output.latched_fault_flags),
        "fsw_highest_fault_severity": int(output.highest_fault_severity),
        "fsw_previous_execution_time_s": output.previous_execution_time_s,
        "fsw_altitude_sigma_m": output.altitude_sigma_m,
        "fsw_vertical_velocity_sigma_m_s": output.vertical_velocity_sigma_m_s,
        "fsw_command_sequence": int(output.command_sequence),
        "fsw_command_type": int(output.command_type),
        "fsw_command_result": int(output.command_result),
        "fsw_inhibit_flags": int(output.inhibit_flags),
        "tvc_pitch_deg": math.degrees(body.last_tvc_rad[0]),
        "tvc_yaw_deg": math.degrees(body.last_tvc_rad[1]),
        "tvc_feedback_pitch_deg": math.degrees(
            body.tvc_actuator.feedback_rad[0]
        ),
        "tvc_feedback_yaw_deg": math.degrees(
            body.tvc_actuator.feedback_rad[1]
        ),
        "tvc_current_a": body.tvc_actuator.current_a,
        "tvc_power_w": body.tvc_actuator.power_w,
        "fin_roll_deg": math.degrees(body.last_fin_rad[0]),
        "fin_pitch_deg": math.degrees(body.last_fin_rad[1]),
        "fin_yaw_deg": math.degrees(body.last_fin_rad[2]),
        "fin_feedback_roll_deg": math.degrees(
            body.fin_actuator.feedback_rad[0]
        ),
        "fin_feedback_pitch_deg": math.degrees(
            body.fin_actuator.feedback_rad[1]
        ),
        "fin_feedback_yaw_deg": math.degrees(
            body.fin_actuator.feedback_rad[2]
        ),
        "fin_current_a": body.fin_actuator.current_a,
        "fin_power_w": body.fin_actuator.power_w,
        "drogue_deployed": int(body.drogue_deployed),
        "main_deployed": int(body.main_deployed),
        "landed": int(body.landed),
        "aero_valid": int(body.aero_valid),
    }
