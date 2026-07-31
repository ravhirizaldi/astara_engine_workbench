"""Rigid-body dynamics, launch-rail constraints, and recovery drag."""

from __future__ import annotations

from typing import Any

import numpy as np

from ..mathematics.frames import (
    EARTH_MU,
    EARTH_RADIUS_M,
    EARTH_ROTATION_RAD_S,
    initial_attitude,
    ned_to_ecef,
)
from ..mathematics.quaternions import (
    quat_conjugate,
    quat_derivative,
    quat_normalize,
    quat_rotate,
)
from ..mathematics.vectors import cross3, unit
from .aerodynamics import AeroResult, atmosphere, estimate
from .propulsion import stage_engines as _stage_engines
from .truth_model import Body


def _forces(
    body: Body,
    scenario: dict[str, Any],
    state: np.ndarray,
    thrust_n: float,
    tvc_rad: np.ndarray,
    fin_commands: np.ndarray,
    time_s: float,
) -> tuple[np.ndarray, AeroResult, np.ndarray]:
    position = state[0:3]
    velocity = state[3:6]
    quaternion = quat_normalize(state[6:10])
    rates = state[10:13]
    launch_altitude = float(scenario["environment"]["launch_altitude_m"])
    altitude = max(float(np.linalg.norm(position) - EARTH_RADIUS_M), launch_altitude)
    density, _pressure, sound_speed = atmosphere(altitude)
    wind_ned = np.asarray(scenario["environment"]["wind_ned_m_s"], dtype=float)
    wind_ecef = ned_to_ecef(wind_ned, position)
    relative_ecef = velocity - wind_ecef
    relative_body = quat_rotate(quat_conjugate(quaternion), relative_ecef)
    aero = estimate(
        relative_body,
        rates,
        density,
        sound_speed,
        body.aerodynamic_stage(),
        fin_commands,
    )
    pitch, yaw = tvc_rad
    cg = float(body.aerodynamic_stage()["center_of_mass_m"])
    thrust_body = np.zeros(3)
    thrust_moment = np.zeros(3)
    for engine in _stage_engines(body.stage):
        engine_thrust = body.last_engine_thrusts_n.get(engine["id"], 0.0)
        direction = np.asarray(engine["direction_body"], dtype=float)
        if engine.get("gimbal_enabled", True):
            direction = direction + np.array([0.0, -yaw, pitch])
        direction = unit(direction, np.array([1.0, 0.0, 0.0]))
        engine_force = engine_thrust * direction
        thrust_body += engine_force
        position = np.asarray(engine["position_body_m"], dtype=float)
        lever = position - np.array([cg, 0.0, 0.0])
        thrust_moment += cross3(lever, engine_force)
    force_body = aero.force_body_n + thrust_body
    force_ecef = quat_rotate(quaternion, force_body)

    recovery = body.stage["recovery"]
    parachute_area = 0.0
    if body.main_deployed:
        parachute_area = float(recovery["main_area_m2"])
    elif body.drogue_deployed:
        parachute_area = float(recovery["drogue_area_m2"])
    if parachute_area > 0.0:
        delay = float(recovery.get("inflation_delay_s", 0.0))
        inflation = (
            min(max((time_s - (body.parachute_deployed_s or time_s)) / max(delay, 0.1), 0.0), 1.0)
            if delay > 0.0
            else 1.0
        )
        speed = float(np.linalg.norm(relative_ecef))
        force_ecef += (
            -0.5
            * density
            * speed**2
            * 1.5
            * parachute_area
            * inflation
            * unit(relative_ecef)
        )

    return force_ecef, aero, aero.moment_body_nm + thrust_moment


def _derivative(
    body: Body,
    scenario: dict[str, Any],
    state: np.ndarray,
    thrust_n: float,
    tvc_rad: np.ndarray,
    fin_commands: np.ndarray,
    time_s: float,
) -> np.ndarray:
    position = state[0:3]
    velocity = state[3:6]
    quaternion = quat_normalize(state[6:10])
    rates = state[10:13]
    force_ecef, _aero, moment_body = _forces(
        body, scenario, state, thrust_n, tvc_rad, fin_commands, time_s
    )
    radius = max(float(np.linalg.norm(position)), EARTH_RADIUS_M)
    earth_rate = np.array([0.0, 0.0, EARTH_ROTATION_RAD_S])
    earth_rate_body = quat_rotate(quat_conjugate(quaternion), earth_rate)
    gravity = -EARTH_MU * position / radius**3
    rotating_terms = -2.0 * cross3(earth_rate, velocity) - cross3(
        earth_rate, cross3(earth_rate, position)
    )
    acceleration = gravity + rotating_terms + force_ecef / body.mass_kg
    inertia = body.inertia_kg_m2
    rates_dot = (
        moment_body - cross3(rates, inertia * rates)
    ) / inertia
    return np.concatenate(
        (
            velocity,
            acceleration,
            quat_derivative(quaternion, rates - earth_rate_body),
            rates_dot,
        )
    )


def _integrate_body(
    body: Body,
    scenario: dict[str, Any],
    thrust_n: float,
    tvc_rad: np.ndarray,
    fin_commands: np.ndarray,
    time_s: float,
    dt_s: float,
    launch_position: np.ndarray,
) -> AeroResult:
    if body.landed:
        return AeroResult(np.zeros(3), np.zeros(3), 0.0, 0.0, 0.0, True)
    rail = scenario["environment"]["launch_rail"]
    constrained_to_rail = (
        body.upper_mass_kg > 0.0 and body.rail_exit_s is None
    )
    rail_attitude = initial_attitude(
        launch_position,
        scenario["environment"]["launch_azimuth_deg"],
    )
    rail_axis = quat_rotate(
        rail_attitude, np.array([1.0, 0.0, 0.0])
    )
    if constrained_to_rail and body.hold_down_released_s is None:
        gravity_m_s2 = EARTH_MU / float(np.linalg.norm(launch_position)) ** 2
        release = rail["hold_down_release"]
        if (
            thrust_n
            < float(release["minimum_thrust_to_weight"])
            * body.mass_kg
            * gravity_m_s2
        ):
            body.position_ecef_m = launch_position.copy()
            body.velocity_ecef_m_s[:] = 0.0
            body.attitude_wxyz = rail_attitude
            body.body_rates_rad_s = quat_rotate(
                quat_conjugate(rail_attitude),
                np.array([0.0, 0.0, EARTH_ROTATION_RAD_S]),
            )
            _force, aero, _moment = _forces(
                body,
                scenario,
                np.concatenate(
                    (
                        body.position_ecef_m,
                        body.velocity_ecef_m_s,
                        body.attitude_wxyz,
                        body.body_rates_rad_s,
                    )
                ),
                thrust_n,
                tvc_rad,
                fin_commands,
                time_s,
            )
            body.last_dynamic_pressure_pa = aero.dynamic_pressure_pa
            body.last_mach = aero.mach
            body.last_angle_of_attack_deg = aero.angle_of_attack_deg
            body.aero_valid = aero.valid
            return aero
        body.hold_down_released_s = time_s
    state = np.concatenate(
        (
            body.position_ecef_m,
            body.velocity_ecef_m_s,
            body.attitude_wxyz,
            body.body_rates_rad_s,
        )
    )
    derivative = lambda value, offset: _derivative(  # noqa: E731
        body,
        scenario,
        value,
        thrust_n,
        tvc_rad,
        fin_commands,
        time_s + offset,
    )
    k1 = derivative(state, 0.0)
    k2 = derivative(state + 0.5 * dt_s * k1, 0.5 * dt_s)
    k3 = derivative(state + 0.5 * dt_s * k2, 0.5 * dt_s)
    k4 = derivative(state + dt_s * k3, dt_s)
    next_state = state + dt_s * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
    next_state[6:10] = quat_normalize(next_state[6:10])

    if constrained_to_rail:
        progress_m = max(
            float(np.dot(next_state[0:3] - launch_position, rail_axis)),
            0.0,
        )
        normal_acceleration = float(
            np.linalg.norm(
                k1[3:6] - np.dot(k1[3:6], rail_axis) * rail_axis
            )
        )
        friction_acceleration = (
            float(rail["friction_coefficient"]) * normal_acceleration
        )
        axial_speed = max(
            float(np.dot(next_state[3:6], rail_axis))
            - friction_acceleration * dt_s,
            0.0,
        )
        progress_m = max(
            progress_m - 0.5 * friction_acceleration * dt_s**2,
            0.0,
        )
        next_state[0:3] = launch_position + progress_m * rail_axis
        next_state[3:6] = axial_speed * rail_axis
        next_state[6:10] = rail_attitude
        next_state[10:13] = quat_rotate(
            quat_conjugate(rail_attitude),
            np.array([0.0, 0.0, EARTH_ROTATION_RAD_S]),
        )
        last_button_m = min(float(value) for value in rail["button_positions_m"])
        if progress_m + last_button_m >= float(rail["length_m"]):
            body.rail_exit_s = time_s + dt_s

    body.position_ecef_m = next_state[0:3]
    body.velocity_ecef_m_s = next_state[3:6]
    body.attitude_wxyz = next_state[6:10]
    body.body_rates_rad_s = next_state[10:13]
    force_ecef, aero, _moment = _forces(
        body, scenario, next_state, thrust_n, tvc_rad, fin_commands, time_s + dt_s
    )
    body.last_specific_force_body_m_s2 = quat_rotate(
        quat_conjugate(body.attitude_wxyz), force_ecef / body.mass_kg
    )
    body.last_dynamic_pressure_pa = aero.dynamic_pressure_pa
    body.last_mach = aero.mach
    body.last_angle_of_attack_deg = aero.angle_of_attack_deg
    body.aero_valid = aero.valid

    if (
        np.linalg.norm(body.position_ecef_m) <= np.linalg.norm(launch_position)
        and np.dot(body.velocity_ecef_m_s, unit(body.position_ecef_m)) < 0.0
        and time_s > 2.0
    ):
        body.position_ecef_m = unit(body.position_ecef_m) * np.linalg.norm(launch_position)
        body.velocity_ecef_m_s[:] = 0.0
        body.body_rates_rad_s[:] = 0.0
        body.landed = True
    return aero
