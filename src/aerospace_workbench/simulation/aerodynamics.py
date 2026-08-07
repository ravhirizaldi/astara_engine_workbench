"""Aerodynamic estimates for the software-in-the-loop truth model."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class AeroResult:
    force_body_n: np.ndarray
    moment_body_nm: np.ndarray
    mach: float
    angle_of_attack_deg: float
    dynamic_pressure_pa: float
    valid: bool


def atmosphere(altitude_m: float) -> tuple[float, float, float]:
    """Return density, pressure, and sound speed using a layered ISA approximation."""
    altitude = max(0.0, float(altitude_m))
    if altitude > 100_000.0:
        # ponytail: vacuum above the ISA ceiling; add an exosphere model only
        # when rarefied-flow fidelity is required.
        return 0.0, 0.0, math.sqrt(1.4 * 287.05287 * 186.946)
    layers = (
        (0.0, 288.15, 101_325.0, -0.0065),
        (11_000.0, 216.65, 22_632.1, 0.0),
        (20_000.0, 216.65, 5_474.89, 0.0010),
        (32_000.0, 228.65, 868.019, 0.0028),
        (47_000.0, 270.65, 110.906, 0.0),
        (51_000.0, 270.65, 66.9389, -0.0028),
        (71_000.0, 214.65, 3.95642, -0.0020),
        (84_852.0, 186.946, 0.3734, 0.0),
    )
    gas_constant = 287.05287
    gravity = 9.80665
    base_altitude, base_temp, base_pressure, lapse = layers[0]
    for candidate in layers:
        if candidate[0] <= altitude:
            base_altitude, base_temp, base_pressure, lapse = candidate
        else:
            break
    delta = altitude - base_altitude
    if lapse == 0.0:
        temperature = base_temp
        pressure = base_pressure * math.exp(-gravity * delta / (gas_constant * temperature))
    else:
        temperature = base_temp + lapse * delta
        pressure = base_pressure * (temperature / base_temp) ** (
            -gravity / (lapse * gas_constant)
        )
    density = pressure / (gas_constant * temperature)
    sound_speed = math.sqrt(1.4 * gas_constant * temperature)
    return density, pressure, sound_speed


def estimate(
    relative_velocity_body_m_s: np.ndarray,
    body_rates_rad_s: np.ndarray,
    density_kg_m3: float,
    sound_speed_m_s: float,
    stage: dict,
    fin_commands: np.ndarray,
) -> AeroResult:
    speed = float(np.linalg.norm(relative_velocity_body_m_s))
    if speed < 0.1:
        return AeroResult(np.zeros(3), np.zeros(3), 0.0, 0.0, 0.0, True)

    diameter = float(stage["diameter_m"])
    length = float(stage["length_m"])
    area = math.pi * diameter**2 / 4.0
    mach = speed / max(sound_speed_m_s, 1.0)
    dynamic_pressure = 0.5 * density_kg_m3 * speed**2
    axial = max(abs(relative_velocity_body_m_s[0]), 1e-9)
    alpha = math.atan2(relative_velocity_body_m_s[2], axial)
    beta = math.atan2(relative_velocity_body_m_s[1], axial)
    angle = math.degrees(math.hypot(alpha, beta))

    fin = stage.get("aerodynamics", {})
    fin_area = float(fin.get("fin_area_m2", area * 0.35))
    fin_count = float(fin.get("fin_count", 4))
    table = fin.get("coefficient_table")
    if table:
        mach_points = [row["mach"] for row in table]
        base_cd = float(
            np.interp(mach, mach_points, [row["drag_coefficient"] for row in table])
        )
        normal_slope = float(
            np.interp(
                mach,
                mach_points,
                [row["normal_force_slope_per_rad"] for row in table],
            )
        )
        damping = float(
            np.interp(
                mach,
                mach_points,
                [row["pitch_damping_coefficient"] for row in table],
            )
        )
        control_coefficient = float(
            np.interp(
                mach,
                mach_points,
                [row["control_force_coefficient"] for row in table],
            )
        )
    else:
        base_cd = float(fin.get("base_drag_coefficient", 0.38))
        base_cd += 0.22 * math.exp(-((mach - 1.05) / 0.32) ** 2)
        base_cd += 0.05 * max(mach - 1.0, 0.0) ** 1.25
        normal_slope = 2.0 + 0.6 * fin_count * fin_area / max(area, 1e-9)
        damping = 0.08
        control_coefficient = 2.4
    cd = base_cd + float(fin.get("induced_drag_factor", 1.8)) * (
        alpha * alpha + beta * beta
    )
    drag = dynamic_pressure * area * cd
    side = -dynamic_pressure * area * normal_slope * beta
    normal = -dynamic_pressure * area * normal_slope * alpha
    force = np.array([-drag, side, normal])

    cp_from_nose = float(fin.get("center_of_pressure_m", length * 0.68))
    cg_from_nose = float(stage.get("center_of_mass_m", length * 0.52))
    lever = cp_from_nose - cg_from_nose
    moment = np.array(
        [
            -damping
            * dynamic_pressure
            * area
            * length**2
            * body_rates_rad_s[0]
            / speed,
            lever * normal,
            -lever * side,
        ]
    )

    commands = np.clip(np.asarray(fin_commands, dtype=float), -1.0, 1.0)
    control_force = dynamic_pressure * fin_area * control_coefficient
    if commands.size >= 3:
        moment += np.array(
            [
                commands[0] * control_force * diameter,
                commands[1] * control_force * length,
                commands[2] * control_force * length,
            ]
        )
    valid = (
        float(fin.get("valid_mach_min", 0.0))
        <= mach
        <= float(fin.get("valid_mach_max", 5.0))
        and angle <= float(fin.get("valid_angle_of_attack_deg", 20.0))
    )
    return AeroResult(force, moment, mach, angle, dynamic_pressure, valid)
