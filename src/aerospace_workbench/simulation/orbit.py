"""Osculating two-body orbit evidence from ECEF truth state."""

from __future__ import annotations

import math

import numpy as np

from ..mathematics.frames import EARTH_MU, EARTH_RADIUS_M, EARTH_ROTATION_RAD_S


def orbital_elements(
    position_ecef_m: np.ndarray,
    velocity_ecef_m_s: np.ndarray,
) -> dict[str, float]:
    position = np.asarray(position_ecef_m, dtype=float)
    earth_rate = np.array([0.0, 0.0, EARTH_ROTATION_RAD_S])
    velocity = np.asarray(velocity_ecef_m_s, dtype=float) + np.cross(
        earth_rate, position
    )
    radius_m = float(np.linalg.norm(position))
    speed_m_s = float(np.linalg.norm(velocity))
    angular_momentum = np.cross(position, velocity)
    angular_momentum_norm = float(np.linalg.norm(angular_momentum))
    eccentricity_vector = (
        np.cross(velocity, angular_momentum) / EARTH_MU
        - position / radius_m
    )
    eccentricity = float(np.linalg.norm(eccentricity_vector))
    energy = 0.5 * speed_m_s**2 - EARTH_MU / radius_m
    semi_major_axis_m = -EARTH_MU / (2.0 * energy)
    return {
        "semi_major_axis_m": semi_major_axis_m,
        "eccentricity": eccentricity,
        "inclination_deg": math.degrees(
            math.acos(
                min(
                    max(
                        angular_momentum[2] / angular_momentum_norm,
                        -1.0,
                    ),
                    1.0,
                )
            )
        ),
        "periapsis_altitude_m": (
            semi_major_axis_m * (1.0 - eccentricity) - EARTH_RADIUS_M
        ),
        "apoapsis_altitude_m": (
            semi_major_axis_m * (1.0 + eccentricity) - EARTH_RADIUS_M
        ),
        "inertial_speed_m_s": speed_m_s,
        "radial_velocity_m_s": float(np.dot(position, velocity) / radius_m),
    }
