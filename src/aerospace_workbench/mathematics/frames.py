"""Earth-fixed and local navigation frame conversions."""

from __future__ import annotations

import math

import numpy as np

from .quaternions import quat_from_matrix
from .vectors import cross3, unit

EARTH_RADIUS_M = 6_378_137.0
EARTH_ROTATION_RAD_S = 7.292115e-5
EARTH_MU = 3.986004418e14


def geodetic_to_ecef(
    latitude_deg: float, longitude_deg: float, altitude_m: float
) -> np.ndarray:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    radius = EARTH_RADIUS_M + altitude_m
    return radius * np.array(
        [
            math.cos(latitude) * math.cos(longitude),
            math.cos(latitude) * math.sin(longitude),
            math.sin(latitude),
        ]
    )


def ecef_to_geodetic(position_ecef: np.ndarray) -> tuple[float, float, float]:
    radius = float(np.linalg.norm(position_ecef))
    if radius <= 1e-12:
        raise ValueError("ECEF position must not be zero")
    latitude_deg = math.degrees(
        math.asin(max(-1.0, min(1.0, float(position_ecef[2]) / radius)))
    )
    longitude_deg = math.degrees(
        math.atan2(float(position_ecef[1]), float(position_ecef[0]))
    )
    return latitude_deg, longitude_deg, radius - EARTH_RADIUS_M


def ned_basis(
    position_ecef: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    up = unit(position_ecef)
    longitude = math.atan2(position_ecef[1], position_ecef[0])
    east = np.array([-math.sin(longitude), math.cos(longitude), 0.0])
    north = unit(cross3(up, east))
    down = -up
    return north, east, down


def initial_attitude(position_ecef: np.ndarray, azimuth_deg: float) -> np.ndarray:
    north, east, down = ned_basis(position_ecef)
    up = -down
    azimuth = math.radians(azimuth_deg)
    body_x = up
    body_y = math.cos(azimuth) * east - math.sin(azimuth) * north
    body_z = unit(cross3(body_x, body_y))
    return quat_from_matrix(np.column_stack((body_x, body_y, body_z)))


def ecef_to_ned(vector_ecef: np.ndarray, position_ecef: np.ndarray) -> np.ndarray:
    north, east, down = ned_basis(position_ecef)
    return np.array(
        [
            float(np.dot(vector_ecef, north)),
            float(np.dot(vector_ecef, east)),
            float(np.dot(vector_ecef, down)),
        ]
    )


def ned_to_ecef(vector_ned: np.ndarray, position_ecef: np.ndarray) -> np.ndarray:
    north, east, down = ned_basis(position_ecef)
    return vector_ned[0] * north + vector_ned[1] * east + vector_ned[2] * down
