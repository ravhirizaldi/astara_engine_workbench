"""Quaternion and coordinate helpers."""

from __future__ import annotations

import math

import numpy as np

EARTH_RADIUS_M = 6_378_137.0
EARTH_ROTATION_RAD_S = 7.292115e-5
EARTH_MU = 3.986004418e14


def unit(vector: np.ndarray, fallback: np.ndarray | None = None) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm > 1e-12:
        return vector / norm
    if fallback is not None:
        return fallback.copy()
    return np.zeros_like(vector)


def cross3(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Cross product specialized for hot-path three-element vectors."""
    return np.array(
        [
            left[1] * right[2] - left[2] * right[1],
            left[2] * right[0] - left[0] * right[2],
            left[0] * right[1] - left[1] * right[0],
        ]
    )


def quat_normalize(quaternion: np.ndarray) -> np.ndarray:
    return unit(quaternion, np.array([1.0, 0.0, 0.0, 0.0]))


def quat_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=float,
    )


def quat_conjugate(quaternion: np.ndarray) -> np.ndarray:
    return quaternion * np.array([1.0, -1.0, -1.0, -1.0])


def quat_rotate(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    pure = np.array([0.0, *vector], dtype=float)
    return quat_multiply(
        quat_multiply(quaternion, pure), quat_conjugate(quaternion)
    )[1:]


def quat_derivative(quaternion: np.ndarray, body_rates: np.ndarray) -> np.ndarray:
    return 0.5 * quat_multiply(quaternion, np.array([0.0, *body_rates]))


def quat_from_matrix(matrix: np.ndarray) -> np.ndarray:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                0.25 * scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                [
                    (matrix[2, 1] - matrix[1, 2]) / scale,
                    0.25 * scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                ]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.array(
                [
                    (matrix[0, 2] - matrix[2, 0]) / scale,
                    (matrix[0, 1] + matrix[1, 0]) / scale,
                    0.25 * scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                ]
            )
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.array(
                [
                    (matrix[1, 0] - matrix[0, 1]) / scale,
                    (matrix[0, 2] + matrix[2, 0]) / scale,
                    (matrix[1, 2] + matrix[2, 1]) / scale,
                    0.25 * scale,
                ]
            )
    return quat_normalize(quaternion)


def quat_to_euler(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_normalize(quaternion)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def geodetic_to_ecef(latitude_deg: float, longitude_deg: float, altitude_m: float) -> np.ndarray:
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


def ned_basis(position_ecef: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
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
