"""Small vector operations shared by the physical models."""

from __future__ import annotations

import numpy as np


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
