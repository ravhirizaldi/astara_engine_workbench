"""Deterministic policy for retaining Monte Carlo telemetry."""

from __future__ import annotations

import math
from typing import Any

import numpy as np


def same_metric(left: Any, right: Any) -> bool:
    if left is None or right is None:
        return left is right
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return math.isclose(
            float(left), float(right), rel_tol=0.0, abs_tol=1e-12
        )
    return left == right


def selected_success_samples(
    rows: list[dict[str, Any]],
    seed: int,
    percent: float,
    success_statuses: frozenset[str],
) -> set[int]:
    successful = [
        int(row["sample"])
        for row in rows
        if row["status"] in success_statuses
    ]
    if not successful or percent <= 0.0:
        return set()
    count = min(
        len(successful),
        max(1, math.ceil(len(successful) * percent / 100.0)),
    )
    selection_rng = np.random.default_rng(seed ^ 0x5A17C0DE)
    return {
        int(value)
        for value in selection_rng.choice(successful, size=count, replace=False)
    }
