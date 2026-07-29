"""Vehicle-definition loading, merging, and evidence snapshots."""

from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any

from .schemas import (
    SCENARIO_SCHEMA_VERSION,
    VEHICLE_KEYS,
    VEHICLE_SCHEMA_VERSION,
)


def load_vehicle_definition(
    scenario: dict[str, Any], scenario_path: Path
) -> tuple[dict[str, Any] | None, Path | None]:
    reference = scenario.get("vehicle_definition")
    if reference is None:
        return None, None
    if not isinstance(reference, str) or not reference:
        raise ValueError("vehicle_definition must be a nonempty path string")
    vehicle_path = (scenario_path.parent / reference).resolve()
    vehicle = json.loads(vehicle_path.read_text(encoding="utf-8"))
    if vehicle.get("schema_version") != VEHICLE_SCHEMA_VERSION:
        raise ValueError(
            f"vehicle schema_version must be {VEHICLE_SCHEMA_VERSION!r}"
        )
    for key in VEHICLE_KEYS:
        if key not in vehicle:
            raise ValueError(f"vehicle definition is missing {key!r}")
    return vehicle, vehicle_path


def merge_vehicle_definition(
    scenario: dict[str, Any], vehicle: dict[str, Any] | None
) -> dict[str, Any]:
    if vehicle:
        for key in VEHICLE_KEYS:
            if key in scenario:
                raise ValueError(
                    f"scenario must not override vehicle definition key {key!r}"
                )
            scenario[key] = copy.deepcopy(vehicle[key])
    return scenario


def evidence_documents(
    scenario: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    scenario_document = copy.deepcopy(scenario)
    if scenario_document.get("schema_version") != SCENARIO_SCHEMA_VERSION:
        return scenario_document, None
    vehicle_document = {
        "schema_version": VEHICLE_SCHEMA_VERSION,
        "name": Path(str(scenario_document["vehicle_definition"])).stem,
        **{key: scenario_document.pop(key) for key in VEHICLE_KEYS},
    }
    scenario_document["vehicle_definition"] = "vehicle_definition.json"
    return scenario_document, vehicle_document
