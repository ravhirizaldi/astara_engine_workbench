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
    normalize_schema_document,
)


def load_vehicle_definition(
    scenario: dict[str, Any],
    scenario_path: Path,
    *,
    allow_inline: bool = False,
) -> tuple[dict[str, Any] | None, Path | None]:
    reference = scenario.get("vehicle_definition")
    if reference is None:
        if allow_inline:
            return None, None
        raise ValueError("vehicle_definition is required")
    if not isinstance(reference, str) or not reference:
        raise ValueError("vehicle_definition must be a nonempty path string")
    vehicle_path = (scenario_path.parent / reference).resolve()
    vehicle = json.loads(vehicle_path.read_text(encoding="utf-8"))
    normalize_schema_document(vehicle, VEHICLE_SCHEMA_VERSION, vehicle_path)
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
    normalize_schema_document(scenario_document, SCENARIO_SCHEMA_VERSION)
    if not all(key in scenario_document for key in VEHICLE_KEYS):
        return scenario_document, None
    reference = scenario_document.get("vehicle_definition", "inline_vehicle")
    source_vehicle = getattr(scenario, "vehicle_document", None)
    vehicle_document = copy.deepcopy(source_vehicle) if source_vehicle else {
        "schema_version": VEHICLE_SCHEMA_VERSION,
        "name": Path(str(reference)).stem,
    }
    vehicle_document["schema_version"] = VEHICLE_SCHEMA_VERSION
    for key in VEHICLE_KEYS:
        vehicle_document[key] = scenario_document.pop(key)
    scenario_document["vehicle_definition"] = "vehicle_definition.json"
    return scenario_document, vehicle_document
