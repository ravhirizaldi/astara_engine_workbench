"""Scenario loading, hashing, and mission schedule access."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
from typing import Any

from .schemas import (
    SCENARIO_SCHEMA_VERSION,
    VEHICLE_KEYS,
    require_schema_version,
)
from .vehicles import (
    load_vehicle_definition,
    merge_vehicle_definition,
)


class _LoadedScenario(dict[str, Any]):
    """Runtime scenario carrying non-serialized input provenance."""

    source_files: dict[str, Path]
    vehicle_document: dict[str, Any]

    def __init__(
        self,
        document: dict[str, Any],
        source_files: dict[str, Path],
        vehicle_document: dict[str, Any],
    ) -> None:
        super().__init__(document)
        self.source_files = source_files
        self.vehicle_document = copy.deepcopy(vehicle_document)


def _repository_root() -> Path:
    return Path(__file__).resolve().parents[3]


def default_scenario_path() -> Path:
    return (
        _repository_root()
        / "configs"
        / "scenarios"
        / "anthariksa_reference_mission.json"
    )


def resolve_scenario_path(path: str | Path | None = None) -> Path:
    """Resolve a scenario path."""
    if path is None:
        return default_scenario_path()
    return Path(path).expanduser().resolve()


def load_scenario_documents(
    path: str | Path | None = None,
) -> tuple[dict[str, Any], dict[str, Any], Path]:
    source = resolve_scenario_path(path)
    scenario = json.loads(source.read_text(encoding="utf-8"))
    require_schema_version(scenario, SCENARIO_SCHEMA_VERSION, source)
    vehicle, vehicle_path = load_vehicle_definition(scenario, source)
    return scenario, vehicle, vehicle_path


def load_scenario(path: str | Path | None = None) -> dict[str, Any]:
    source = resolve_scenario_path(path)
    scenario, vehicle, vehicle_path = load_scenario_documents(source)
    return scenario_from_documents(
        scenario,
        vehicle,
        {"scenario": source, "vehicle": vehicle_path},
    )


def scenario_from_documents(
    scenario_document: dict[str, Any],
    vehicle_document: dict[str, Any],
    source_files: dict[str, Path] | None = None,
) -> dict[str, Any]:
    """Build one validated runtime scenario from editable source documents."""
    loaded = _LoadedScenario(
        copy.deepcopy(scenario_document),
        dict(source_files or {}),
        vehicle_document,
    )
    require_schema_version(loaded, SCENARIO_SCHEMA_VERSION)
    merge_vehicle_definition(loaded, copy.deepcopy(vehicle_document))
    from .validation import validate_scenario

    validate_scenario(loaded)
    return loaded


def default_scenario() -> dict[str, Any]:
    return copy.deepcopy(load_scenario())


def scenario_hash(scenario: dict[str, Any]) -> str:
    canonical = copy.deepcopy(scenario)
    if canonical.get("vehicle_definition") and all(
        key in canonical for key in VEHICLE_KEYS
    ):
        source_vehicle = getattr(scenario, "vehicle_document", None)
        vehicle = copy.deepcopy(source_vehicle) if source_vehicle else {}
        for key in VEHICLE_KEYS:
            vehicle[key] = canonical.pop(key)
        canonical.pop("vehicle_definition")
        canonical["vehicle_definition_sha256"] = hashlib.sha256(
            json.dumps(vehicle, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
    encoded = json.dumps(canonical, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def configuration_source_files(scenario: dict[str, Any]) -> dict[str, Path]:
    """Return file-backed input provenance without serializing it."""
    return dict(getattr(scenario, "source_files", {}))


def resolve_mission_events(scenario: dict[str, Any]) -> dict[str, float]:
    """Return nominal FSW transition times from configured policy."""
    mission = scenario.get("mission", {})
    stage1_burn_s = float(
        scenario["vehicle"]["stages"][0]["propulsion"]["burn_duration_s"]
    )
    policy = mission.get("flight_core", {})
    separation_delay_s = float(policy["separation_delay_s"])
    ignition_delay_s = float(policy["stage2_ignition_delay_s"])
    return {
        "burnout_stage_1": stage1_burn_s,
        "stage_separation": stage1_burn_s + separation_delay_s,
        "stage2_ignition": (
            stage1_burn_s + separation_delay_s + ignition_delay_s
        ),
    }


def mission_timeline(scenario: dict[str, Any]) -> list[dict[str, Any]]:
    """Return the validated typed mission timeline."""
    return list(scenario["mission"]["timeline"])


def model_source_hash() -> str:
    root = _repository_root()
    paths = sorted((root / "src" / "aerospace_workbench").rglob("*.py"))
    flight_core = root / "flight_core"
    paths.extend(sorted((flight_core / "include").rglob("*.h")))
    paths.extend(sorted((flight_core / "src").rglob("*.hpp")))
    paths.extend(sorted((flight_core / "src").rglob("*.cpp")))
    digest = hashlib.sha256()
    for path in paths:
        digest.update(str(path.relative_to(root)).encode())
        digest.update(b"\0")
        digest.update(path.read_bytes())
    return digest.hexdigest()
