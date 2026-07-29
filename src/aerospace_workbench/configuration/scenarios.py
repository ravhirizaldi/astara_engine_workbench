"""Scenario loading, hashing, and mission schedule access."""

from __future__ import annotations

import copy
import hashlib
import json
import math
import warnings
from pathlib import Path
from typing import Any

from .schemas import (
    LEGACY_SCENARIO_SCHEMA_VERSION,
    SCENARIO_SCHEMA_VERSION,
    SchemaMigrationWarning,
    VEHICLE_KEYS,
    normalize_schema_document,
)
from .vehicles import (
    load_vehicle_definition,
    merge_vehicle_definition,
)


class _LoadedScenario(dict[str, Any]):
    """Runtime scenario carrying non-serialized input provenance."""

    source_files: dict[str, Path]
    vehicle_document: dict[str, Any] | None

    def __init__(
        self,
        document: dict[str, Any],
        source_files: dict[str, Path],
        vehicle_document: dict[str, Any] | None,
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
    """Resolve a scenario path, including the deprecated repository location."""
    if path is None:
        return default_scenario_path()
    candidate = Path(path).expanduser()
    if candidate.exists():
        return candidate.resolve()

    root = _repository_root()
    mapped: Path | None = None
    if not candidate.is_absolute() and candidate.parts[:1] == ("scenarios",):
        mapped = root / "configs" / candidate
    else:
        absolute = candidate.resolve()
        legacy_root = root / "scenarios"
        try:
            mapped = root / "configs" / "scenarios" / absolute.relative_to(
                legacy_root
            )
        except ValueError:
            pass
    if mapped is not None and mapped.exists():
        warnings.warn(
            f"scenario path {path!s} is deprecated; use {mapped}",
            SchemaMigrationWarning,
            stacklevel=2,
        )
        return mapped.resolve()
    return candidate.resolve()


def load_scenario_documents(
    path: str | Path | None = None,
) -> tuple[dict[str, Any], dict[str, Any] | None, Path | None]:
    source = resolve_scenario_path(path)
    scenario = json.loads(source.read_text(encoding="utf-8"))
    source_schema = normalize_schema_document(
        scenario, SCENARIO_SCHEMA_VERSION, source
    )
    vehicle, vehicle_path = load_vehicle_definition(
        scenario,
        source,
        allow_inline=source_schema == LEGACY_SCENARIO_SCHEMA_VERSION,
    )
    return scenario, vehicle, vehicle_path


def load_scenario(path: str | Path | None = None) -> dict[str, Any]:
    source = resolve_scenario_path(path)
    scenario, vehicle, vehicle_path = load_scenario_documents(source)
    loaded = _LoadedScenario(
        scenario,
        {
            "scenario": source,
            **({"vehicle": vehicle_path} if vehicle_path is not None else {}),
        },
        vehicle,
    )
    merge_vehicle_definition(loaded, vehicle)
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
    """Resolve mission event chains into absolute simulation times."""
    mission = scenario.get("mission", {})
    stage1_burn_s = float(
        scenario["vehicle"]["stages"][0]["propulsion"]["burn_duration_s"]
    )
    definitions = mission.get("events")
    if definitions is None:
        separation = stage1_burn_s + float(mission["separation_delay_s"])
        return {
            "burnout_stage_1": stage1_burn_s,
            "stage_separation": separation,
            "stage2_ignition": separation
            + float(mission["stage2_ignition_delay_s"]),
        }
    if not isinstance(definitions, list) or not definitions:
        raise ValueError("mission.events must contain at least one event")

    pending: dict[str, tuple[str, float]] = {}
    for index, definition in enumerate(definitions):
        prefix = f"mission.events[{index}]"
        if not isinstance(definition, dict):
            raise ValueError(f"{prefix} must be an object")
        event = definition.get("event")
        trigger = definition.get("trigger")
        delay = definition.get("delay")
        if not isinstance(event, str) or not event:
            raise ValueError(f"{prefix}.event must be a nonempty string")
        if not isinstance(trigger, str) or not trigger:
            raise ValueError(f"{prefix}.trigger must be a nonempty string")
        if event == "burnout_stage_1" or event in pending:
            raise ValueError(
                f"mission.events contains duplicate event {event!r}"
            )
        if (
            not isinstance(delay, (int, float))
            or not math.isfinite(delay)
            or delay < 0.0
        ):
            raise ValueError(f"{prefix}.delay must be finite and nonnegative")
        pending[event] = (trigger, float(delay))

    resolved = {"burnout_stage_1": stage1_burn_s}
    while pending:
        ready = [
            event
            for event, (trigger, _delay) in pending.items()
            if trigger in resolved
        ]
        if not ready:
            unresolved = ", ".join(sorted(pending))
            raise ValueError(
                f"mission.events has cyclic or unknown triggers: {unresolved}"
            )
        for event in ready:
            trigger, delay = pending.pop(event)
            resolved[event] = resolved[trigger] + delay

    for required in ("stage_separation", "stage2_ignition"):
        if required not in resolved:
            raise ValueError(
                f"mission.events is missing required event {required!r}"
            )
    if resolved["stage2_ignition"] < resolved["stage_separation"]:
        raise ValueError(
            "stage2_ignition cannot occur before stage_separation"
        )
    return resolved


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
