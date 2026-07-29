"""Version identifiers and compatibility aliases for persisted documents."""

from __future__ import annotations

import warnings
from pathlib import Path
from typing import Any

SCENARIO_SCHEMA_VERSION = "aerospace-workbench.scenario.v1"
VEHICLE_SCHEMA_VERSION = "aerospace-workbench.vehicle.v1"
RUN_SCHEMA_VERSION = "aerospace-workbench.run-manifest.v1"
SENSOR_STREAM_SCHEMA_VERSION = "aerospace-workbench.sensor-stream.v1"
CREDIBILITY_SCHEMA_VERSION = "aerospace-workbench.credibility.v2"
ROCKETPY_REFERENCE_SCHEMA_VERSION = "aerospace-workbench.rocketpy-reference.v1"

LEGACY_SCENARIO_SCHEMA_VERSION = "astara.scenario.v0"

SCHEMA_MIGRATIONS = {
    LEGACY_SCENARIO_SCHEMA_VERSION: SCENARIO_SCHEMA_VERSION,
    "astara.scenario.v1": SCENARIO_SCHEMA_VERSION,
    "c1.scenario.v1": SCENARIO_SCHEMA_VERSION,
    "astara.vehicle.v1": VEHICLE_SCHEMA_VERSION,
    "astara.run.v1": RUN_SCHEMA_VERSION,
    "astara.sensor-stream.v1": SENSOR_STREAM_SCHEMA_VERSION,
    "astara.credibility.v2": CREDIBILITY_SCHEMA_VERSION,
    "astara.rocketpy-reference.v1": ROCKETPY_REFERENCE_SCHEMA_VERSION,
}

VEHICLE_KEYS = ("vehicle", "sensors", "actuators")


class SchemaMigrationWarning(FutureWarning):
    """A persisted schema or path is supported but deprecated."""


def normalize_schema_identifier(
    identifier: Any,
    expected: str,
    source: str | Path | None = None,
) -> str:
    """Return the canonical identifier or reject an unsupported schema."""
    if identifier == expected:
        return expected
    normalized = (
        SCHEMA_MIGRATIONS.get(identifier)
        if isinstance(identifier, str)
        else None
    )
    if normalized != expected:
        raise ValueError(f"schema_version must be {expected!r}, got {identifier!r}")
    location = f" in {source}" if source is not None else ""
    warnings.warn(
        f"schema_version {identifier!r}{location} is deprecated; "
        f"use {expected!r}",
        SchemaMigrationWarning,
        stacklevel=2,
    )
    return expected


def normalize_schema_document(
    document: dict[str, Any],
    expected: str,
    source: str | Path | None = None,
) -> str:
    """Normalize a document in place and return its source identifier."""
    original = document.get("schema_version")
    document["schema_version"] = normalize_schema_identifier(
        original, expected, source
    )
    return str(original)
